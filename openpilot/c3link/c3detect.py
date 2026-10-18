"""C3 即時偵測器：VisionIPC road 影像 -> ONNX 推論 -> 共享記憶體給 Qt 畫面。

只負責取幀與推論，不畫框也不編碼 JPEG（那是 c3_screen 的事）。
輸出兩個檔到 /dev/shm/c3det/：
  frame.rgb  顯示用 RGB888，固定 DISP_W x DISP_H，原子替換（C3DET_FRAME=1 才寫；畫面已停用）
  det.json   偵測框（座標already在顯示解析度）+ 狀態數字

2026-10-18 使用者：C3 即時畫面很頓、不需要即時畫面也可以。實查：辨識＋深度每張 577＋650 ms（1.5 張/秒），
畫面只是顯示這裡寫的 frame.rgb 所以跟著頓；8 核被降頻到 1.69 GHz、78–86°C。改成：
  - 深度關掉（框的距離紅綠燈用不到，距離走 OSM 號誌）；4 核（單獨量 317 ms → 335 ms，幾乎一樣、少熱）
  - 只在 C4 發動、透過 USB 線送 carState 來時才辨識（c3calib 寫的 driving），停車時休息
  - 有自動標定（c3calib）時只送地平線以上＋HORIZON_MARGIN 進模型：燈都在地平線上，裁掉下半部模型
    裡的燈從 0.90 倍放大到約 1.0 倍、運算量不變；框座標從頂端算起所以不用換算
"""
import json
import os
import time

import numpy as np
import onnxruntime as ort
from msgq.visionipc import VisionIpcClient, VisionStreamType

MODEL = os.environ.get('C3DET_MODEL', '/data/c3det/models/c4light13_960x544.onnx')
THREADS = int(os.environ.get('C3DET_THREADS', '6'))
CONF_TH = float(os.environ.get('C3DET_CONF', '0.35'))
IOU_TH = 0.45
OUT_DIR = '/dev/shm/c3det'
# 深度模型（空字串 = 關閉）。每 DEPTH_EVERY 幀跑一次，框的距離沿用最近一次深度圖。
DEPTH_MODEL = os.environ.get('C3DET_DEPTH', '')
DEPTH_EVERY = int(os.environ.get('C3DET_DEPTH_EVERY', '2'))
WRITE_FRAME = os.environ.get('C3DET_FRAME', '0') == '1'
ALWAYS = os.environ.get('C3DET_ALWAYS', '0') == '1'      # 測試用：不管有沒有在開車都辨識
CALIB = '/dev/shm/c3calib.json'
HORIZON_MARGIN = 120     # px（半解析度，約 5°）：地平線下多留這麼多
MIN_CROP = 300

# 顯示解析度 = 相機半解析度（NV12 的 UV 剛好是這個尺寸，轉換零插值）
DISP_W, DISP_H = 964, 604

ZH = {
    'person': '人', 'bicycle': '腳踏車', 'car': '汽車', 'motorcycle': '機車',
    'bus': '公車', 'truck': '卡車', 'stop_sign': '停車再開',
    'light_red': '紅燈', 'light_green': '綠燈', 'light_yellow': '黃燈',
    'light_off': '熄燈', 'light_pedestrian': '行人號誌', 'light_green_arrow': '綠箭頭',
    'traffic light': '號誌燈', 'stop sign': '停車再開', 'train': '火車', 'boat': '船',
}
# 畫框顏色（Qt 端直接吃）
COLOR = {
    'person': '#ff4d6d', 'bicycle': '#ffa94d', 'car': '#4dabf7', 'motorcycle': '#ffd43b',
    'bus': '#845ef7', 'truck': '#5c7cfa', 'stop_sign': '#ff6b6b',
    'light_red': '#ff2d2d', 'light_green': '#2dff6b', 'light_yellow': '#ffe02d',
    'light_off': '#868e96', 'light_pedestrian': '#22d3ee', 'light_green_arrow': '#51ff8a',
}
DEFAULT_COLOR = '#adb5bd'


def cpu_temp():
    for p in ('/sys/class/thermal/thermal_zone0/temp',):
        try:
            v = int(open(p).read().strip())
            return v / 1000.0 if v > 1000 else float(v)
        except Exception:
            pass
    return 0.0


def nv12_to_rgb_half(buf):
    """NV12 -> 半解析度 RGB888。UV 本來就是半解析度，所以完全不用插值。"""
    d = np.frombuffer(buf.data, dtype=np.uint8)
    h, w, stride = buf.height, buf.width, buf.stride
    y = d[:h * stride].reshape(h, stride)[:, :w]
    uv = d[buf.uv_offset:buf.uv_offset + (h // 2) * stride].reshape(h // 2, stride)[:, :w]

    yh = y[::2, ::2].astype(np.float32)          # (h/2, w/2)
    u = uv[:, 0::2].astype(np.float32) - 128.0   # (h/2, w/2)
    v = uv[:, 1::2].astype(np.float32) - 128.0

    n = min(yh.shape[0], u.shape[0]), min(yh.shape[1], u.shape[1])
    yh, u, v = yh[:n[0], :n[1]], u[:n[0], :n[1]], v[:n[0], :n[1]]

    rgb = np.empty((n[0], n[1], 3), dtype=np.float32)
    rgb[:, :, 0] = yh + 1.402 * v
    rgb[:, :, 1] = yh - 0.344136 * u - 0.714136 * v
    rgb[:, :, 2] = yh + 1.772 * u
    return np.clip(rgb, 0, 255).astype(np.uint8)


def letterbox(rgb, tw, th):
    """等比縮放 + 置中補邊，回傳 (輸入張量用的影像, scale, pad_x, pad_y)。"""
    from PIL import Image
    h, w = rgb.shape[:2]
    s = min(tw / w, th / h)
    nw, nh = int(round(w * s)), int(round(h * s))
    im = Image.fromarray(rgb).resize((nw, nh), Image.BILINEAR)
    canvas = Image.new('RGB', (tw, th), (114, 114, 114))
    px, py = (tw - nw) // 2, (th - nh) // 2
    canvas.paste(im, (px, py))
    return np.asarray(canvas), s, px, py


def nms(boxes, scores, iou_th):
    idx = scores.argsort()[::-1]
    keep = []
    while idx.size:
        i = idx[0]
        keep.append(i)
        if idx.size == 1:
            break
        xx1 = np.maximum(boxes[i, 0], boxes[idx[1:], 0])
        yy1 = np.maximum(boxes[i, 1], boxes[idx[1:], 1])
        xx2 = np.minimum(boxes[i, 2], boxes[idx[1:], 2])
        yy2 = np.minimum(boxes[i, 3], boxes[idx[1:], 3])
        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        a1 = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        a2 = (boxes[idx[1:], 2] - boxes[idx[1:], 0]) * (boxes[idx[1:], 3] - boxes[idx[1:], 1])
        iou = inter / (a1 + a2 - inter + 1e-9)
        idx = idx[1:][iou < iou_th]
    return keep


def decode(out, names, scale, px, py, disp_w, disp_h):
    """YOLO11 輸出 [1, 4+nc, N] -> 顯示座標的框。"""
    p = out[0]                      # (4+nc, N)
    box = p[:4].T                   # cx, cy, w, h (模型輸入座標)
    cls = p[4:].T                   # (N, nc)
    conf = cls.max(axis=1)
    m = conf > CONF_TH
    if not m.any():
        return []
    box, cls, conf = box[m], cls[m].argmax(axis=1), conf[m]

    xy = np.empty((box.shape[0], 4), dtype=np.float32)
    xy[:, 0] = (box[:, 0] - box[:, 2] / 2 - px) / scale
    xy[:, 1] = (box[:, 1] - box[:, 3] / 2 - py) / scale
    xy[:, 2] = (box[:, 0] + box[:, 2] / 2 - px) / scale
    xy[:, 3] = (box[:, 1] + box[:, 3] / 2 - py) / scale

    dets = []
    for c in np.unique(cls):
        sel = np.where(cls == c)[0]
        for k in nms(xy[sel], conf[sel], IOU_TH):
            i = sel[k]
            name = names.get(int(c), str(int(c)))
            x1 = max(0, min(disp_w - 1, float(xy[i, 0])))
            y1 = max(0, min(disp_h - 1, float(xy[i, 1])))
            x2 = max(0, min(disp_w - 1, float(xy[i, 2])))
            y2 = max(0, min(disp_h - 1, float(xy[i, 3])))
            if x2 - x1 < 3 or y2 - y1 < 3:
                continue
            dets.append({
                'x': round(x1, 1), 'y': round(y1, 1), 'w': round(x2 - x1, 1), 'h': round(y2 - y1, 1),
                'cls': name, 'zh': ZH.get(name, name), 'conf': round(float(conf[i]), 2),
                'color': COLOR.get(name, DEFAULT_COLOR),
            })
    dets.sort(key=lambda d: -d['w'] * d['h'])
    return dets[:40]


def box_distance(dm, d, dscale, dpx, dpy, DH, DW):
    """框內深度的穩健距離：取最近的四分位（車尾／人身比車頂近，中位會被背景拉遠）。"""
    sy, sx = dm.shape[0] / float(DH), dm.shape[1] / float(DW)
    y1 = int((d['y'] * dscale + dpy) * sy)
    y2 = int(((d['y'] + d['h']) * dscale + dpy) * sy)
    x1 = int((d['x'] * dscale + dpx) * sx)
    x2 = int(((d['x'] + d['w']) * dscale + dpx) * sx)
    a = dm[max(0, y1):max(y1 + 1, y2), max(0, x1):max(x1 + 1, x2)]
    if a.size == 0:
        return None
    return round(float(np.percentile(a, 25)), 1)


def calib_state():
    """c3calib 的狀態：(在開車嗎, 要裁到第幾列或 None)。"""
    try:
        with open(CALIB) as f:
            c = json.load(f)
    except (OSError, ValueError):
        return False, None
    if time.monotonic() - float(c.get('t', 0)) > 15:
        return False, None
    hr = c.get('horizon_row')
    crop = None if hr is None else int(max(MIN_CROP, min(DISP_H, hr + HORIZON_MARGIN)))
    return bool(c.get('driving')), crop


def atomic_write(path, data, binary=True):
    tmp = path + '.tmp'
    with open(tmp, 'wb' if binary else 'w') as f:
        f.write(data)
    os.replace(tmp, path)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    opts = ort.SessionOptions()
    opts.intra_op_num_threads = THREADS
    opts.inter_op_num_threads = 1
    opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    sess = ort.InferenceSession(MODEL, opts, providers=['CPUExecutionProvider'])
    inp = sess.get_inputs()[0]
    _, _, MH, MW = [d if isinstance(d, int) else 1 for d in inp.shape]

    meta = sess.get_modelmeta().custom_metadata_map
    names = {int(k): v for k, v in eval(meta.get('names', '{}')).items()}
    model_label = os.path.basename(MODEL).replace('.onnx', '')

    depth_sess = None
    depth_ms = 0.0
    dm = None
    dgeom = None
    dinp = None
    DH = DW = 0
    if DEPTH_MODEL:
        dopts = ort.SessionOptions()
        dopts.intra_op_num_threads = THREADS
        dopts.inter_op_num_threads = 1
        dopts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        depth_sess = ort.InferenceSession(DEPTH_MODEL, dopts, providers=['CPUExecutionProvider'])
        dinp = depth_sess.get_inputs()[0]
        _, _, DH, DW = [d if isinstance(d, int) else 768 for d in dinp.shape]
        model_label += ' + ' + os.path.basename(DEPTH_MODEL).replace('.onnx', '')

    client = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, False)
    state = {'fps': 0.0, 'infer_ms': 0.0, 'depth_ms': 0.0, 'temp': 0.0, 'model': model_label,
             'dets': [], 'counts': {}, 'w': DISP_W, 'h': DISP_H, 'cam': 'road', 'err': 'starting'}
    atomic_write(os.path.join(OUT_DIR, 'det.json'), json.dumps(state, ensure_ascii=False), binary=False)

    t_last = time.monotonic()
    fps_ema = 0.0
    frame_i = 0
    last_check = 0.0
    driving, crop = False, None

    while True:
        now = time.monotonic()
        if now - last_check > 1.0:
            last_check = now
            driving, crop = calib_state()
        if not (driving or ALWAYS):
            state.update({'dets': [], 'counts': {}, 'n': 0, 'err': '', 'idle': True,
                          'temp': round(cpu_temp(), 1), 'ts': round(time.time(), 2)})
            atomic_write(os.path.join(OUT_DIR, 'det.json'), json.dumps(state, ensure_ascii=False), binary=False)
            time.sleep(1.0)
            continue
        state['idle'] = False

        if not client.is_connected():
            if not client.connect(False):
                state['err'] = 'camerad 未連線'
                state['dets'] = []
                atomic_write(os.path.join(OUT_DIR, 'det.json'), json.dumps(state, ensure_ascii=False), binary=False)
                time.sleep(1.0)
                continue

        buf = client.recv()
        if buf is None:
            time.sleep(0.02)
            continue

        rgb = nv12_to_rgb_half(buf)
        if rgb.shape[0] != DISP_H or rgb.shape[1] != DISP_W:
            dh, dw = min(DISP_H, rgb.shape[0]), min(DISP_W, rgb.shape[1])
            pad = np.zeros((DISP_H, DISP_W, 3), dtype=np.uint8)
            pad[:dh, :dw] = rgb[:dh, :dw]
            rgb = pad

        img = rgb if crop is None else rgb[:crop]
        lb, scale, px, py = letterbox(img, MW, MH)
        x = lb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0

        t0 = time.perf_counter()
        out = sess.run(None, {inp.name: x})[0]
        infer_ms = (time.perf_counter() - t0) * 1000.0

        if depth_sess is not None and frame_i % DEPTH_EVERY == 0:
            dlb, ds_, dpx, dpy = letterbox(rgb, DW, DH)
            dx = dlb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
            t1 = time.perf_counter()
            dm = np.squeeze(depth_sess.run(None, {dinp.name: dx})[0])
            depth_ms = (time.perf_counter() - t1) * 1000.0
            dgeom = (ds_, dpx, dpy, DH, DW)
        frame_i += 1

        dets = decode(out, names, scale, px, py, DISP_W, img.shape[0])
        if dm is not None and dgeom is not None:
            for d in dets:
                d['dist'] = box_distance(dm, d, *dgeom)

        counts = {}
        for d in dets:
            counts[d['zh']] = counts.get(d['zh'], 0) + 1

        now = time.monotonic()
        dt = now - t_last
        t_last = now
        inst = 1.0 / dt if dt > 0 else 0.0
        fps_ema = inst if fps_ema == 0 else fps_ema * 0.8 + inst * 0.2

        if WRITE_FRAME:
            atomic_write(os.path.join(OUT_DIR, 'frame.rgb'), rgb.tobytes())
        state['crop'] = img.shape[0]
        state.update({'fps': round(fps_ema, 1), 'infer_ms': round(infer_ms, 1),
                      'depth_ms': round(depth_ms, 1),
                      'temp': round(cpu_temp(), 1), 'dets': dets, 'counts': counts,
                      'n': len(dets), 'err': '', 'ts': round(time.time(), 2)})
        atomic_write(os.path.join(OUT_DIR, 'det.json'), json.dumps(state, ensure_ascii=False), binary=False)


if __name__ == '__main__':
    main()
