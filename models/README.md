# 紅綠燈辨識模型

> 這個資料夾裡的模型以 **AGPL-3.0** 授權（全文見 `LICENSE-AGPL-3.0.txt`），跟本 repo 程式碼的 MIT 授權不同。
>
> 模型是用 [Ultralytics](https://github.com/ultralytics/ultralytics) YOLO11 在作者車輛錄的影像上訓練出來的；Ultralytics 以 AGPL-3.0 授權，以它訓練出的模型同樣適用 AGPL-3.0。

只用作者一台車在台灣錄的影像訓練，其他地區、其他鏡頭位置或安裝角度可能判錯。

## comma four（`c4/`）

放到 comma four 的 `/data/yolo/`，由 `openpilot/yolo/tlwatchd.py` 使用。有 comma 3 時只在 comma 3 沒有資料時當備援。

| 檔案 | 用途 | MD5 |
|---|---|---|
| `c4band1_1344x352.int8v2.onnx` | 找燈（畫面上方橫帶，1344×352） | `af3e07bb219f1460494a942a3939982b` |
| `tlcls_v2_1344x352.onnx` | 判燈色 | `823347133e8e20c5a756caf5a5a87934` |

## comma 3（`c3/`，只有 `full`、`no-led` 分支）

放到 comma 3 的 `c3link/models/`，由 `openpilot/c3link/c3lights.py` 使用。

| 檔案 | 用途 | MD5 |
|---|---|---|
| `c3light_ft1n_704x1248_raw.onnx` | 找燈 | `569ac1ec23f0af65770926de766995d9` |
| `lightcls_64.onnx` | 判燈色（green／green_arrow／off／pedestrian／red／yellow） | `70534849dc86c9085d8d851d8e3c143a` |
| `lampcls_96.onnx` | 判箭頭方向 | `412a57f0e87e3e3a2750a64b55eb1f29` |

兩台都需要另外安裝 onnxruntime。
