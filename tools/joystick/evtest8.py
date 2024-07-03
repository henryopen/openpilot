import evdev
import subprocess

def find_usb_device_event(device_name):
    try:
        # 獲取所有輸入裝置
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

        print("Available input devices:")
        for device in devices:
            print(device.path, device.name, device.phys)

        # 尋找匹配的USB裝置
        for device in devices:
            if device_name in device.name:
                return device.path

        return None
    except subprocess.CalledProcessError:
        return None

# 指定要檢查的USB裝置名稱
usb_device_name = 'VKB-Sim (C) Alex Oz 2023  VKBsim Gladiator EVO L'

# 尋找USB裝置對應的事件檔案
event_file = find_usb_device_event(usb_device_name)

if event_file:
    # 創建InputDevice物件
    device = evdev.InputDevice(event_file)
    print(f'已找到USB裝置,事件檔案: {event_file}')

    # 讀取輸入事件
    for event in device.read_loop():
            print(f'type:{event.type} code:{event.code} value:{event.value}')
else:
    print('未找到指定的USB裝置')
