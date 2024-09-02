#!/usr/bin/env python
import evdev
import math
from openpilot.common.params import Params
from openpilot.common.numpy_fast import interp
from select import select
import serial
import os

mem_params = Params("/dev/shm/params")
params = Params()
mem_params.put_bool("KeyResume", False)
mem_params.put_bool("KeyCancel", False)
params.put_bool("SpeedLimitController",True)
mem_params.put_bool("SLC",True)
mem_params.put_bool("IsLockOn", True)
mem_params.put_bool("AutoTurn", False)

event_file = None

def find_usb_device_event(device_name):
        # 獲取所有輸入裝置
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if device_name in device.name:
                return device.path
        return None

def main():
    # 指定要檢查的USB裝置名稱
    usb_device_name = 'Thrustmaster TWCS Throttle'
    # 尋找USB裝置對應的事件檔案
    event_file = find_usb_device_event(usb_device_name)
    if event_file is not None:
        # 創建InputDevice物件
        device = evdev.InputDevice(event_file)
    turnon = False
    for event in device.read_loop():
            set_speed = mem_params.get_int("KeySetSpeed")
            key_setspeed = set_speed
            #data = evdev.categorize(event)
            if event.type == 1 and event.value == 1:
                # if event.code == 288 :
                #     if mem_params.get_bool("IsLockOn"):
                #         mem_params.put_bool("IsLockOn", False)
                #     else:
                #         mem_params.put_bool("IsLockOn", True)
                #         key_setspeed = mem_params.get_int("KeyThSpeed")
                if event.code == 288 :
                    if mem_params.get_bool("AutoAcce"):
                        mem_params.put_bool("AutoAcce", False)
                        mem_params.put_int("KeyAcce", 0)
                    else:
                        mem_params.put_bool("AutoAcce", True)
                elif event.code == 289:
                     if turnon:
                        mem_params.put_int("KeyTurnLight", 0)
                        # mem_params.put_bool("AutoTurn",True)
                        turnon = False
                elif event.code == 290:
                    if params.get_bool("IsEngaged"):
                        mem_params.put_bool("KeyResume", False)
                        mem_params.put_bool("KeyCancel", True)
                    else:
                        mem_params.put_bool("KeyResume", True)
                        mem_params.put_bool("KeyCancel", False)
                        if mem_params.get_bool("IsLockOn"):
                            key_setspeed = mem_params.get_int("KeyThSpeed")
                elif event.code == 291:
                    if mem_params.get_int("KeyTurnLight") == 1 and turnon:
                        mem_params.put_int("KeyTurnLight", 0)
                        # mem_params.put_bool("AutoTurn",True)
                        turnon = False
                    else:
                        mem_params.put_int("KeyTurnLight", 1)
                        mem_params.put_bool("AutoTurn",False)
                        turnon = True
                elif event.code == 292:
                    if mem_params.get_int("KeyTurnLight") == 2 and turnon:
                        mem_params.put_int("KeyTurnLight", 0)
                        # mem_params.put_bool("AutoTurn",True)
                        turnon = False
                    else:
                        mem_params.put_int("KeyTurnLight", 2)
                        mem_params.put_bool("AutoTurn",False)
                        turnon = True
                elif event.code == 297 :
                    if mem_params.get_bool("AutoTurn"):
                        mem_params.put_bool("AutoTurn",False)
                        mem_params.put_int("KeyTurnLight", 0)
                    else:
                        mem_params.put_bool("AutoTurn",True)
                        mem_params.put_bool("AutoChange",False)
                elif event.code == 294 :
                        params.put("NavDestination", "{\"latitude\": %f, \"longitude\": %f, \"place_name\": \"%s\"}" % (24.9489724, 121.348714, "\u5c16\u5c71\u8def101\u5df77\u5f04"))
                elif event.code == 295 :
                    params.put("NavDestination", "{\"latitude\": %f, \"longitude\": %f, \"place_name\": \"%s\"}" % (25.038796, 121.563405, "\u561f\u561f\u623f\u5e9c\u524d\u5ee3\u5834\u5730\u4e0b\u505c\u8eca\u5834"))
                elif event.code == 296 :
                    params.remove("NavDestination")
                    mem_params.put_bool("AutoTurn",False)
                    mem_params.put_int("KeyTurnLight", 0)
                elif event.code == 298 :
                    slcenabled = mem_params.get_bool("SLC")
                    mem_params.put_bool("SLC", not slcenabled)
                elif event.code == 299 :
                    conditional_status = mem_params.get_int("CEStatus")
                    override_value = 0 if conditional_status in {1, 2, 3, 4, 5, 6} else 1 if conditional_status >= 7 else 2
                    mem_params.put_int("CEStatus", override_value)
                elif event.code == 300 :
                    alongenabled = mem_params.get_bool("AutoLong")
                    mem_params.put_bool("AutoLong", not alongenabled)
                elif event.code == 301 :
                    long_per = int(params.get("LongitudinalPersonality"))
                    long_per = (long_per + 1) % 3
                    params.put("LongitudinalPersonality", str(long_per))
                    mem_params.put_bool("AutoLong",False)
            elif event.type == 3:
                if event.code == 2:
                    range_index = (65535-event.value) // 5042
                    mapped_value = range_index * 10
                    if mapped_value < 0:
                        mapped_value = 0
                    mem_params.put_int("KeyThSpeed", mapped_value)
                    if mem_params.get_bool("IsLockOn"):
                        key_setspeed = mapped_value
                    if mapped_value == 0:
                        params.put_bool("ExperimentalMode", False)
                elif event.code == 1:
                    mem_params.put_bool("AutoAcce", False)
                    if event.value < 512:
                        out_abs_b = interp(event.value, [0,511], [-1., 0.])
                    elif event.value > 512:
                        out_abs_b = interp(event.value, [513,1023], [0., 1.])
                    else:
                        out_abs_b = 0
                    mem_params.put_int("KeyAcce", out_abs_b*100)
                elif event.code == 5:
                    if event.value < 448:
                        out_abs_c = interp(event.value, [0,447], [1., 0.])
                    elif event.value > 576:
                        out_abs_c = interp(event.value, [577,1023], [0., -1.])
                    else:
                        out_abs_c = 0
                    mem_params.put_int("KeyTurn", out_abs_c*100)
            new_setspeed = key_setspeed
            if new_setspeed != set_speed:
                mem_params.put_int("KeySetSpeed", new_setspeed)
                mem_params.put_bool("KeyChanged", True)
                mem_params.put_bool("SpeedLimitChanged", False)
if __name__ == "__main__":
    main()
