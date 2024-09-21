import serial
#import random
import time
import json
import os
import math
from openpilot.common.params import Params

pa = Params()
mp = Params("/dev/shm/params")
mp.put("ANav", "")

def main():
    ser = None
    turnlight = False
    keytl_prev = 0
    a10 = 0
    ma21 = pa.get_int("ALTvol")
    while ser is None:
        ser = serial.Serial("/dev/ttyACM0", 115200)
        time.sleep(2)
    while True:
        lpvalue = int(pa.get("LongitudinalPersonality"))
        keyturnlight = mp.get_int("KeyTurnLight")
        if keyturnlight == 1 and not turnlight:
            a10 = 1
            keytl_prev = keyturnlight
            turnlight = True
        elif keyturnlight == 2 and not turnlight:
            a10 = 2
            keytl_prev = keyturnlight
            turnlight = True
        elif (keyturnlight == 0 or keyturnlight != keytl_prev) and turnlight:
            a10 = 0
            turnlight = False
        # 要發送的 17 個數據
        a00, a01, a02 = mp.get_int("DisMax"), math.ceil(mp.get_int('DetectSpeedLimit')/10)*10, mp.get_int("KeyThSpeed")
        a03, a04, a05 = mp.get_int("ASpeed"), mp.get_int("AdvRatio"), mp.get_int("ADF")
        a06, a07 = mp.get_int("ADrel"), mp.get_int("ASO")
        x08, a09 = mp.get_int("ALight"), mp.get_int("AAccel")
        #if mp.get_bool("AutoLong"):
            #a18 = 0
        lvalue = 2 if x08 < 40 else 1 if x08 < 80 else 0
        if not lvalue == lpvalue:
            pa.put("LongitudinalPersonality", str(lvalue))
        if mp.get_bool("AutoAcce"):
            a18 = 0
        else:
            a18 = 1
        a08 = mp.get_int("TrafficState")
        if mp.get_bool("IsLockOn"):
            a11 = 1
        else:
            a11 = 0
        if mp.get_bool("SLC"):
            a12 = 0
        else:
            a12 = 1
        if mp.get_int("CEStatus") == 13 or mp.get_int("CEStatus") == 14:
            a13 = 0
        elif mp.get_int("CEStatus") == 15:
            a13 = 1
        elif mp.get_int("CEStatus") == 10 or mp.get_int("CEStatus") == 11:
            a13 = 3
        else:
            a13 = 2
        if mp.get_bool("AutoTurn"):
            a14 = 1
        else:
            a14 = 0
        if mp.get_bool("RightBlind"):
            a15 = 0
        else:
            a15 = 1
        if mp.get_bool("LeftBlind"):
            a16 = 1
        else:
            a16 = 0
        if pa.get_bool("IsEngaged"):
            a17 = 1
        else:
            a17 = 0
        a19 = mp.get_int("TrafficState1")
        a20 = mp.get_int("KeyTurn")
        xa21 = mp.get_int("ATvol")
        ma22 = mp.get_int("AKvs")
        if xa21 / 10 > 65.8:
            if ma21 == 0:
                pa.put_int("ALTvol", 800)
                ma21 = 800
            else:
                pa.put_int("ALTvol", (ma21/10 - ma22 / 100) * 10)
            a21 = (ma21/10 - ma22 / 100)
        else:
            pa.put_int("ALTvol", 0)
            a21 = xa21 / 10
        a22, a23 = ma22 / 100, mp.get_int("AKML") / 100
        a24o = mp.get("ANav", encoding="utf8")
        if a24o is not None:
            a24 = a24o.replace("\n", "").replace(" ", "")
        else:
            a24 = "HFOP External Display by Henry Chen"
        a25 = mp.get("RoadName", encoding="utf8")
        # 將數據轉換為字符串形式，並以逗號分隔
        data_str = f"{a00},{a01},{a02},{a03},{a04},{a05},{a06},{a07},{a08},{a09},{a10},{a11},{a12},{a13},{a14},{a15},{a16},{a17},{a18},{a19},{a20},{a21},{a22},{a23},{a24},{a25}\n"
        # 發送數據到 Arduino
        ser.write(data_str.encode())
        time.sleep(0.05)


if __name__ == "__main__":
    main()