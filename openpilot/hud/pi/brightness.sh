#!/bin/bash
# 用法: brightness.sh 0.45   (0.10 最暗 ~ 1.00 最亮)
# 走 wlroots gamma 控制（軟體調光）；此面板無 /sys/class/backlight，DDC/CI 也不支援
B=${1:-1.0}
export WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run/user/1000
setsid -f gammastep -m wayland -O 6500 -b "$B" </dev/null >/dev/null 2>&1
sleep 0.6
pkill -x -o gammastep 2>/dev/null   # 殺掉最舊的那個，只留新設定
echo "brightness=$B"
