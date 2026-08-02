#!/usr/bin/env bash

# 任意命令失败、使用未定义变量或管道中间失败时立即退出，
# 避免编译失败后误运行旧的可执行文件。
set -euo pipefail

# 本脚本在树莓派5 Ubuntu上原生运行，不用于Windows交叉编译。
if [[ "$(uname -s)" != "Linux" ]]; then
    echo "Error: build.sh must run on Raspberry Pi/Linux." >&2
    exit 1
fi

for command in g++ pkg-config; do
    if ! command -v "$command" >/dev/null 2>&1; then
        echo "Error: missing $command" >&2
        echo "Install with: sudo apt update && sudo apt install -y build-essential pkg-config libopencv-dev" >&2
        exit 1
    fi
done

if ! pkg-config --exists opencv4; then
    echo "Error: OpenCV 4 development files were not found." >&2
    echo "Install with: sudo apt update && sudo apt install -y libopencv-dev" >&2
    exit 1
fi

architecture="$(uname -m)"
compiler_version="$(g++ -dumpfullversion -dumpversion)"
echo "Building on Linux ${architecture} with g++ ${compiler_version}"

# 当前文件夹是第3题速度模式串级控制版：钢球PDI外环输出目标水管角度，
# 电机层读取ZDT编码器位置和速度，再通过位置/速度/加速度环发送0xF6。
# -O3用于树莓派端视觉和控制计算；pkg-config自动加入OpenCV 4头文件和库；
# pthread用于OpenCV和程序中的标准线程/等待功能。
opencv_flags=( $(pkg-config --cflags --libs opencv4) )
common_flags=(-std=c++17 -Wall -Wextra -Wpedantic -pthread)

# 先在树莓派本机验证0xF6报文、软限位和位置/速度/加速度串级环。
g++ -O2 "${common_flags[@]}" motor_velocity_control_test.cpp \
    -o motor_velocity_control_test "${opencv_flags[@]}"
./motor_velocity_control_test

g++ -O2 "${common_flags[@]}" task3_sequence_test.cpp \
    -o task3_sequence_test "${opencv_flags[@]}"
./task3_sequence_test

# 测试通过后才生成比赛正式程序，避免误运行旧二进制文件。
g++ -O3 -DNDEBUG "${common_flags[@]}" main.cpp \
    -o ball2_task3_velocity "${opencv_flags[@]}"

echo "Build complete: ./ball2_task3_velocity"
echo "  ZDT: 0xF6 velocity mode + 0x35 speed + 0x36 encoder position"
