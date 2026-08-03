#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -s)" != "Linux" ]]; then
    echo "Error: build.sh must run on Raspberry Pi/Linux." >&2
    exit 1
fi

for command in g++ pkg-config; do
    if ! command -v "$command" >/dev/null 2>&1; then
        echo "Error: missing $command" >&2
        exit 1
    fi
done

if ! pkg-config --exists opencv4; then
    echo "Error: OpenCV 4 development files were not found." >&2
    exit 1
fi

architecture="$(uname -m)"
compiler_version="$(g++ -dumpfullversion -dumpversion)"
echo "Building TASK5 on Linux ${architecture} with g++ ${compiler_version}"

opencv_flags=( $(pkg-config --cflags --libs opencv4) )
common_flags=(-std=c++17 -Wall -Wextra -Wpedantic -pthread)
shared_dir="../111.4_p"

g++ -O2 "${common_flags[@]}" "${shared_dir}/contest_control_uart_test.cpp" \
    -o contest_control_uart_test
./contest_control_uart_test

g++ -O2 "${common_flags[@]}" "${shared_dir}/contest_control_ipc_test.cpp" \
    -o contest_control_ipc_test
./contest_control_ipc_test

g++ -O2 "${common_flags[@]}" "${shared_dir}/motor_velocity_control_test.cpp" \
    -o motor_velocity_control_test "${opencv_flags[@]}"
./motor_velocity_control_test

g++ -O2 "${common_flags[@]}" "${shared_dir}/task4_balance_control_test.cpp" \
    -o task5_balance_control_test "${opencv_flags[@]}"
./task5_balance_control_test

g++ -O3 -DNDEBUG "${common_flags[@]}" main.cpp \
    -o ball2_task5_velocity "${opencv_flags[@]}"

echo "Build complete: ./ball2_task5_velocity"
echo "  Shared control: ../111.4_p"
echo "  Evaluation time: 20.0 seconds"
