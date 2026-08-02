# 第4问：速度模式滚球平衡

本目录由当前 `111.3_p` 的视觉、ZDT速度模式、异步采集、终端控制和UDP图传架构重新组织而成。`111.3_p` 没有被修改。

## 目标

- 小车从A点到B点顺时针行驶，总时间不超过8秒。
- 行驶过程中钢球保持在O点附近，最大绝对位置误差不超过1 cm。
- O-5、O、O+5的实测像素坐标分别为 `(176,240)`、`(285,240)`、`(390,240)`。

## 控制结构

```text
旧pre钢球视觉
  -> 三点分段标定得到球位置(cm)
  -> 真实检测帧计算球速度(cm/s)
  -> 第4问PI-D外环（含泄漏积分和抗饱和）
  -> 水管倾角斜率限制
  -> 推拉机构角度/脉冲换算
  -> 电机位置环
  -> 电机速度环
  -> 加速度与jerk限制
  -> ZDT 0xF6速度命令

ZDT反馈：0x36编码器位置 + 0x35实时转速
```

这里不是ZDT位置模式。水管角度只产生电机目标轴位，实际驱动命令始终是 `0xF6` 速度模式。

## 树莓派5编译

```bash
cd 111.4_rewrite
chmod +x build.sh
./build.sh
```

生成：

```bash
./ball2_task4_velocity
```

依赖：

```bash
sudo apt update
sudo apt install -y build-essential pkg-config libopencv-dev
```

## 操作

- 启动前保证水管真实水平，因为当前 `zeroOnStart=true` 会把启动轴位定义为水平零位。
- 把钢球放在O点，绿色识别连续稳定后按 `SPACE` 开始。
- 小车开始从A向B顺时针行驶。
- `F`：到达B点时手动结束考核；程序也会在8秒时自动给出PASS/FAIL。
- `SPACE`：中止本轮并让水管回平。
- `R`：暂停时重置视觉跟踪。
- `Q` 或 `ESC`：安全回零并退出。

GUI、SSH终端按键和E611 UDP视频可同时使用。SSH/X11显示帧率不参与视觉控制；采集线程始终只提供最新帧。

## 现场调参

所有实机参数集中在 `main.cpp` 的 `makeUserConfig()`。

1. `task4Kp`：球偏离O点后的回中力度。
2. `task4Kd`：使用钢球速度提前制动，过小会过冲，过大会放大测速噪声。
3. `task4Ki`：补偿小车持续加速度、安装倾斜和机构静差，只在有限误差范围内积累。
4. `task4FineKp/task4FineKd`：O点附近的小动作参数。
5. `task4MaximumAngleDeg`：第四问最大倾角，目前为0.60度。
6. `task4AngleSlewDegS`：水管目标倾角变化速度，不改变最大幅度。
7. `motorPositionKpRpmPerStep`：电机目标轴位到目标RPM的位置环。
8. `motorVelocityKpPerSecond`：实测RPM到命令RPM的速度环。
9. `motorMaximumAccelerationRpmS` 和 `motorMaximumJerkRpmS3`：软件加速度和jerk限制。

先在原地轻推小车检查纠偏方向：小车加速使球向右偏时，程序应输出正倾角、抬高水管右端，让球向左回O。方向相反时只修改 `motorSign`，不要同时改控制器符号。

## 文件职责

- `main.cpp`：沿用第三问视觉宏和全部现场参数。
- `task4_app.hpp`：第四问流程、8秒考核、终端/UDP和安全退出。
- `task4_balance_control.hpp`：钢球PI-D外环。
- `balance_control.hpp`：像素厘米换算、钢球测速和机构换算。
- `zdt_stepper_uart.hpp`：ZDT协议、电机位置/速度/加速度串级环。
- `steel_ball_vision.hpp`：与当前 `111.3_p` 完全相同的旧pre视觉。
