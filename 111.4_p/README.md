# 第4问：运行中保持O点

目标：小车从A到B运行约8秒，钢球全过程保持在O点，最大位置误差不超过
`±1 cm`。

## 当前控制链

```text
120 FPS视觉
  -> 三点标定位置
  -> 两帧位置差速度 + 轻低通
  -> 对称PDI外环
  -> 水管角度变化率限制
  -> 电机位置/速度/加速度串级环
  -> Emm V5 0xF6速度命令
```

- 标定点：`O-5=(167,247)`、`O=(275,247)`、`O+5=(398,247)`。
- 曝光：手动固定 `60`，MJPG、`640x480@120 FPS`。
- P：左右统一 `0.110 deg/cm`。
- D：两帧速度反馈，`0.080 deg/(cm/s)`，极性用于阻挡当前速度。
- I：只在中心 `±1 cm` 且速度低于 `1 cm/s` 时工作，最大 `±0.12°`。
- P+I推进限幅：`±0.55°`；加入D后的总制动限幅：`±0.65°`。
- 电机：1.8°整步量纲 `200 PPR`，软件与0xF6加速度均为 `200 RPM/s`。
- 启动自动回到已保存的绝对编码器LEVEL零点，不把任意上电位置当零点。

## 编译

```bash
cd /home/boweny/111.4_p
./build.sh
```

生成 `./ball2_task4_velocity`。构建会先运行串口协议、电机串级环和第四问
控制器测试。

## 操作

```bash
cd /home/boweny/111.4_p
./ball2_task4_velocity
```

程序自动归位后处于 `PAUSED`。等待阶段始终发送图传，并显示未开始的原因。
球进入O点 `±1 cm` 且基本静止后，画面显示 `READY - PRESS SPACE`，按空格开始：

- `SPACE`：开始或中止本轮。
- `F`：到达B点时手动结束；程序也会在8秒自动判定。
- `R`：暂停时重置视觉跟踪。
- `Q` / `ESC`：回LEVEL并退出。

Windows图传：

```powershell
ffplay -fflags nobuffer -flags low_delay -framedrop "udp://@:5600"
```

每次启动会在 `/home/boweny/111.4_p/logs/` 生成一份 `.log` 和 `.csv`，
记录球位置、两帧速度、P/D/I、目标角度、电机位置与转速、最大误差和丢球时间。

## 现场调参

参数都在 `main.cpp`：

1. `task4Kp`：回中力度；超调大先小幅降低。
2. `task4Kd`：速度阻尼；提前减速不足就增加，抖动明显就降低。
3. `task4Ki`、`task4IntegralLimitDeg`：持续偏向一边时再增加。
4. `task4DriveAngleLimitDeg`：P+I推进上限。
5. `task4BrakeAngleLimitDeg`：P+I+D总制动上限。
6. `task4AngleSlewDegS`：角度响应速度。

先看日志区分问题：位置偏差增长但D太小是阻尼不足；D频繁顶限并换向是D过大或
视觉速度噪声；速度已接近零但长期偏一侧才应该增加I。
