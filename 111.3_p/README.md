# 111.3_pre_github：第3题 ZDT 速度模式串级控制版

本目录面向 H 题第3问：小车静止时，钢球从中心 `O` 到 `O+5 cm`，折返到
`O-5 cm` 并稳定，总时间不超过 `5 s`，两端最大误差绝对值不超过 `1 cm`。

## 钢球识别

当前识别已恢复为 `111.3_pre.zip` 中 2026-08-01 06:41 的原始版本。该版本使用
灰度霍夫圆与暗斑轮廓逐帧跟踪，不包含后来增加的 Lab 颜色异常、背景参考、轴线扫描
后备和运动方向提示。`steel_ball_vision.hpp` 保持与该压缩包逐字节一致。

锁定后暗斑轮廓每帧运行，霍夫圆每8帧复核一次；暗斑失败时同帧调用霍夫备用。
霍夫只精修票数最高的4个候选，相关 `BALL_CFG_*` 参数也已恢复为旧 `pre` 数值。

SSH/X11 窗口只显示异步预览，显示卡顿或刷新率低不会减少识别次数，也不会进入丢球
计数和控制计算。画面左上角的诊断字段含义如下：

```text
H = OpenCV产生的霍夫圆数量
D = 暗斑轮廓候选数量
V = 通过旧版外观与几何评分的候选数量
MISS = 锁定后的连续漏检次数
```

`VISION=BLOB` 表示暗斑轮廓，`VISION=HOUGH` 表示霍夫圆，`VISION=FUSED`
表示两个通道在同一位置相互确认。

若仍显示 `H>0` 但 `V=0`，保留原始相机画面并记录这些数值；不要根据 SSH 窗口
看起来的帧率继续放宽门限，否则容易把管壁阴影或固定螺丝锁成钢球。

## 这次为什么重写电机层

旧版持续发送 ZDT `0xFD` 绝对位置命令。视觉外环每次改变水管角度时，驱动器会
重新规划一段位置运动，软件无法直接利用编码器实时位置和实时转速控制换向过程。

新版本改为 ZDT `0xF6` 速度模式，但没有直接把钢球误差粗暴换算成电机 RPM。
推拉结构需要稳定保持水管角度，单纯速度模式会持续漂移，所以仍保留目标轴位并
在树莓派上闭合电机位置外环：

```text
视觉采集：钢球位置/速度 -> 第3题状态机 -> 目标水管角度
机构模型：目标水管角度 -> 目标电机轴位（脉冲）

50 Hz电机串级环：
0x36高分辨率位置 -> 位置误差 -> 目标RPM（含制动速度上限）
0x35电机实时转速 -> 速度误差 -> 目标加速度
目标加速度 -> 加速度限制 -> 跃度限制 -> 0xF6速度命令
0xF6速度斜率(RPM/s) -> ZDT内部20 kHz速度闭环
```

这样既使用了速度模式，又不会失去目标角度保持能力。电机受负载、间隙或堵转影响
时，编码器反馈会直接改变下一周期的速度和加速度，不再靠软件积分猜测电机位置。

## 推拉机构对应的处理

实物是电机曲柄、长连杆和摆杆组成的曲柄摇杆机构。它有三个明显特性：

1. 电机角度与水管角度不是严格线性关系。
2. 正反换向时存在销轴间隙和结构弹性。
3. 靠近曲柄死点时，传动比和机构冲击会明显变化。

因此 `balance_control.hpp` 仍优先使用 `calibrationPoints` 标定表。未填写标定表时的
曲柄近似公式只适合检查方向，比赛前应使用电子水平仪实测至少 5 个点。

## ZDT 驱动器必须设置

驱动器面板或上位机需要确认：

```text
P_Pul       = PUL_FOC
P_Serial    = UART_FUN
Baud        = 115200
ID_Addr     = 1
Checksum    = 0x6B
Response    = Receive
S_Vel_IS    = Disable
En          = Hold
```

程序启动时读取驱动参数，并根据 `Response` 自动决定是否接收控制命令 ACK；当前实机
为 `Receive`。位置反馈使用 `0x36`，速度反馈使用 `0x35`。`S_Vel_IS` 必须关闭；若
开启，驱动器会把代码发送的 RPM 再缩小 10 倍。

## 当前电机环参数

参数集中在 `main.cpp`：

```cpp
config.motorRpm = 6;                          // 速度模式最大RPM
config.motorSpeedSlopeRpmS = 200;              // 0xF6速度斜率，单位RPM/s
config.motorCommandHz = 50;                   // 读位置、读速度、发速度的频率
config.motorPositionKpRpmPerStep = 0.72;      // 1.8度模式(200 PPR)：位置误差 -> 目标RPM
config.motorVelocityKpPerSecond = 8.0;        // 速度误差 -> 目标加速度
config.motorMaximumAccelerationRpmS = 200.0;  // 软件最大加速度
config.motorMaximumJerkRpmS3 = 300.0;         // 软件最大跃度
config.motorBrakingAccelerationRpmS = 200.0;  // 制动距离使用的等效减速度
config.motorPositionToleranceSteps = 1.0;
config.motorStopSpeedRpm = 1.0;
config.motorSoftLimitSteps = 11;               // 相对水平零位的正负软限位
config.motorReplyTimeoutMs = 15;
config.absoluteEncoderHomeOnStart = true;
config.absoluteEncoderHomeRpm = 6;
config.exitReturnTimeoutMs = 1800;
```

代码把目标速度量化为 Emm V5 `0xF6` 的整数 RPM，并把 `200 RPM/s` 转换为驱动协议的
加速度档位。`motorSpeedSlopeRpmS` 必须不低于软件最大加速度
`motorMaximumAccelerationRpmS`，否则驱动器会额外限速，软件制动模型会失真。

## 第3题钢球 PDI 控制

状态机只负责目标顺序 `O -> O+5 -> O-5` 与完成判定，绝不输出水管角度。每一张真实
检测帧都运行同一个外环：

```text
error = position - target
v2 = (position[n] - position[n-2]) / (time[n] - time[n-2])
pipe_angle = clamp(Kp * error + Kd * v2_filtered + Ki * integral(error), +/- 0.35 deg)
```

`Kp=0.070 deg/cm` 使起点距离 `+5 cm` 时自然得到 `0.35 deg`，保留原来已经有效的
输出量级，但没有任何“去 +5 用固定角度”的分支。`Kd=0.020 deg/(cm/s)`，所以球速
每增加 `5 cm/s`，D项增加 `0.10 deg` 的反向制动。积分在两个目标附近的 `0.50 cm`
范围内、速度不超过 `1 cm/s` 时工作；目标切换时立即清零，最大积分输出为 `0.09 deg`。
大误差且球速低于 `0.35 cm/s` 持续 `120 ms` 时，反馈触发的虚位补偿最多再增加
`0.15 deg`，球开始运动后快速撤销。

`task3PositiveToleranceCm=0.25` 后，状态机在 `+4.75 cm` 以内的连续两帧真实检测后
才切到 `O-5`，不再在 `+3 cm` 提前折返。

## 画面和 CSV 新增信息

预览中的电机行：

```text
M tgt=目标轴位  pos=0x36实时位置  rpm=0x35实测RPM  cmd=命令RPM  acc=命令加速度
```

CSV 同时记录这些量。现场判断顺序：

1. `tgt` 正确但 `pos` 跟不上：调电机速度/加速度环或检查机械阻力。
2. `pos` 能跟上但水管角度不对：重新标定 `calibrationPoints`。
3. 水管角度正确但钢球轨迹不对：先看预览 `P/D/I/v2`。球越快时 `D` 绝对值必须越大；
   再只调 `Kp`、`Kd` 或最终小 `Ki` 中的一个。

## 运行诊断日志

每次启动都会自动创建 `logs/task1_pdi_YYYYMMDD_HHMMSS_mmm.log` 和同名 `.csv`，并
同步打印事件到终端。`.log` 记录串口、细分、ZDT `0x36/0x35/0x3A/0x3B` 状态、按空格
被拦截、ARM、丢球、串口失败和退出；`.csv` 每 `200 ms` 记录钢球位置/速度、是否已
ARM、O 点稳定帧数、目标/实际电机脉冲、实际 RPM、控制 RPM 和实际下发的 F6 RPM，便于
一次运行后直接导入 Excel 或画曲线。

## 推荐调试顺序

1. 取下钢球，确认水管真实水平、曲柄不在死点、连杆没有顶机械限位。
2. 暂时设置 `config.motorEnabled = false`，检查视觉位置、目标方向和脉冲符号。
3. 启用电机后先保持暂停，观察 `tgt=0`、`pos` 接近 0、`rpm` 接近 0。
4. 用小角度目标检查正负方向，实测并填写水管角度标定表。
5. 最后放球运行第3题；一次只修改一个参数并重新编译。

电机环调参优先级：

```text
跟随太慢：先小幅增加 motorPositionKpRpmPerStep
换向太慢：再小幅增加 motorMaximumAccelerationRpmS
换向冲击：降低 motorMaximumJerkRpmS3；不要把 motorSpeedSlopeRpmS 低于软件加速度上限
目标附近来回抖：降低位置环增益，或稍增 motorPositionToleranceSteps
编码器位置越界：立即停机，重新确认水平零位、motorSign和软限位
```

## 编译和运行

在树莓派 5 的 Ubuntu 终端执行：

```bash
sudo apt update
sudo apt install -y build-essential pkg-config libopencv-dev \
  gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav
cd ~/111.3_p
bash build.sh
./ball2_task3_velocity
```

目录名以你实际复制到树莓派上的名字为准。`build.sh` 会先在树莓派本机编译并运行
`motor_velocity_control_test`，测试通过后才生成正式程序。

正式代码使用 Linux `termios/read/write` 访问 `/dev/ttyAMA0`。Windows 只用于离线
语法检查，比赛程序是在树莓派 5 Ubuntu 上原生编译、原生运行。

首次运行前确认串口权限：

```bash
ls -l /dev/ttyAMA0
sudo usermod -aG dialout "$USER"
```

加入 `dialout` 后需要注销并重新登录。不要长期使用 `sudo` 运行视觉程序。

快捷键：

```text
SPACE   开始或中止第3题
R       暂停时清除视觉旧轨迹，重新在O点锁球
Q/ESC   用速度串级环回水平零位，再发送立即停止并退出
```

## E611低延迟图传

程序默认把带钢球识别、任务阶段和电机状态的画面编码为
`640x480 20 FPS`灰度`H.264/MPEG-TS`，通过UDP发送到场外电脑：

```text
目标IP：192.168.137.1
目标端口：5600/UDP
码率：1500 kbit/s
```

摄像头采集和识别仍保持120 FPS。图传使用独立单帧队列，编码或E611链路
来不及时只丢弃旧图传帧，不会阻塞视觉识别或电机控制。

树莓派与场外电脑的有线网卡分别配置为同一网段，例如：

```text
树莓派eth0：192.168.137.2/24
场外电脑：  192.168.137.1/24
```

Windows VLC选择“媒体 -> 打开网络串流”，输入：

```text
udp://@:5600
```

当前配置为`config.gui = false`，不创建X11窗口。通过普通SSH登录树莓派后直接
运行程序，保持该终端为前台，即可使用`SPACE/R/Q`单键；不需要`ssh -X/-Y`，
也不需要MobaXterm X Server。正式比赛应把相同启动事件接到车载实体按钮。

打开VLC的“查看 -> 高级控制”后，可用红色录像按钮完整记录每次测试并回放。
启动程序时若出现`cannot open GStreamer UDP video pipeline`，先检查：

```bash
gst-inspect-1.0 x264enc
gst-inspect-1.0 mpegtsmux
opencv_version --verbose | grep -i gstreamer
```

`main.cpp`中的`videoStreamHost`、端口、帧率和码率可按现场网络修改。
不使用图传时设置`config.videoStreamEnabled = false`。

## 安全检查

当前配置启用真实电机并在每次启动时回到持久绝对编码器原点：

```cpp
config.motorEnabled = true;
config.absoluteEncoderHomeOnStart = true;
config.absoluteEncoderHomeRpm = 6;
config.zeroOnStart = false;
config.serialPort = "/dev/ttyAMA0";
```

第一次使用时，将水管放在真实水平位置并执行：

```bash
./motor_cli origin-set
```

该命令用 `0x93` 把当前单圈绝对编码器角度保存为永久零点，并保存 `Nearest / 6 RPM`
回零参数。以后程序启动先用 `0x9A` 自动回到该原点，确认完成后才把高分辨率 `0x36`
运行坐标清零并进入 `PAUSED`。回零失败、参数不一致或超时都会拒绝启动。GPIO14/TXD
物理 8 脚接 ZDT RX、GPIO15/RXD 物理 10 脚接 ZDT TX，并共地。

任意编码器查询、速度查询或控制应答超时都会结束主循环。退出时程序会在
`exitReturnTimeoutMs` 内闭环回零，随后无论是否到位都发送 `0xFE` 立即停止。

## 本地验证

`motor_velocity_control_test.cpp` 覆盖：

```text
ZDT 0xF6帧格式
最大RPM限制
位置 -> 速度 -> 加速度 -> 跃度串级控制
+97脉冲到位、反向到-60脉冲
±130脉冲软限位
```

`task3_sequence_test.cpp` 另外验证漏检会清零折返确认计数，并且只有连续 3 个
真实到达帧才能从 `GO O+5` 切换到 `GO O-5`。
