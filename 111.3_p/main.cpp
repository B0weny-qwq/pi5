// main.cpp
// ============================================================================
// 第3题位置PDI版：状态机只切换O+5/O-5目标，角度每帧由位置误差、两帧
// 速度和终点小积分共同计算。视觉每帧同时使用反光暗斑和完整霍夫圆，
// SSH/X11预览速度不参与识别或控制判断。
// 本文件也是唯一需要经常改参数的文件。
//
// 文件顶部是反光钢球识别参数；makeUserConfig()中是ROI、水管轴线、PD、
// 曲柄连杆、实测标定表、ZDT驱动器和安全时序参数；最下面是main()启动函数。
// 其他头文件属于算法实现层，正常调试时不要进去改数值。
// ============================================================================

// ======================== A. 视觉参数修改区 ========================

// 【通常不改】你的摄像头已实测640x480、MJPG、约120 FPS。
#define BALL_CFG_CAMERA_WIDTH 640
#define BALL_CFG_CAMERA_HEIGHT 480
#define BALL_CFG_CAMERA_FPS 120

// Restore the visual parameters from the archived 111.3_pre snapshot.
#define BALL_CFG_RADIUS_MIN 7.5f
#define BALL_CFG_RADIUS_MAX 16.0f
#define BALL_CFG_RADIUS_EXPECTED 11.0f

#define BALL_CFG_MIN_DETECTION_SCORE 0.34f

// 【识别调参】首次锁定比持续跟踪严格，防止无球时锁到螺丝或阴影。
#define BALL_CFG_MIN_ACQUIRE_SCORE 0.44f

// 静止跟踪时第一名和第二名过于接近会拒绝跳目标；行进时不使用该拒绝，
// 否则真球运动后大量霍夫候选会造成连续BALL LOST。
#define BALL_CFG_AMBIGUITY_GAP 0.050f

#define BALL_CFG_ACQUIRE_FRAMES 3

#define BALL_CFG_MAX_MISSES 18

// 首次连续确认和锁定后的预测搜索门限，单位像素。
// 当前钢球每帧正常位移远小于原来的20～58 px；门限过大会允许绿圈跳到管壁假圆。
#define BALL_CFG_ACQUIRE_GATE_PX 16.0f
#define BALL_CFG_TRACK_GATE_MIN_PX 14.0f
#define BALL_CFG_TRACK_GATE_MAX_PX 90.0f

// 首次锁定必须满足的金属钢球外观条件；一次只改一个参数。
#define BALL_CFG_ACQUIRE_CONTRAST_MIN 8.0f
#define BALL_CFG_ACQUIRE_EDGE_SUPPORT_MIN 0.28f
#define BALL_CFG_ACQUIRE_RING_MEAN_MIN 40.0f
#define BALL_CFG_ACQUIRE_INNER_STD_MIN 2.5f

// ---------------- 灰度霍夫圆专用参数 ----------------

// 轴线已按最新画面的y=240标定，正式识别启用轴线过滤，排除管外螺丝和圆孔。
#define BALL_CFG_USE_AXIS_GATE 1
#define BALL_CFG_AXIS_GATE_PX 14.0f

// 局部灰度增强、霍夫圆和圆心二次拟合参数。一般保持以下实测初值。
// 由+16降到+10，让白色水管少一点过曝；这里只改变算法输入灰度。
#define BALL_CFG_GRAY_BRIGHTNESS 10
#define BALL_CFG_CLAHE_CLIP_LIMIT 2.2
#define BALL_CFG_HOUGH_DP 1.0
#define BALL_CFG_HOUGH_CANNY_HIGH 74.0
#define BALL_CFG_HOUGH_ACCUM_ACQUIRE 3.0
#define BALL_CFG_HOUGH_ACCUM_TRACK 3.0
#define BALL_CFG_REFINE_GRADIENT_MIN 16.0f
#define BALL_CFG_REFINE_RESIDUAL_MAX 2.0f

// 不再为SSH预览帧率削减识别。每帧运行霍夫并精修全部合理候选；截图中已有
// H=38但旧代码只检查前4个，这正是钢球存在却V=0的主要原因之一。
#define BALL_CFG_HOUGH_INTERVAL 8
#define BALL_CFG_HOUGH_MAX_CANDIDATES 4

// 第3题开始前球必须在O点。首次捕获只接受O点附近75像素内的候选，
// 这样无球或刚启动时不会锁到水管右端固定螺丝。
#define BALL_CFG_INITIAL_ACQUIRE_GATE_PX 55.0f

// ====================== A. 视觉参数修改区结束 ======================

// include会把视觉、控制、ZDT串口和完整主循环一起编译进最终可执行文件。
#include "stepper_app.hpp"

#include <cmath>
#include <csignal>

ball_stepper::AppConfig makeUserConfig()
{
    ball_stepper::AppConfig config;

    // ========================================================================
    //                       B. 系统参数修改区开始
    // ========================================================================

    // ---------------- 1. 摄像头 ----------------
    config.cameraIndex = 0;   // /dev/video0通常为0；第二个摄像头通常为1。
    config.cameraWidth = BALL_CFG_CAMERA_WIDTH;
    config.cameraHeight = BALL_CFG_CAMERA_HEIGHT;
    config.cameraFps = BALL_CFG_CAMERA_FPS;
    config.cameraFourcc = "MJPG";

    // 固定安装后关闭自动对焦，避免识别过程中镜头反复改变清晰度和钢球外观。
    config.disableAutofocus = true;

    // 【当前保持false】完全不让OpenCV改曝光。你已经验证这只摄像头原设置能
    // 640x480 MJPG 120 FPS；程序强制切自动/手动曝光反而可能掉帧并增加拖影。
    // 如果必须手动曝光，先用v4l2-ctl确认单位，保证曝光时间小于8.3 ms。
    config.configureExposure = false;
    config.useManualExposure = false;
    config.exposureAbsolute = 45.0;  // 如后续重开手动曝光，先从4.5 ms试起。

    // ---------------- 2. ROI和水管轴线 ----------------
// 黄框和实际识别ROI都覆盖整段可见水管；最新三个位置为x=167、275、398。
    // 两者使用同一个矩形，避免钢球明明还在黄色框内却已经离开算法搜索范围。
    config.pipeDisplayArea = cv::Rect(10, 210, 535, 65);
    config.roi = config.pipeDisplayArea;
    config.drawPipeDetectionArea = true;

    // 摄像头目前看不到整根水管，因此先用第3题-5 cm和+5 cm两个实测点
    // 定义运动轴线方向；实际位置换算由下面的三点分段标定完成。
    config.axisLeft = cv::Point2f(167.0f, 247.0f);
    config.axisRight = cv::Point2f(398.0f, 247.0f);
    config.axisConfigured = true;

    // 【必须实测】上面两个图像点之间对应的真实有效滚球长度，单位厘米。
    config.pipeLengthCm = 10.0;

    // 【第3题核心标定】本次图传实测：O-5/O/O+5约为167/275/398。
    // 左侧5 cm对应108像素，右侧5 cm对应123像素，分段换算会分别使用。
    // 三个点当前都使用实测水管轴线y=247。
    config.useThreePointPositionCalibration = true;
    config.minus5CalibrationPoint = cv::Point2f(167.0f, 247.0f);
    config.centerCalibrationPoint = cv::Point2f(275.0f, 247.0f);
    config.plus5CalibrationPoint = cv::Point2f(398.0f, 247.0f);

    // 兼容基础控制模块的初始目标，比赛运行时实际目标由题目管理器自动给出。
    config.targetCm = config.pipeLengthCm * 0.5;

    // ---------------- 3. 第3题自动流程 ----------------
    // 第3题规定的两个目标相对中心O分别为+5 cm和-5 cm，一般不要改。
    config.task3OffsetCm = 5.0;
    // 实机最终仍停在x=194右侧，因此内部控制终点向左补偿增加到0.40 cm；
    // x=194仍是真实-5标定点，不修改视觉坐标含义。
    config.task3NegativeTargetBiasCm = 0.0;

    // +5 is a real passing point, not an early braking trigger.  The state
    // machine changes target only after two real detections within 0.25 cm.
    config.task3PositiveToleranceCm = 0.25;
    config.task3ArrivalConfirmFrames = 2;

    // O-5 completion only controls the status display.  The same PDI loop
    // continues holding the target after completion.
    config.task3FinalToleranceCm = 0.20;
    config.task3FinalRightToleranceCm = 0.02;
    config.task3FinalSpeedCmS = 0.6;
    config.task3FinalStableMs = 250;
    config.task3TimeLimitMs = 5000;

    // ---------------- 4. 钢球 PDI 外环 ----------------
    // 直管左右机械条件接近，两侧使用完全相同的位置P增益。
    // 5 cm误差对应0.30 deg；接近目标后按误差线性减小。
    config.task3MoveRightPositionKpDegPerCm = 0.050;
    config.task3MoveLeftPositionKpDegPerCm = 0.050;

    // D仍使用x[n]与x[n-2]的速度。日志显示0.015在高速过靶时制动力不足；
    // P保持柔和的0.060，D恢复0.030只增强速度阻尼，不增加静止起步推力。
    config.task3VelocityKdDegPerCmS = 0.060;

    // I只在最后2.5 cm且球速较低时介入。最大I输出为0.250*0.80=0.20 deg，
    // 足够消除直管静差，同时比旧版0.40 deg柔和，目标反向时仍会清零。
    config.task3IntegralKiDegPerCmSecond = 0.250;
    config.task3IntegralZoneCm = 2.50;
    config.task3IntegralSpeedLimitCmS = 1.0;
    config.task3IntegralLimitCmSeconds = 0.80;

    // Breakaway overlaps the final I window, avoiding the old 0.5-1.0 cm gap
    // where neither compensation was active. It decays as soon as motion is
    // measured, so this is still feedback driven rather than a minimum angle.
    config.task3BreakawayErrorCm = 0.35;
    config.task3BreakawaySpeedCmS = 0.35;
    config.task3BreakawayDelaySeconds = 0.08;
    config.task3BreakawayRampDegPerSecond = 0.40;
    config.task3MoveRightBreakawayMaximumAngleDeg = 0.12;
    config.task3MoveLeftBreakawayMaximumAngleDeg = 0.12;

    // 直管双向基础输出和脱困输出完全对称：正常最多+/-0.60 deg，
    // 反馈确认卡滞后最多再叠加0.12 deg，但仍受总机械角度限幅保护。
    config.task3MoveRightOutputAngleLimitDeg = 0.80;
    config.task3MoveLeftOutputAngleLimitDeg = 0.80;

    // v[n] = (x[n] - x[n-2]) / (t[n] - t[n-2]); low-pass only removes vision
    // jitter after that two-frame difference.
    config.speedDifferenceFrames = 2;
    config.speedFilterSeconds = 0.020;

    config.maximumPipeAngleDeg = 1.00;
    config.angleSlewDegS = 5.0;

    // ---------------- 5. 曲柄连杆尺寸 ----------------
    // 【必须实测】电机输出轴中心到实际使用曲柄孔中心的距离，不是圆盘直径。
    config.crankRadiusMm = 15.0;

    // 【必须实测】水管固定铰链中心到连杆与水管连接点中心的距离。
    config.actuatorDistanceMm = 250.0;

    // Must match the ZDT driver: 1.8-degree full-step mode, 200 PPR.
    config.pulsesPerRevolution = 200;

    // +1表示正脉冲应使axisRight端升高；实际相反时只改成-1。
    config.motorSign = +1;

    // ---------------- 6. 水管角度到电机脉冲实测标定表 ----------------
    // 【最终实机强烈建议填写】没有标定表时使用理想曲柄公式，只适合检查思路。
    // 标定方法：
    // 1. 不放钢球，让水管水平并清零，记录{0.0, 0}。
    // 2. 分别给出若干正负目标轴位脉冲，用电子水平仪测水管真实角度。
    // 3. 按角度严格从小到大填写至少5个点。
    //
    // 下面是格式示例，不是你的机构实测值。启用时删除外层注释并替换全部数字。
    config.calibrationPoints = {
        // {-1.92, -300},
        // {-1.27, -200},
        // {-0.64, -100},
        // { 0.00,    0},
        // { 0.66,  100},
        // { 1.31,  200},
        // { 1.96,  300},
    };

    // ---------------- 7. 树莓派GPIO14/15与ZDT串口 ----------------
    // 【第一次保持false】false只显示角度和脉冲，不打开串口、不驱动电机。
    // 轴线、方向、脉冲量和机械限位全部确认后再改为true。
#if defined(BALL_E611_VIDEO_ONLY)
    config.motorEnabled = false;
#else
    config.motorEnabled = true;
#endif

    // 树莓派5：GPIO14/TXD物理8脚接ZDT RX，GPIO15/RXD物理10脚接ZDT TX，GND共地。
    // 当前这台Ubuntu系统已确认GPIO14/15对应/dev/ttyAMA0，且没有/dev/serial0别名。
    config.serialPort = "/dev/ttyAMA0";
    config.serialBaud = 115200;   // 必须与ZDT驱动器设置一致，格式8N1。

    // ZDT通信地址。单电机通常为1，但必须以驱动器面板/上位机实际设置为准。
    config.motorAddress = 1;

    // 速度模式的最大转速。0.91°约对应136脉冲，25 RPM已经足够快速，
    // 位置外环还会按剩余距离自动降低目标RPM，不会一直满速冲向目标轴位。
    config.motorRpm = 6;

    // ZDT 0xF6 speed slope is an explicit RPM/s value. Keep it above the
    // software acceleration limit so the software trajectory remains active.
    config.motorSpeedSlopeRpmS = 200;

    // Each cycle reads Emm V5 0x36 position and 0x35 speed, then sends 0xF6.
    config.motorCommandHz = 50;

    // Preserve the physical position-loop gain after changing from 6400 PPR.
    config.motorPositionKpRpmPerStep = 0.72;

    // 速度误差乘以本项得到期望加速度；ZDT内部仍使用自己的20 kHz速度闭环。
    config.motorVelocityKpPerSecond = 8.0;

    // 软件加速度环和跃度限制。推拉连杆换向时先平滑减速过零，再反向加速。
    config.motorMaximumAccelerationRpmS = 200.0;
    config.motorMaximumJerkRpmS3 = 300.0;

    // Use the same bounded acceleration for the braking-distance model.
    config.motorBrakingAccelerationRpmS = 200.0;

    // 0.75 step is the exact scale conversion; use one feedback pulse.
    config.motorPositionToleranceSteps = 0.35;
    config.motorStopSpeedRpm = 1.0;
    config.motorEncoderSpeedFilterSeconds = 0.04;

    // Preserve the physical travel range of the former +/-360-step limit.
    config.motorSoftLimitSteps = 11;

    // Driver Response must be None. Read commands still return 0x30/0x35
    // data, while suppressing asynchronous 0xF6 ACK frames at 50 Hz.
    config.motorExpectCommandAck = false;
    config.motorReplyTimeoutMs = 20;
    config.motorMaximumConsecutiveFailures = 3;

    // 【极其重要】motorEnabled=true时这里也必须改true，程序才允许启动。
    // 它会把启动瞬间的当前位置清为0；此时水管必须真实水平且未顶机械限位。
#if defined(BALL_E611_VIDEO_ONLY)
    config.zeroOnStart = false;
#else
    config.absoluteEncoderHomeOnStart = true;
    config.absoluteEncoderHomeRpm = 6;
    config.absoluteEncoderHomeTimeoutMs = 5000;
    config.absoluteEncoderHomePollMs = 40;
    config.zeroOnStart = false;
#endif

    // ---------------- 8. ZDT启动与退出等待 ----------------
    // 这些等待让驱动器有时间完成使能、停止和清零，一般保持默认。
    config.enableSettleMs = 80;
    config.stopSettleMs = 30;
    config.zeroSettleMs = 50;

    // 退出时继续运行位置/速度/加速度环回到编码器零位，到位或超时后再立即停止。
    config.exitReturnTimeoutMs = 1800;

    // ---------------- 9. 丢球保护 ----------------
    // 短时运动模糊先保持最后角度；超过150 ms开始平滑回水平，500 ms归零。
    config.lostHoldMs = 150;
    config.lostNeutralMs = 500;

    // ---------------- 10. 启动、显示和数据记录 ----------------
    // false最安全：启动后处于PAUSED并保持水平，按空格才进入闭环。
    config.startArmed = false;
    // E611图传直接发送displayFrame，不再创建X11/OpenCV本地窗口。
    // 普通SSH终端仍可直接按SPACE/R/Q；正式比赛应接车载实体启动按钮。
    config.gui = false;
    config.terminalKeys = true;
    config.csv = false;
    config.runtimeLogEnabled = true;
    config.runtimeLogDirectory = "logs";
    config.runtimeLogEvent = "task1_pdi_asym";
    config.runtimeLogIntervalMs = 200;

    // 每个控制帧都提交最新预览。SSH/X11显示慢时只丢旧预览帧，
    // 不参与钢球识别、丢球计数或电机控制。
    config.previewEveryNFrames = 1;

    // E611只传输网络数据：程序把带识别标记的displayFrame编码成
    // H.264/MPEG-TS，通过树莓派eth0向场外电脑发送。识别仍保持120 FPS，
    // 图传独立限为30 FPS；VLC打开udp://@:5600即可显示和录像。
    config.videoStreamEnabled = true;
    config.videoStreamHost = "192.168.137.1";
    config.videoStreamPort = 5600;
    config.videoStreamFps = 30;
    config.videoStreamBitrateKbps = 1000;

    // ========================================================================
    //                       B. 系统参数修改区结束
    // ========================================================================
    return config;
}

#if defined(__linux__) || defined(BALL_STEPPER_SYNTAX_CHECK)

int main()
{
    // 启用OpenCV的SIMD/NEON优化；树莓派5先使用4个OpenCV工作线程。
    cv::setUseOptimized(true);
    cv::setNumThreads(4);

    // 信号函数只请求结束，真正的电机回零和stop在主循环退出部分完成。
    std::signal(SIGINT, ball_stepper::onSignal);
    std::signal(SIGTERM, ball_stepper::onSignal);

    // 第3题专用版没有题号命令行参数，启动后按空格开始本轮。
    const ball_stepper::AppConfig config = makeUserConfig();
    std::fprintf(stderr,
        "TASK3 TUNING: minus5_x=%.1f bias=%.2f "
        "Kp[R=%.3f L=%.3f] Kd=%.3f Ki=%.3f "
        "limit[R=%.3f+%.3f L=%.3f+%.3f]\n",
        config.minus5CalibrationPoint.x,
        config.task3NegativeTargetBiasCm,
        config.task3MoveRightPositionKpDegPerCm,
        config.task3MoveLeftPositionKpDegPerCm,
        config.task3VelocityKdDegPerCmS,
        config.task3IntegralKiDegPerCmSecond,
        config.task3MoveRightOutputAngleLimitDeg,
        config.task3MoveRightBreakawayMaximumAngleDeg,
        config.task3MoveLeftOutputAngleLimitDeg,
        config.task3MoveLeftBreakawayMaximumAngleDeg);
    return ball_stepper::runTask3App(config);
}

#else

int main(int, char**)
{
    // Windows没有树莓派V4L2和GPIO14/15串口，本分支只用于给出明确提示。
    std::fprintf(stderr,
        "This project is intended for Raspberry Pi/Linux.\n");
    return 1;
}

#endif
