// main.cpp
// ============================================================================
// 第4问速度模式版：小车从A到B顺时针行驶时，钢球持续保持在O点。
// 视觉每帧同时使用反光暗斑和完整霍夫圆，
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

// 复用第一问当前稳定曝光；需要现场改亮度时只改这一处。
#define BALL_CFG_CONFIGURE_EXPOSURE 1
#define BALL_CFG_USE_MANUAL_EXPOSURE 1
#define BALL_CFG_EXPOSURE_ABSOLUTE 60.0

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
#define BALL_CFG_HOUGH_ACCUM_ACQUIRE 9.0
#define BALL_CFG_HOUGH_ACCUM_TRACK 10.0
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
#include "task4_app.hpp"

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

    config.configureExposure = BALL_CFG_CONFIGURE_EXPOSURE != 0;
    config.useManualExposure = BALL_CFG_USE_MANUAL_EXPOSURE != 0;
    config.exposureAbsolute = BALL_CFG_EXPOSURE_ABSOLUTE;

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

    // 实测三点标定：O-5/O/O+5的x分别为167/275/398。
    // 左侧5 cm对应108像素，右侧5 cm对应123像素，分段换算会分别使用。
    // 三个点当前都使用实测水管轴线y=247。
    config.useThreePointPositionCalibration = true;
    config.minus5CalibrationPoint = cv::Point2f(167.0f, 247.0f);
    config.centerCalibrationPoint = cv::Point2f(275.0f, 247.0f);
    config.plus5CalibrationPoint = cv::Point2f(398.0f, 247.0f);

    config.positionCalibrationOffsetCm = 5.0;

    // O点位于10 cm有效段的中点。
    config.targetCm = config.pipeLengthCm * 0.5;

    // ---------------- 3. 第4问钢球PI-D外环 ----------------
    config.task4TargetCm = config.targetCm;
    config.task4EvaluationSeconds = 8.0;
    config.task4AllowedErrorCm = 1.0;
    // 等待阶段球只要位于O点±1 cm并基本静止即可开始。
    config.task4StartToleranceCm = 1.00;
    config.task4StartSpeedCmS = 2.0;
    config.task4StartConfirmFrames = 6;

    // 复用第一问回程已验证的对称PD量级。D来自两帧位置差，极性与P一致，
    // 钢球向哪边运动就提前抬高同侧水管，直接阻挡速度。
    config.task4Kp = 0.110;
    config.task4Kd = 0.080;

    // I只在中心±1 cm且速度较小时工作，用于补偿小车持续加速度和机构静差。
    // 最大只贡献±0.12°，换边立即清掉旧极性积分。
    config.task4Ki = 0.150;
    config.task4IntegralZoneCm = 1.00;
    config.task4IntegralSpeedLimitCmS = 1.00;
    config.task4IntegralLimitDeg = 0.12;
    config.task4IntegralLeakSeconds = 4.0;
    config.task4LevelTrimDeg = 0.0;
    config.task4DeadbandCm = 0.02;
    config.task4StopSpeedCmS = 0.15;

    // P/I/vehicle-feedforward share the drive envelope. D is added last as
    // braking authority. With no vehicle encoder source, FF stays exactly zero.
    config.task4DriveAngleLimitDeg = 0.80;
    config.task4BrakeAngleLimitDeg = 0.91;
    config.task4AngleSlewDegS = 8.0;
    config.task4LossFailureMs = 180;

    // ---------------- 4. Chassis acceleration feedforward ----------------
    // The chassis encoder is cleared every 50 ms and returns one speed value.
    // Typical values are 50-100 and normal cruise is about 80. A future
    // VehicleEncoderSource only supplies that value and its sample timestamp.
    config.task4VehicleEncoderCruiseValue = 80.0;
    config.task4VehicleEncoderMaximumAbsValue = 150.0;
    config.task4VehicleEncoderDirectionSign = +1.0;
    config.task4VehicleSpeedFilterSeconds = 0.035;
    config.task4VehicleAccelerationFilterSeconds = 0.060;
    config.task4VehicleAccelerationDecaySeconds = 0.120;
    config.task4VehicleAccelerationDeadbandUnitsS = 15.0;
    config.task4VehicleAccelerationLimitUnitsS = 2000.0;

    // Positive chassis acceleration toward axisRight makes the ball lag left,
    // so the default compensation is a negative pipe angle. Flip only this
    // sign if the supplied chassis-speed coordinate is opposite.
    config.task4VehicleAccelerationAngleSign = -1.0;
    config.task4VehicleFeedforwardDegPerUnitS = 0.00040;
    config.task4VehicleFeedforwardLimitDeg = 0.65;
    config.task4VehicleInputTimeoutMs = 140;
    config.task4VehicleSampleMaximumGapMs = 120;

    // v[n]=(x[n]-x[n-2])/(t[n]-t[n-2])，之后只做很轻的低通。
    config.speedDifferenceFrames = 2;
    config.speedFilterSeconds = 0.020;

    // 机构总安全限位大于第四问控制限幅，形成第二层保护。
    config.maximumPipeAngleDeg = 0.91;

    // ---------------- 5. 曲柄连杆尺寸 ----------------
    // 【必须实测】电机输出轴中心到实际使用曲柄孔中心的距离，不是圆盘直径。
    config.crankRadiusMm = 15.0;

    // 【必须实测】水管固定铰链中心到连杆与水管连接点中心的距离。
    config.actuatorDistanceMm = 250.0;

    // 驱动器当前为1.8°整步模式，命令量纲按200 PPR计算。
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
    config.motorEnabled = true;

    // 树莓派5：GPIO14/TXD物理8脚接ZDT RX，GPIO15/RXD物理10脚接ZDT TX，GND共地。
    // 当前这台Ubuntu系统已确认GPIO14/15对应/dev/ttyAMA0，且没有/dev/serial0别名。
    config.serialPort = "/dev/ttyAMA0";
    config.serialBaud = 115200;   // 必须与ZDT驱动器设置一致，格式8N1。

    // ZDT通信地址。单电机通常为1，但必须以驱动器面板/上位机实际设置为准。
    config.motorAddress = 1;

    // 速度模式的最大转速。0.91°约对应136脉冲，25 RPM已经足够快速，
    // 位置外环还会按剩余距离自动降低目标RPM，不会一直满速冲向目标轴位。
    config.motorRpm = 6;

    // 电机加速度与第一问一致，统一使用200 RPM/s，不能给满量程。
    config.motorSpeedSlopeRpmS = 200;

    // 每周期读取0x36位置和0x35速度，再发送一条0xF6；115200波特率下50 Hz有余量。
    config.motorCommandHz = 50;

    // 轴位外环受6 RPM和制动速度上限约束，不会因目标脉冲变化而无限加速。
    config.motorPositionKpRpmPerStep = 0.72;

    // 速度误差乘以本项得到期望加速度；ZDT内部仍使用自己的20 kHz速度闭环。
    config.motorVelocityKpPerSecond = 8.0;

    // 软件加速度环和跃度限制。推拉连杆换向时先平滑减速过零，再反向加速。
    config.motorMaximumAccelerationRpmS = 200.0;
    config.motorMaximumJerkRpmS3 = 300.0;

    // 制动距离模型与软件和驱动器统一按200 RPM/s计算。
    config.motorBrakingAccelerationRpmS = 200.0;

    // 误差约0.35脉冲且实测转速不高于1 RPM才认为目标轴位已稳定。
    config.motorPositionToleranceSteps = 0.35;
    config.motorStopSpeedRpm = 1.0;
    config.motorEncoderSpeedFilterSeconds = 0.04;

    // 0.91°按200 PPR量纲约10脉冲，软限位设为±11脉冲。
    config.motorSoftLimitSteps = 11;

    // 驱动器Response当前为None；位置和速度查询仍返回有效数据，0xF6不等ACK。
    config.motorExpectCommandAck = false;
    config.motorReplyTimeoutMs = 20;
    config.motorMaximumConsecutiveFailures = 3;

    // 正式模式先回到已保存的绝对编码器LEVEL零点，再清运行时位置坐标。
#if defined(BALL_E611_VIDEO_ONLY)
    config.absoluteEncoderHomeOnStart = false;
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
    // 短时运动模糊先保持最后角度；超过60 ms开始回水平，180 ms归零并判丢球。
    config.lostHoldMs = 60;
    config.lostNeutralMs = 180;

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
    config.runtimeLogEvent = "task4_balance";
    config.runtimeLogIntervalMs = 100;

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

    // 第4问专用版没有题号参数；钢球在O点稳定后按空格开始8秒考核。
    const ball_stepper::AppConfig config = makeUserConfig();
    std::fprintf(stderr,
        "TASK4 VELOCITY: O=(%.1f,%.1f) limit=%.2fcm time=%.1fs "
        "PDI=%.3f/%.3f/%.3f drive=%.3f brake=%.3fdeg\n",
        config.centerCalibrationPoint.x,
        config.centerCalibrationPoint.y,
        config.task4AllowedErrorCm,
        config.task4EvaluationSeconds,
        config.task4Kp,
        config.task4Ki,
        config.task4Kd,
        config.task4DriveAngleLimitDeg,
        config.task4BrakeAngleLimitDeg);
    return ball_stepper::runTask4VelocityApp(config);
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
