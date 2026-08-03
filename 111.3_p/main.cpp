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

// 新灰色水管实测：钢球本体半径约9～11 px；排除约15 px的外层阴影圆。
#define BALL_CFG_RADIUS_MIN 8.0f
#define BALL_CFG_RADIUS_MAX 13.5f
#define BALL_CFG_RADIUS_EXPECTED 11.0f

#define BALL_CFG_MIN_DETECTION_SCORE 0.34f

// 【识别调参】首次锁定比持续跟踪严格，防止无球时锁到螺丝或阴影。
#define BALL_CFG_MIN_ACQUIRE_SCORE 0.44f

// 静止跟踪时第一名和第二名过于接近会拒绝跳目标；行进时不使用该拒绝，
// 否则真球运动后大量霍夫候选会造成连续BALL LOST。
#define BALL_CFG_AMBIGUITY_GAP 0.050f

#define BALL_CFG_ACQUIRE_FRAMES 6

#define BALL_CFG_MAX_MISSES 18

// 首次连续确认和锁定后的预测搜索门限，单位像素。
// 当前钢球每帧正常位移远小于原来的20～58 px；门限过大会允许绿圈跳到管壁假圆。
#define BALL_CFG_ACQUIRE_GATE_PX 16.0f
#define BALL_CFG_TRACK_GATE_MIN_PX 10.0f
#define BALL_CFG_TRACK_GATE_MAX_PX 45.0f

// 新灰管上的钢球可能比外环亮或暗，因此对比度按绝对值判断。
// 真球样本的圆边缘支持约0.98～1.00、内部纹理标准差约29～65。
#define BALL_CFG_ACQUIRE_CONTRAST_MIN 0.0f
#define BALL_CFG_ACQUIRE_EDGE_SUPPORT_MIN 0.65f
#define BALL_CFG_ACQUIRE_RING_MEAN_MIN 40.0f
#define BALL_CFG_ACQUIRE_INNER_STD_MIN 20.0f

// ---------------- 灰度霍夫圆专用参数 ----------------

// 轴线已按最新画面的y=240标定，正式识别启用轴线过滤，排除管外螺丝和圆孔。
#define BALL_CFG_USE_AXIS_GATE 1
#define BALL_CFG_AXIS_GATE_PX 10.0f

// 局部灰度增强、霍夫圆和圆心二次拟合参数。一般保持以下实测初值。
// 真实曝光固定在实测最亮的60档后，算法输入再小幅+10；继续加到+20
// 会削弱球周围暗环。该处理不延长曝光时间，不降低120 FPS帧率。
#define BALL_CFG_GRAY_BRIGHTNESS 10
#define BALL_CFG_CLAHE_CLIP_LIMIT 2.2
#define BALL_CFG_HOUGH_DP 1.0
#define BALL_CFG_HOUGH_CANNY_HIGH 74.0
#define BALL_CFG_HOUGH_ACCUM_ACQUIRE 8.0
#define BALL_CFG_HOUGH_ACCUM_TRACK 8.0
#define BALL_CFG_REFINE_GRADIENT_MIN 16.0f
#define BALL_CFG_REFINE_RESIDUAL_MAX 2.0f
// 真球圆周梯度基本指向圆心；灰槽水平反光假圆实测低于0.70。
#define BALL_CFG_HOUGH_RADIAL_SUPPORT_MIN 0.72f

// 新灰管每帧会产生约24～42个霍夫圆；精修前16个，避免真球因票数不是前4而漏检。
// 锁定后仍主要依靠局部预测窗口，扩大首次候选数不会放宽运动轨迹门限。
#define BALL_CFG_HOUGH_INTERVAL 8
#define BALL_CFG_HOUGH_MAX_CANDIDATES 16

// 第3题开始前球必须在O点。新管中点实测为x约269、y约247；
// 首次只接收锚点15 px内的候选，之后跟踪仍覆盖完整左右行程。
#define BALL_CFG_INITIAL_ACQUIRE_GATE_PX 15.0f

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

    // 新灰管按同一摄像头会话逐档实测，60档的ROI平均亮度最高且12帧波动
    // 只有约0.41灰度；继续增大到65会进入另一档较暗的传感器时序。
    // 60通常对应约6.0 ms，仍短于120 FPS的8.3 ms单帧周期。
    config.configureExposure = true;
    config.useManualExposure = true;
    config.exposureAbsolute = 60.0;

    // ---------------- 2. ROI和水管轴线 ----------------
// 黄框仍显示整段水管；识别只处理轴线附近的45 px高窄带，排除上下金属边。
    // 横向仍覆盖完整左右行程，不限制钢球到达+5 cm和-5 cm。
    config.pipeDisplayArea = cv::Rect(10, 210, 535, 65);
    config.roi = cv::Rect(10, 225, 535, 45);
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
    config.centerCalibrationPoint = cv::Point2f(269.0f, 247.0f);
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
    // 输出角度 = Kp(按运动方向选择) * (钢球位置 - 目标位置)
    //          + Kd * 两帧差速度 + Ki * 误差积分。
    // 负角度使球向右运动，正角度使球向左运动；以下参数全部是反馈增益，
    // 不是写死的固定运动角度。机构左右传动能力不同，因此左右 P、限幅和脱困量分开设置。

    // 向右运动的位置 P 增益，单位 deg/cm；增大后右移响应更快、更硬，过大会过冲或振荡。
    config.task3MoveRightPositionKpDegPerCm = 0.200;

    // 向左运动的位置 P 增益，单位 deg/cm；增大后左移响应更快，过大会在左侧来回振荡。
    config.task3MoveLeftPositionKpDegPerCm = 0.060;

    // 速度 D 增益，单位 deg/(cm/s)；直接使用 x[n] 与 x[n-2] 算出的原始速度。
    // 增大可更早抑制高速冲向目标的趋势，但过大会放大视觉位置抖动并引起高频摆动。
    config.task3VelocityKdDegPerCmS = 0.040;

    // I 增益，单位 deg/(cm*s)；决定累计误差转成补偿角度的速度。
    // 增大可更快消除静差，但过大会积分过冲、低频振荡；当前最大 I 输出为 0.400 deg。
    config.task3IntegralKiDegPerCmSecond = 0.100;

    // 积分位置窗口，单位 cm；仅当 |位置误差| <= 3 cm 时允许积分。
    // 增大可更早启用 I，但也更容易在高速接近阶段积累过多误差。
    config.task3IntegralZoneCm = 2.00;

    // 积分速度门限，单位 cm/s；仅当两帧差速度绝对值不超过此值时才累计积分。
    // 增大后 I 更容易介入，减小则必须更接近静止才介入。
    config.task3IntegralSpeedLimitCmS = 2;

    // 积分状态限幅，单位 cm*s；与 Ki 相乘得到 I 项最大角度：0.400 * 1.00 = 0.400 deg。
    // 增大可对抗更大的长期偏差，但会增加积分释放时的过冲；每次目标反向都会清零。
    config.task3IntegralLimitCmSeconds = 1.00;

    // 脱困触发的最小剩余误差，单位 cm；只有 |误差| >= 此值才认为仍需克服虚位/静摩擦。
    // 减小会更靠近目标仍持续加力，过小可能破坏最终稳定；增大可能留下较大静差。
    config.task3BreakawayErrorCm = 0.35;

    // 脱困触发的最大球速，单位 cm/s；只有球速绝对值不超过此值才判定“基本没动”。
    // 增大后更容易触发脱困，过大会在球仍运动时继续加力；减小则触发更谨慎。
    config.task3BreakawaySpeedCmS = 0.35;

    // 脱困触发延时，单位 s；误差较大且球近乎不动持续 0.08 s 后才开始增加补偿。
    // 增大可过滤短时误判但破除虚位更慢，减小则响应更快但更容易误触发。
    config.task3BreakawayDelaySeconds = 0.08;

    // 脱困角度爬升速度，单位 deg/s；决定触发后附加角度建立得多快。
    // 增大可更快顶过虚位和静摩擦，过大会突然冲球；条件消失时按该速度的 3 倍退回零。
    config.task3BreakawayRampDegPerSecond = 0.20;

    // 向右运动时允许叠加的最大脱困角度，单位 deg；只在反馈确认卡滞后加入基础 PDI。
    // 增大可提高向右破静摩擦能力，但会增加启动突跳和越过目标的风险。
    config.task3MoveRightBreakawayMaximumAngleDeg = 0.11;

    // 向左运动时允许叠加的最大脱困角度，单位 deg；用于补偿左向机构虚位/静摩擦。
    // 增大可提高向左脱困能力，但过大会使左移启动过猛。
    config.task3MoveLeftBreakawayMaximumAngleDeg = 0.28;

    // 向右基础 PDI 输出限幅，单位 deg；P+D+I 正常最多输出 -0.80 deg。
    // 增大可提高右移加速度，但更易过冲；脱困激活后可再叠加 0.11 deg，最终到 -0.91 deg。
    config.task3MoveRightOutputAngleLimitDeg = 0.80;

    // 向左基础 PDI 输出限幅，单位 deg；P+D+I 正常最多输出 +0.60 deg。
    // 增大可提高左移加速度，但更易振荡；脱困激活后可再叠加 0.28 deg，最终到 +0.88 deg。
    config.task3MoveLeftOutputAngleLimitDeg = 0.60;

    // 速度差分帧数；当前 2 表示 v[n]=(x[n]-x[n-2])/(t[n]-t[n-2])。
    // 帧数增大可降低单帧噪声但增加速度反馈延迟；当前控制器校验要求必须为 2。
    config.speedDifferenceFrames = 2;

    // 速度低通滤波时间常数，单位 s；增大后显示/时序使用的速度更平滑但延迟更大。
    // 注意：PDI 的 D、积分速度门和脱困速度门直接使用上面的原始两帧差速度，不经过此低通。
    config.speedFilterSeconds = 0.020;

    // 水管最终机械安全角度限幅，单位 deg；基础 PDI 与脱困叠加后仍不得超过 +/-0.91 deg。
    // 增大前必须确认机构行程和连杆安全，减小会直接限制整个控制器的最大控制能力。
    config.maximumPipeAngleDeg = 0.91;

    // 水管目标角度变化率上限，单位 deg/s；限制每秒角度指令能变化多少，避免瞬间跳变。
    // 增大后跟随更快但冲击更大，减小后更平滑但响应变慢；它不是电机的 200 RPM/s 加速度。
    config.angleSlewDegS = 8.0;

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
