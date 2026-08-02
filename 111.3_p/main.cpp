// main.cpp
// ============================================================================
// 第3题预测制动版：O到O+5固定驱动，到点立即折返；返回途中按速度计算
// 制动距离，最后在O-5用小幅PD稳定。视觉每帧同时使用反光暗斑和完整霍夫圆，
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

    // 固定安装后关闭自动对焦，避免识别过程中镜头反复改变清晰度和钢球外观。
    config.disableAutofocus = true;

    // 【当前保持false】完全不让OpenCV改曝光。你已经验证这只摄像头原设置能
    // 640x480 MJPG 120 FPS；程序强制切自动/手动曝光反而可能掉帧并增加拖影。
    // 如果必须手动曝光，先用v4l2-ctl确认单位，保证曝光时间小于8.3 ms。
    config.configureExposure = false;
    config.useManualExposure = false;
    config.exposureAbsolute = 45.0;  // 如后续重开手动曝光，先从4.5 ms试起。

    // ---------------- 2. ROI和水管轴线 ----------------
// 黄框和实际识别ROI都覆盖整段可见水管；最新三个位置为x=176、285、390。
    // 两者使用同一个矩形，避免钢球明明还在黄色框内却已经离开算法搜索范围。
    config.pipeDisplayArea = cv::Rect(10, 210, 535, 65);
    config.roi = config.pipeDisplayArea;
    config.drawPipeDetectionArea = true;

    // 摄像头目前看不到整根水管，因此先用第3题-5 cm和+5 cm两个实测点
    // 定义运动轴线方向；实际位置换算由下面的三点分段标定完成。
    config.axisLeft = cv::Point2f(176.0f, 240.0f);
    config.axisRight = cv::Point2f(390.0f, 240.0f);
    config.axisConfigured = true;

    // 【必须实测】上面两个图像点之间对应的真实有效滚球长度，单位厘米。
    config.pipeLengthCm = 10.0;

    // 【第3题核心标定】最新实测：O-5/O/O+5的x分别约198/300/410。
    // 左侧5 cm对应102像素，右侧5 cm对应110像素，分段换算会分别使用。
    // 三个点当前都使用实测水管轴线y=240。
    config.useThreePointPositionCalibration = true;
    config.minus5CalibrationPoint = cv::Point2f(176.0f, 240.0f);
    config.centerCalibrationPoint = cv::Point2f(285.0f, 240.0f);
    config.plus5CalibrationPoint = cv::Point2f(390.0f, 240.0f);

    // 兼容基础控制模块的初始目标，比赛运行时实际目标由题目管理器自动给出。
    config.targetCm = config.pipeLengthCm * 0.5;

    // ---------------- 3. 第3题自动流程 ----------------
    // 第3题规定的两个目标相对中心O分别为+5 cm和-5 cm，一般不要改。
    config.task3OffsetCm = 5.0;
    // 实机最终仍停在x=194右侧，因此内部控制终点向左补偿增加到0.40 cm；
    // x=194仍是真实-5标定点，不修改视觉坐标含义。
    config.task3NegativeTargetBiasCm = 0.0;

    // 题目只要求在+5 cm处“到达后折返”，允许最大误差1 cm，不要求停稳。
    // 真实位置越过+4.00 cm后连续2帧确认再折返；利用剩余惯性接近+5，
    // 单个错误颜色候选把状态机从O附近直接推进到返程。
    config.task3PositiveToleranceCm = 2.00;
    config.task3ArrivalConfirmFrames = 2;

    // 两倍版幅度过大，按现场要求整体降低30%为0.91°。
    // maximumPipeAngleDeg必须同步，否则最终请求会被另一项重新截断。
    config.task3MinimumTravelAngleDeg = 0.35;

    // 返程倾角由两倍版0.20°降低30%为0.14°。
    config.task3ReturnAngleLimitDeg = 0.0728;

    // 保留配置字段用于兼容，但远距离返程已恢复固定0.10°；接近O-5后才提前制动。
    config.task3ReturnCruiseSpeedCmS = 4.5;
    config.task3ReturnSpeedKpDegPerCmS = 0.05;
    config.task3ReturnApproachZoneCm = 0.80;

    // 【真正解决提前回正的参数】只有连续确认越过O-5后，才允许反向制动。
    config.task3ReturnBrakeAngleLimitDeg = 0.10192;

    // 该Kd现在只用于-5附近的小幅稳定，不参与远距离返程，所以不会再
    // 出现“离-5还很远，Kd就把水管反向抬起”的问题。
    config.task3ReturnKd = 0.010;

    // 【最重要的现场动态参数】实机仍在O-5前停下，说明原先7.0 cm/s^2
    // 把制动距离估计得过长；提高到8.5后会明显推迟满幅反向制动。
    // 程序按stopDistance=v^2/(2a)+margin决定刹车点：
    // 制动余量维持0.10 cm；返程速度由速度环限制，制动仍由目标锁存控制。
    // 若下一次变成冲过O-5，只把0.10提高到0.25，不要再改两个角度。
    config.task3BrakingAccelerationCmS2 = 8.5;
    config.task3BrakingMarginCm = 0.10;

    // 【本次核心修正】离O-5超过2.5 cm时不进入终点制动窗口；更重要的是，
    // 在目标穿越锁存以前完全禁止反向制动，防止钢球在O-5前被推回中心。
    // 该距离仍作为越过目标后的安全制动窗口，滞环参数保留以兼容配置格式。
    config.task3MaximumBrakeStartDistanceCm = 2.5;
    config.task3BrakeReleaseHysteresisCm = 0.25;

    // 终点制动和保持也从两倍版同步降低30%。
    config.task3SettleZoneCm = 2.5;
    config.task3SettleAngleLimitDeg = 0.060;
    config.task3CreepAngleDeg = 0.015;
    config.task3CreepErrorCm = 0.15;
    config.task3CreepSpeedCmS = 0.25;

    // 题目允许O-5最大位置误差≤1 cm；这里收紧到0.50 cm，避免在刻度前
    // 0.8 cm就显示完成，让终点小幅控制继续把钢球送到更接近-5的位置。
    // 同时还必须满足“速度≤1.5 cm/s”，不能只靠高速穿过目标就判定完成。
    // 并连续维持80 ms后，画面才显示TASK3 DONE。
    config.task3FinalToleranceCm = 0.20;
    // 球在补偿目标右侧时只允许0.02 cm误差；左侧也只允许0.20 cm，
    // 防止状态机在明显偏离-5的位置提前进入HOLD。
    config.task3FinalRightToleranceCm = 0.02;
    // 结束判定只用于决定何时显示 TASK3 DONE，不会改变左右运动幅度。
    // 允许低速阈值由 1.0 放宽到 1.5 cm/s，避免球已到 O-5 附近却因轻微抖动一直等到超时。
    config.task3FinalSpeedCmS = 1.5;

    // 已满足位置和低速门限后连续80 ms即可判定完成；HOLD仍会继续保持O-5。
    config.task3FinalStableMs = 80;

    // 题目规定第3题总时间≤5 s；超过后画面显示TIMEOUT，但程序仍继续控制到-5 cm。
    config.task3TimeLimitMs = 5000;

    // ---------------- 4. 钢球PD外环 ----------------
    // 控制规律：目标水管角度 = Kp×位置误差 + Kd×钢球速度。
    // 正角度定义为axisRight端升高，让右侧钢球向左减速。

    // Kp只用于-5附近2.5 cm范围的小幅稳定；远距离行程由上面的固定幅度控制。
    config.kp = 0.025;  // deg/cm

    // 第3题阶段控制不读取通用kd；保留它仅供基础PdController兼容使用。
    config.kd = 0.0;

    // 制动距离依赖速度，滤波不能太滞后。速度显示明显抖动时再加到0.045。
    config.speedFilterSeconds = 0.035;

    // 位置与速度同时进入这两个范围才认为球稳定，避免中心附近电机反复微动。
    config.centerDeadbandCm = 0.12;
    config.stopSpeedCmS = 0.8;

    // 全局硬上限同步降低30%为0.91°。
    config.maximumPipeAngleDeg = 0.91;

    // 只提高角度建立速度，不改变0.91°幅度。25°/s使正反切换更快，仍保留
    // 平滑过渡；若实机出现明显机械冲击，再单独降回20°/s。
    config.angleSlewDegS = 3.0;

    // ---------------- 5. 曲柄连杆尺寸 ----------------
    // 【必须实测】电机输出轴中心到实际使用曲柄孔中心的距离，不是圆盘直径。
    config.crankRadiusMm = 15.0;

    // 【必须实测】水管固定铰链中心到连杆与水管连接点中心的距离。
    config.actuatorDistanceMm = 250.0;

    // 【必须与ZDT细分设置一致】200整步×细分数：8细分1600，16细分3200。
    config.pulsesPerRevolution = 3200;

    // +1表示正脉冲应使axisRight端升高；实际相反时只改成-1。
    config.motorSign = 1;

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

    // ZDT 0xF6硬件曲线加减速档。手册规定0会关闭曲线加减速，本项目禁止为0。
    // 档位12约对应82 RPM/s，略高于下面的软件最大加速度65 RPM/s。
    config.motorAcceleration = 12;

    // 每周期读取0x36位置和0x35速度，再发送一条0xF6；115200波特率下50 Hz有余量。
    config.motorCommandHz = 50;

    // 轴位外环仍受25 RPM和制动速度上限约束，不会因目标脉冲翻倍而无限加速。
    config.motorPositionKpRpmPerStep = 0.045;

    // 速度误差乘以本项得到期望加速度；ZDT内部仍使用自己的20 kHz速度闭环。
    config.motorVelocityKpPerSecond = 8.0;

    // 软件加速度环和跃度限制。推拉连杆换向时先平滑减速过零，再反向加速。
    config.motorMaximumAccelerationRpmS = 20.0;
    config.motorMaximumJerkRpmS3 = 300.0;

    // 制动速度公式使用45 RPM/s，低于最大65，补偿加速度经跃度限制后不能瞬间建立。
    config.motorBrakingAccelerationRpmS = 8.0;

    // 约1.5脉冲且实测转速不高于1.5 RPM才认为目标轴位已稳定。
    config.motorPositionToleranceSteps = 1.5;
    config.motorStopSpeedRpm = 1.0;

    // 0.91°按当前曲柄模型约136脉冲，软限位收回到±180并保留余量。
    config.motorSoftLimitSteps = 180;

    // 驱动器菜单Response必须为Receive或Both，所有控制命令都核对02成功应答。
    // This driver is configured without control-command replies. Position and
    // speed queries still require valid 0x36/0x35 responses every motor cycle.
    config.motorExpectCommandAck = false;
    config.motorReplyTimeoutMs = 15;

    // 【极其重要】motorEnabled=true时这里也必须改true，程序才允许启动。
    // 它会把启动瞬间的当前位置清为0；此时水管必须真实水平且未顶机械限位。
    config.zeroOnStart = true;

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

    // 每个控制帧都提交最新预览。SSH/X11显示慢时只丢旧预览帧，
    // 不参与钢球识别、丢球计数或电机控制。
    config.previewEveryNFrames = 1;

    // E611只传输网络数据：程序把带识别标记的displayFrame编码成
    // H.264/MPEG-TS，通过树莓派eth0向场外电脑发送。识别仍保持120 FPS，
    // 图传独立限为30 FPS；VLC打开udp://@:5600即可显示和录像。
    config.videoStreamEnabled = true;
    config.videoStreamHost = "192.168.50.1";
    config.videoStreamPort = 5600;
    config.videoStreamFps = 30;
    config.videoStreamBitrateKbps = 4000;

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
        "angle(+5)=%.3f angle(-5)=%.3f cruise=%.2f\n",
        config.minus5CalibrationPoint.x,
        config.task3NegativeTargetBiasCm,
        config.task3MinimumTravelAngleDeg,
        config.task3ReturnAngleLimitDeg,
        config.task3ReturnCruiseSpeedCmS);
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
