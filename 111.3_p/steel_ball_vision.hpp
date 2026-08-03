#pragma once

// steel_ball_vision.hpp
// ============================================================================
// 120 FPS灰度暗斑质心+霍夫验证钢球识别器。
//
// 本版本与原111的主要区别：
// 1. 不再用黑帽连通区域产生候选，避免水管阴影和刻度线形成假目标。
// 2. BGR画面先转灰度，再做局部对比度增强和高斯去噪。
// 3. 每帧同时计算暗斑轮廓与HoughCircles；暗斑质心负责定位，霍夫负责验证
//    圆形和辅助半径，避免在球本体、外层阴影和管壁圆弧之间切换。
// 4. 两种圆心相差超过6 px时拒绝霍夫跳点；暗斑连续漏两帧后，才允许
//    接近预测位置的纯霍夫结果临时接管。
// 5. 候选必须满足直径、内外亮度差、圆周完整度、金属反光纹理等条件。
// 6. 可要求圆心靠近main.cpp标定的水管轴线，排除管外螺丝和圆形结构。
// 7. 首次锁定需要连续多帧确认；锁定后只在预测位置附近寻找。
// 8. 漏检时只保留内部预测，绝不把预测位置作为真实测量送入控制器。
//
// 注意：霍夫圆不是“参数越松越好”。参数太松会检测到水管边缘和螺丝，
// 参数太紧则会在钢球高速运动模糊时漏检。所有现场参数都集中在main.cpp。
// ============================================================================

#include <opencv2/opencv.hpp>
#ifdef __linux__
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

// ---------------- main.cpp可覆盖的默认参数 ----------------
#ifndef BALL_CFG_CAMERA_WIDTH
#define BALL_CFG_CAMERA_WIDTH 640
#endif
#ifndef BALL_CFG_CAMERA_HEIGHT
#define BALL_CFG_CAMERA_HEIGHT 480
#endif
#ifndef BALL_CFG_CAMERA_FPS
#define BALL_CFG_CAMERA_FPS 120
#endif

#ifndef BALL_CFG_RADIUS_MIN
#define BALL_CFG_RADIUS_MIN 8.0f
#endif
#ifndef BALL_CFG_RADIUS_MAX
#define BALL_CFG_RADIUS_MAX 18.0f
#endif
#ifndef BALL_CFG_RADIUS_EXPECTED
#define BALL_CFG_RADIUS_EXPECTED 13.0f
#endif

#ifndef BALL_CFG_MIN_DETECTION_SCORE
#define BALL_CFG_MIN_DETECTION_SCORE 0.38f
#endif
#ifndef BALL_CFG_MIN_ACQUIRE_SCORE
#define BALL_CFG_MIN_ACQUIRE_SCORE 0.50f
#endif
#ifndef BALL_CFG_AMBIGUITY_GAP
#define BALL_CFG_AMBIGUITY_GAP 0.05f
#endif
#ifndef BALL_CFG_ACQUIRE_FRAMES
#define BALL_CFG_ACQUIRE_FRAMES 5
#endif
#ifndef BALL_CFG_MAX_MISSES
#define BALL_CFG_MAX_MISSES 18
#endif
#ifndef BALL_CFG_ACQUIRE_GATE_PX
#define BALL_CFG_ACQUIRE_GATE_PX 26.0f
#endif
#ifndef BALL_CFG_TRACK_GATE_MIN_PX
#define BALL_CFG_TRACK_GATE_MIN_PX 20.0f
#endif
#ifndef BALL_CFG_TRACK_GATE_MAX_PX
#define BALL_CFG_TRACK_GATE_MAX_PX 58.0f
#endif

#ifndef BALL_CFG_ACQUIRE_CONTRAST_MIN
#define BALL_CFG_ACQUIRE_CONTRAST_MIN 10.0f
#endif
#ifndef BALL_CFG_ACQUIRE_EDGE_SUPPORT_MIN
#define BALL_CFG_ACQUIRE_EDGE_SUPPORT_MIN 0.35f
#endif
#ifndef BALL_CFG_ACQUIRE_RING_MEAN_MIN
#define BALL_CFG_ACQUIRE_RING_MEAN_MIN 50.0f
#endif
#ifndef BALL_CFG_ACQUIRE_INNER_STD_MIN
#define BALL_CFG_ACQUIRE_INNER_STD_MIN 3.0f
#endif

#ifndef BALL_CFG_USE_AXIS_GATE
#define BALL_CFG_USE_AXIS_GATE 1
#endif
#ifndef BALL_CFG_AXIS_GATE_PX
#define BALL_CFG_AXIS_GATE_PX 14.0f
#endif

#ifndef BALL_CFG_GRAY_BRIGHTNESS
#define BALL_CFG_GRAY_BRIGHTNESS 12
#endif
#ifndef BALL_CFG_CLAHE_CLIP_LIMIT
#define BALL_CFG_CLAHE_CLIP_LIMIT 2.2
#endif
#ifndef BALL_CFG_HOUGH_DP
#define BALL_CFG_HOUGH_DP 1.0
#endif
#ifndef BALL_CFG_HOUGH_CANNY_HIGH
#define BALL_CFG_HOUGH_CANNY_HIGH 74.0
#endif
#ifndef BALL_CFG_HOUGH_ACCUM_ACQUIRE
#define BALL_CFG_HOUGH_ACCUM_ACQUIRE 9.0
#endif
#ifndef BALL_CFG_HOUGH_ACCUM_TRACK
#define BALL_CFG_HOUGH_ACCUM_TRACK 6.0
#endif
#ifndef BALL_CFG_HOUGH_INTERVAL
#define BALL_CFG_HOUGH_INTERVAL 8
#endif
#ifndef BALL_CFG_HOUGH_MAX_CANDIDATES
#define BALL_CFG_HOUGH_MAX_CANDIDATES 4
#endif
#ifndef BALL_CFG_INITIAL_ACQUIRE_GATE_PX
#define BALL_CFG_INITIAL_ACQUIRE_GATE_PX 55.0f
#endif
#ifndef BALL_CFG_REFINE_GRADIENT_MIN
#define BALL_CFG_REFINE_GRADIENT_MIN 16.0f
#endif
#ifndef BALL_CFG_REFINE_RESIDUAL_MAX
#define BALL_CFG_REFINE_RESIDUAL_MAX 2.0f
#endif

namespace ball_stepper {

constexpr int CAMERA_WIDTH = BALL_CFG_CAMERA_WIDTH;
constexpr int CAMERA_HEIGHT = BALL_CFG_CAMERA_HEIGHT;
constexpr int CAMERA_FPS = BALL_CFG_CAMERA_FPS;
constexpr float BALL_RADIUS_MIN = BALL_CFG_RADIUS_MIN;
constexpr float BALL_RADIUS_MAX = BALL_CFG_RADIUS_MAX;
constexpr float BALL_RADIUS_EXPECTED = BALL_CFG_RADIUS_EXPECTED;
constexpr float MIN_DETECTION_SCORE = BALL_CFG_MIN_DETECTION_SCORE;
constexpr float MIN_ACQUIRE_SCORE = BALL_CFG_MIN_ACQUIRE_SCORE;
constexpr float AMBIGUITY_GAP = BALL_CFG_AMBIGUITY_GAP;
constexpr int ACQUIRE_FRAMES = BALL_CFG_ACQUIRE_FRAMES;
constexpr int MAX_MISSES = BALL_CFG_MAX_MISSES;
constexpr float ACQUIRE_GATE_PX = BALL_CFG_ACQUIRE_GATE_PX;
constexpr float TRACK_GATE_MIN_PX = BALL_CFG_TRACK_GATE_MIN_PX;
constexpr float TRACK_GATE_MAX_PX = BALL_CFG_TRACK_GATE_MAX_PX;
constexpr float ACQUIRE_CONTRAST_MIN = BALL_CFG_ACQUIRE_CONTRAST_MIN;
constexpr float ACQUIRE_EDGE_SUPPORT_MIN =
    BALL_CFG_ACQUIRE_EDGE_SUPPORT_MIN;
constexpr float ACQUIRE_RING_MEAN_MIN = BALL_CFG_ACQUIRE_RING_MEAN_MIN;
constexpr float ACQUIRE_INNER_STD_MIN = BALL_CFG_ACQUIRE_INNER_STD_MIN;
constexpr float AXIS_GATE_PX = BALL_CFG_AXIS_GATE_PX;
constexpr int HOUGH_INTERVAL = BALL_CFG_HOUGH_INTERVAL;
constexpr int HOUGH_MAX_CANDIDATES = BALL_CFG_HOUGH_MAX_CANDIDATES;
constexpr float INITIAL_ACQUIRE_GATE_PX =
    BALL_CFG_INITIAL_ACQUIRE_GATE_PX;

// ---------------- 暗斑轮廓定位的固定安全门限 ----------------
// 这些门限只用于局部暗斑候选，不改变main.cpp中的霍夫参数。
// 面积和包围盒范围覆盖当前截图中约12～13像素半径的钢球，
// 同时排除细线、管壁长边和大面积阴影。
constexpr double DARK_BLOB_AREA_MIN = 100.0;
constexpr double DARK_BLOB_AREA_MAX = 1000.0;
constexpr int DARK_BLOB_WIDTH_MIN = 12;
constexpr int DARK_BLOB_WIDTH_MAX = 45;
constexpr int DARK_BLOB_HEIGHT_MIN = 12;
constexpr int DARK_BLOB_HEIGHT_MAX = 45;
constexpr float DARK_BLOB_ASPECT_MIN_TRACK = 0.36f;
constexpr float DARK_BLOB_ASPECT_MIN_ACQUIRE = 0.52f;
constexpr float DARK_BLOB_CIRCULARITY_MIN_TRACK = 0.34f;
constexpr float DARK_BLOB_CIRCULARITY_MIN_ACQUIRE = 0.50f;
constexpr float DARK_BLOB_SOLIDITY_MIN_TRACK = 0.58f;
constexpr float DARK_BLOB_SOLIDITY_MIN_ACQUIRE = 0.72f;
// 霍夫和暗斑圆心相差不超过该值才视为同一个钢球。
// 超过时优先采用暗斑质心，避免霍夫跳到钢球阴影或管壁圆弧。
constexpr float HOUGH_BLOB_AGREEMENT_PX = 6.0f;

static_assert(CAMERA_WIDTH >= 160 && CAMERA_HEIGHT >= 120,
              "camera image size is too small");
static_assert(CAMERA_FPS > 0, "camera FPS must be positive");
static_assert(BALL_RADIUS_MIN > 0.0f &&
              BALL_RADIUS_MAX > BALL_RADIUS_MIN,
              "invalid steel-ball radius range");
static_assert(BALL_RADIUS_EXPECTED >= BALL_RADIUS_MIN &&
              BALL_RADIUS_EXPECTED <= BALL_RADIUS_MAX,
              "expected radius must be inside min/max range");
static_assert(BALL_CFG_USE_AXIS_GATE == 0 || BALL_CFG_USE_AXIS_GATE == 1,
              "BALL_CFG_USE_AXIS_GATE must be 0 or 1");
static_assert(BALL_CFG_HOUGH_DP >= 1.0,
              "Hough dp must be at least 1.0");
static_assert(BALL_CFG_HOUGH_CANNY_HIGH > 0.0 &&
              BALL_CFG_HOUGH_ACCUM_ACQUIRE > 0.0 &&
              BALL_CFG_HOUGH_ACCUM_TRACK > 0.0,
              "invalid Hough parameter");
static_assert(HOUGH_INTERVAL >= 1 && HOUGH_MAX_CANDIDATES >= 1,
              "invalid Hough scheduling parameter");
static_assert(INITIAL_ACQUIRE_GATE_PX > BALL_RADIUS_MAX,
              "initial acquire gate is too small");

using VisionClock = std::chrono::steady_clock;

template <typename T>
T clampVision(T value, T low, T high)
{
    return std::max(low, std::min(value, high));
}

inline float unitScore(float value)
{
    return clampVision(value, 0.0f, 1.0f);
}

inline int64_t visionNowMicroseconds()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        VisionClock::now().time_since_epoch()).count();
}

inline cv::Rect boundedVisionRect(const cv::Rect& rectangle,
                                  const cv::Size& size)
{
    return rectangle & cv::Rect(0, 0, size.width, size.height);
}

struct Candidate {
    bool valid = false;
    cv::Point2f center{};
    float radius = BALL_RADIUS_EXPECTED;
    float score = 0.0f;
    float contrast = 0.0f;
    float edgeSupport = 0.0f;
    float ringMean = 0.0f;
    float innerStd = 0.0f;
    float fitQuality = 0.0f;
    float axisDistance = 0.0f;
    // true表示该候选来自暗斑轮廓后备。它仍需经过同样的时序确认，
    // 但首次捕获会要求更多连续帧，降低把固定杂物当成钢球的概率。
    bool contourFallback = false;
    // houghOnly表示没有暗斑轮廓验证的纯霍夫结果；跟踪态不会立即采纳它。
    bool houghOnly = false;
    // fused表示霍夫和暗斑位置一致；圆心仍使用更稳定的暗斑质心。
    bool fused = false;
};

struct Result {
    // measured=true才代表当前帧真的检测到了钢球，可用于闭环控制。
    // locked=true且measured=false只表示跟踪器暂时保留旧轨迹。
    bool measured = false;
    bool locked = false;
    cv::Point2f center{};
    float radius = BALL_RADIUS_EXPECTED;
    float confidence = 0.0f;
    int houghCandidates = 0;
    int validCandidates = 0;
    int darkBlobCandidates = 0;
    int missStreak = 0;
    // true表示当前真实测量来自运动拖影暗斑；false表示来自霍夫圆。
    bool contourFallback = false;
    bool fused = false;
};

class SteelBallDetector {
    struct RefinedCircle {
        cv::Point2f center{};
        float radius = BALL_RADIUS_EXPECTED;
        float quality = 0.45f;
    };

    cv::Rect configuredRoi_{};
    cv::Point2f axisStart_{};
    cv::Point2f axisEnd_{};
    bool axisGateEnabled_ = false;

    bool locked_ = false;
    bool hasTrackHistory_ = false;
    int misses_ = 0;
    int acquireCount_ = 0;
    int64_t lastFrameUs_ = 0;
    uint64_t frameCounter_ = 0;
    cv::Point2f center_{};
    cv::Point2f velocity_{};
    float radius_ = BALL_RADIUS_EXPECTED;
    float confidence_ = 0.0f;
    Candidate pending_{};
    // 第3题启动前钢球必须放在O点。首次捕获只在O点附近寻找，避免
    // 程序一启动就锁到管端螺丝；失锁后该锚点更新为最后真实轨迹位置。
    cv::Point2f acquireAnchor_{};
    bool acquireAnchorConfigured_ = false;

    // 所有图像缓存按帧重复使用。gray_保留原始灰度用于明暗统计；
    // enhanced_/smooth_用于霍夫和边缘，降低曝光变化与MJPG块状噪声的影响。
    cv::Mat gray_;
    cv::Mat enhanced_;
    cv::Mat smooth_;
    cv::Mat gradientX_;
    cv::Mat gradientY_;
    cv::Mat gradient_;
    cv::Mat edgeMap_;
    cv::Ptr<cv::CLAHE> clahe_;
    // 暗斑定位只在当前search ROI内复用这块掩膜，不创建整帧二值图。
    cv::Mat darkMask_;
    cv::Mat darkKernel_;
    int lastHoughCandidates_ = 0;
    int lastValidCandidates_ = 0;
    int lastDarkBlobCandidates_ = 0;

    cv::Rect baseRoi(const cv::Size& size) const
    {
        if (configuredRoi_.width <= 0 || configuredRoi_.height <= 0) {
            return cv::Rect(0, 0, size.width, size.height);
        }
        return boundedVisionRect(configuredRoi_, size);
    }

    float distanceToAxis(const cv::Point2f& point) const
    {
        if (!axisGateEnabled_) return 0.0f;
        const cv::Point2f direction = axisEnd_ - axisStart_;
        const float length = static_cast<float>(cv::norm(direction));
        if (length < 1.0f) return 0.0f;
        // 二维叉积绝对值除以轴线长度，得到点到无限轴线的垂直距离。
        const cv::Point2f relative = point - axisStart_;
        return std::abs(relative.x * direction.y -
                        relative.y * direction.x) / length;
    }

    static bool fitCircleLeastSquares(const std::vector<cv::Point2f>& points,
                                      const cv::Point2f& origin,
                                      cv::Point2f& center,
                                      float& radius)
    {
        if (points.size() < 12) return false;

        // 使用局部坐标可避免x/y约几百像素时平方项过大，改善数值稳定性。
        cv::Mat matrix(static_cast<int>(points.size()), 3, CV_64F);
        cv::Mat values(static_cast<int>(points.size()), 1, CV_64F);
        for (int index = 0; index < static_cast<int>(points.size()); ++index) {
            const double x = points[index].x - origin.x;
            const double y = points[index].y - origin.y;
            matrix.at<double>(index, 0) = 2.0 * x;
            matrix.at<double>(index, 1) = 2.0 * y;
            matrix.at<double>(index, 2) = 1.0;
            values.at<double>(index, 0) = x * x + y * y;
        }

        cv::Mat solution;
        if (!cv::solve(matrix, values, solution, cv::DECOMP_SVD)) {
            return false;
        }

        const double centerX = solution.at<double>(0, 0);
        const double centerY = solution.at<double>(1, 0);
        const double radiusSquared =
            centerX * centerX + centerY * centerY +
            solution.at<double>(2, 0);
        if (!std::isfinite(radiusSquared) || radiusSquared <= 1.0) {
            return false;
        }

        center = origin + cv::Point2f(
            static_cast<float>(centerX),
            static_cast<float>(centerY));
        radius = static_cast<float>(std::sqrt(radiusSquared));
        return std::isfinite(center.x) && std::isfinite(center.y) &&
               std::isfinite(radius);
    }

    RefinedCircle refineCircle(const cv::Point2f& initialCenter,
                               float initialRadius) const
    {
        RefinedCircle result{initialCenter, initialRadius, 0.45f};
        const int margin = cvCeil(initialRadius * 1.38f) + 3;
        const cv::Rect area = boundedVisionRect(
            cv::Rect(cvFloor(initialCenter.x) - margin,
                     cvFloor(initialCenter.y) - margin,
                     margin * 2 + 1, margin * 2 + 1),
            gray_.size());
        if (area.width < 12 || area.height < 12) return result;

        std::vector<cv::Point2f> edgePoints;
        edgePoints.reserve(180);
        const float innerRadius = initialRadius * 0.70f;
        const float outerRadius = initialRadius * 1.30f;
        const float innerRadius2 = innerRadius * innerRadius;
        const float outerRadius2 = outerRadius * outerRadius;

        for (int y = area.y; y < area.y + area.height; ++y) {
            const uchar* edgeRow = edgeMap_.ptr<uchar>(y);
            const float* magnitudeRow = gradient_.ptr<float>(y);
            const float* gradientXRow = gradientX_.ptr<float>(y);
            const float* gradientYRow = gradientY_.ptr<float>(y);
            for (int x = area.x; x < area.x + area.width; ++x) {
                if (!edgeRow[x] ||
                    magnitudeRow[x] < BALL_CFG_REFINE_GRADIENT_MIN) {
                    continue;
                }

                const float dx = x + 0.5f - initialCenter.x;
                const float dy = y + 0.5f - initialCenter.y;
                const float distance2 = dx * dx + dy * dy;
                if (distance2 < innerRadius2 || distance2 > outerRadius2) {
                    continue;
                }

                // 真圆边缘的梯度方向应大致沿半径方向。绝对值同时允许亮到暗和暗到亮。
                const float distance = std::sqrt(distance2);
                const float magnitude = magnitudeRow[x];
                const float alignment = std::abs(
                    (gradientXRow[x] * dx + gradientYRow[x] * dy) /
                    std::max(1.0f, magnitude * distance));
                if (alignment < 0.52f) continue;
                edgePoints.emplace_back(x + 0.5f, y + 0.5f);
            }
        }

        cv::Point2f firstCenter;
        float firstRadius = 0.0f;
        if (!fitCircleLeastSquares(edgePoints, initialCenter,
                                   firstCenter, firstRadius)) {
            return result;
        }
        if (cv::norm(firstCenter - initialCenter) >
                std::max(4.0f, initialRadius * 0.38f) ||
            firstRadius < BALL_RADIUS_MIN * 0.85f ||
            firstRadius > BALL_RADIUS_MAX * 1.15f) {
            return result;
        }

        // 第一次拟合后剔除偏离圆周较远的管壁和刻度边缘，再拟合一次。
        std::vector<cv::Point2f> inliers;
        inliers.reserve(edgePoints.size());
        for (const cv::Point2f& point : edgePoints) {
            const float residual = std::abs(
                static_cast<float>(cv::norm(point - firstCenter)) -
                firstRadius);
            if (residual <= BALL_CFG_REFINE_RESIDUAL_MAX) {
                inliers.push_back(point);
            }
        }

        cv::Point2f finalCenter = firstCenter;
        float finalRadius = firstRadius;
        cv::Point2f secondCenter;
        float secondRadius = 0.0f;
        if (fitCircleLeastSquares(inliers, firstCenter,
                                  secondCenter, secondRadius) &&
            cv::norm(secondCenter - initialCenter) <=
                std::max(4.0f, initialRadius * 0.38f) &&
            secondRadius >= BALL_RADIUS_MIN * 0.85f &&
            secondRadius <= BALL_RADIUS_MAX * 1.15f) {
            finalCenter = secondCenter;
            finalRadius = secondRadius;
        }

        double residualSum = 0.0;
        int residualCount = 0;
        for (const cv::Point2f& point : inliers) {
            residualSum += std::abs(
                cv::norm(point - finalCenter) - finalRadius);
            ++residualCount;
        }
        if (residualCount < 12) return result;

        const float meanResidual = static_cast<float>(
            residualSum / residualCount);
        const float inlierFraction = static_cast<float>(inliers.size()) /
                                     std::max<std::size_t>(1, edgePoints.size());
        const float residualQuality = std::exp(-meanResidual / 2.2f);
        result.center = finalCenter;
        result.radius = finalRadius;
        result.quality = unitScore(
            0.72f * residualQuality + 0.28f * inlierFraction);
        return result;
    }

    Candidate scoreCircle(const RefinedCircle& circle) const
    {
        Candidate candidate;
        const cv::Point2f center = circle.center;
        const float radius = circle.radius;
        if (radius < BALL_RADIUS_MIN || radius > BALL_RADIUS_MAX) {
            return candidate;
        }

        const float axisDistance = distanceToAxis(center);
        if (axisGateEnabled_ && axisDistance > AXIS_GATE_PX) {
            return candidate;
        }

        const int margin = cvCeil(radius * 1.62f) + 2;
        const cv::Rect area = boundedVisionRect(
            cv::Rect(cvFloor(center.x) - margin,
                     cvFloor(center.y) - margin,
                     margin * 2 + 1, margin * 2 + 1),
            gray_.size());
        if (area.width < 12 || area.height < 12) return candidate;

        const float innerRadius2 = radius * radius * 0.72f * 0.72f;
        const float ringInner2 = radius * radius * 1.08f * 1.08f;
        const float ringOuter2 = radius * radius * 1.52f * 1.52f;
        double innerSum = 0.0;
        double innerSquareSum = 0.0;
        double ringSum = 0.0;
        int innerCount = 0;
        int ringCount = 0;

        for (int y = area.y; y < area.y + area.height; ++y) {
            const uchar* row = gray_.ptr<uchar>(y);
            for (int x = area.x; x < area.x + area.width; ++x) {
                const float dx = x + 0.5f - center.x;
                const float dy = y + 0.5f - center.y;
                const float distance2 = dx * dx + dy * dy;
                if (distance2 <= innerRadius2) {
                    const float value = row[x];
                    innerSum += value;
                    innerSquareSum += value * value;
                    ++innerCount;
                } else if (distance2 >= ringInner2 &&
                           distance2 <= ringOuter2) {
                    ringSum += row[x];
                    ++ringCount;
                }
            }
        }
        if (innerCount < 25 || ringCount < 35) return candidate;

        const float innerMean = static_cast<float>(innerSum / innerCount);
        const float ringMean = static_cast<float>(ringSum / ringCount);
        const float variance = std::max(
            0.0f,
            static_cast<float>(innerSquareSum / innerCount -
                               innerMean * innerMean));
        const float innerStd = std::sqrt(variance);
        const float contrast = ringMean - innerMean;

        int brightCount = 0;
        int darkCount = 0;
        for (int y = area.y; y < area.y + area.height; ++y) {
            const uchar* row = gray_.ptr<uchar>(y);
            for (int x = area.x; x < area.x + area.width; ++x) {
                const float dx = x + 0.5f - center.x;
                const float dy = y + 0.5f - center.y;
                if (dx * dx + dy * dy > innerRadius2) continue;
                darkCount += row[x] < innerMean - 0.55f * innerStd;
                brightCount += row[x] > innerMean + 0.55f * innerStd;
            }
        }

        // 48个方向统计圆周是否存在真实梯度，防止只靠局部弧线组成假圆。
        constexpr int EDGE_SAMPLES = 48;
        int supportedEdges = 0;
        for (int index = 0; index < EDGE_SAMPLES; ++index) {
            const float angle = static_cast<float>(
                2.0 * CV_PI * index / EDGE_SAMPLES);
            float strongest = 0.0f;
            for (int radial = -2; radial <= 2; ++radial) {
                const float sampleRadius = radius + radial;
                const int x = cvRound(center.x +
                                      std::cos(angle) * sampleRadius);
                const int y = cvRound(center.y +
                                      std::sin(angle) * sampleRadius);
                if (x >= 0 && x < gradient_.cols &&
                    y >= 0 && y < gradient_.rows) {
                    strongest = std::max(
                        strongest, gradient_.at<float>(y, x));
                }
            }
            supportedEdges += strongest >= BALL_CFG_REFINE_GRADIENT_MIN;
        }

        const float edgeSupport =
            static_cast<float>(supportedEdges) / EDGE_SAMPLES;
        const float brightFraction =
            static_cast<float>(brightCount) / innerCount;
        const float darkFraction =
            static_cast<float>(darkCount) / innerCount;

        const float contrastScore = unitScore((contrast - 1.0f) / 34.0f);
        const float edgeScore = unitScore((edgeSupport - 0.24f) / 0.66f);
        const float textureScore = unitScore((innerStd - 4.0f) / 30.0f);
        const float reflectionScore = unitScore(
            (std::min(brightFraction, darkFraction) - 0.025f) / 0.18f);
        const float radiusScore = std::exp(
            -std::abs(radius - BALL_RADIUS_EXPECTED) /
                std::max(2.0f, BALL_RADIUS_EXPECTED * 0.24f));
        const float axisScore = axisGateEnabled_ ?
            std::exp(-axisDistance /
                     std::max(2.0f, AXIS_GATE_PX * 0.48f)) : 1.0f;

        // 霍夫候选仍必须通过外观评分。圆周边缘和亮度差权重最高，
        // 拟合质量用于压制仅由几段杂乱边缘拼成的假圆。
        const float score =
            0.27f * contrastScore +
            0.27f * edgeScore +
            0.12f * textureScore +
            0.10f * reflectionScore +
            0.10f * radiusScore +
            0.09f * circle.quality +
            0.05f * axisScore;
        if (score < MIN_DETECTION_SCORE) return candidate;

        candidate.valid = true;
        candidate.center = center;
        candidate.radius = radius;
        candidate.score = score;
        candidate.contrast = contrast;
        candidate.edgeSupport = edgeSupport;
        candidate.ringMean = ringMean;
        candidate.innerStd = innerStd;
        candidate.fitQuality = circle.quality;
        candidate.axisDistance = axisDistance;
        candidate.houghOnly = true;
        return candidate;
    }

    // ------------------------------------------------------------------------
    // 暗斑轮廓定位
    // ------------------------------------------------------------------------
    // 霍夫圆要求边缘接近完整圆形；钢球快速运动时，曝光造成的拖影会把
    // 圆变成短椭圆，甚至只剩一个连续的暗色斑块。此函数只在当前search
    // ROI中做自适应二值化和轮廓筛选，不扫描整幅图，也不创建整帧副本。
    //
    // 重要安全顺序：
    // 1. 先用面积、宽高、长宽比、圆度和实心度排除管壁长条及噪声；
    // 2. 再用轴线和（跟踪态）预测位置门限排除管外固定物；
    // 3. 最后计算局部暗对比、边缘支持和时序分数，才生成Candidate。
    // 捕获态的形状门限比跟踪态严格，且后面的update()仍要求连续多帧。
    std::vector<Candidate> detectDarkBlobCandidates(
        const cv::Rect& search,
        const cv::Point2f& expected,
        bool tracking,
        float trackingGate)
    {
        std::vector<Candidate> candidates;
        if (search.width < DARK_BLOB_WIDTH_MIN ||
            search.height < DARK_BLOB_HEIGHT_MIN) {
            return candidates;
        }

        const cv::Mat smoothSearch = smooth_(search);
        darkMask_.create(search.size(), CV_8U);

        // 自适应阈值使用局部均值，不依赖摄像头当前的绝对曝光值。
        // 实机截图测试中49、C=8可完整分出约25x21像素的钢球暗斑。
        // blockSize必须为奇数；第3题ROI高50像素，49仍然只处理很小区域。
        cv::adaptiveThreshold(
            smoothSearch, darkMask_, 255,
            cv::ADAPTIVE_THRESH_GAUSSIAN_C,
            cv::THRESH_BINARY_INV, 49, 8.0);

        // 小开运算去掉椒盐噪声，小闭运算填补运动拖影中的窄裂缝。
        // 内核只创建一次，后续帧复用，避免循环内反复分配。
        if (darkKernel_.empty()) {
            darkKernel_ = cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(3, 3));
        }
        cv::morphologyEx(darkMask_, darkMask_, cv::MORPH_OPEN,
                         darkKernel_);
        cv::morphologyEx(darkMask_, darkMask_, cv::MORPH_CLOSE,
                         darkKernel_);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(darkMask_, contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);
        candidates.reserve(contours.size());

        const float minimumAspect = tracking ?
            DARK_BLOB_ASPECT_MIN_TRACK : DARK_BLOB_ASPECT_MIN_ACQUIRE;
        const float minimumCircularity = tracking ?
            DARK_BLOB_CIRCULARITY_MIN_TRACK :
            DARK_BLOB_CIRCULARITY_MIN_ACQUIRE;
        const float minimumSolidity = tracking ?
            DARK_BLOB_SOLIDITY_MIN_TRACK : DARK_BLOB_SOLIDITY_MIN_ACQUIRE;

        for (const std::vector<cv::Point>& contour : contours) {
            const double area = cv::contourArea(contour);
            if (area < DARK_BLOB_AREA_MIN ||
                area > DARK_BLOB_AREA_MAX || contour.size() < 5) {
                continue;
            }

            const cv::Rect bounds = cv::boundingRect(contour);
            if (bounds.width < DARK_BLOB_WIDTH_MIN ||
                bounds.width > DARK_BLOB_WIDTH_MAX ||
                bounds.height < DARK_BLOB_HEIGHT_MIN ||
                bounds.height > DARK_BLOB_HEIGHT_MAX) {
                continue;
            }

            // 轮廓接触搜索窗边界时通常是管壁/阴影被截断，不能当球。
            if (bounds.x <= 1 || bounds.y <= 1 ||
                bounds.x + bounds.width >= search.width - 1 ||
                bounds.y + bounds.height >= search.height - 1) {
                continue;
            }

            const float shortSide = static_cast<float>(
                std::min(bounds.width, bounds.height));
            const float longSide = static_cast<float>(
                std::max(bounds.width, bounds.height));
            const float aspect = shortSide / std::max(1.0f, longSide);
            if (aspect < minimumAspect) continue;

            const double perimeter = cv::arcLength(contour, true);
            if (perimeter <= 1.0) continue;
            const float circularity = static_cast<float>(
                4.0 * CV_PI * area / (perimeter * perimeter));
            if (circularity < minimumCircularity) continue;

            std::vector<cv::Point> hull;
            cv::convexHull(contour, hull);
            const double hullArea = cv::contourArea(hull);
            const float solidity = hullArea > 1.0 ? static_cast<float>(
                area / hullArea) : 0.0f;
            if (solidity < minimumSolidity) continue;

            const cv::Moments moments = cv::moments(contour, false);
            if (std::abs(moments.m00) < 1e-6) continue;
            const cv::Point2f center(
                static_cast<float>(search.x + moments.m10 / moments.m00),
                static_cast<float>(search.y + moments.m01 / moments.m00));

            const float axisDistance = distanceToAxis(center);
            if (axisGateEnabled_ && axisDistance > AXIS_GATE_PX) {
                continue;
            }

            if (tracking && cv::norm(center - expected) > trackingGate) {
                // 搜索窗在丢球后可能暂时扩展到整个ROI；即使如此，暗斑
                // 候选也必须接近运动模型预测点，不能跳到远处固定物。
                continue;
            }

            const float observedRadius =
                0.25f * (bounds.width + bounds.height);
            if (observedRadius < BALL_RADIUS_MIN * 0.72f ||
                observedRadius > BALL_RADIUS_MAX * 1.35f) {
                continue;
            }
            // 二值暗斑有时只覆盖钢球的暗色核心，直接把包围盒半径写入
            // 跟踪器会偏小。向实测期望半径轻微收缩，可让下一帧恢复的
            // 霍夫圆顺利通过半径连续性门限，同时仍保留拖影尺寸变化。
            const float radius =
                0.60f * observedRadius +
                0.40f * BALL_RADIUS_EXPECTED;

            // 用轮廓包围盒定义一个椭圆坐标系，统计暗斑内部和外环的
            // 灰度。这样横向拖影不会因圆形采样落空而被误拒。
            const float halfWidth = std::max(2.0f, bounds.width * 0.5f);
            const float halfHeight = std::max(2.0f, bounds.height * 0.5f);
            const int sampleMargin = cvCeil(
                std::max(halfWidth, halfHeight) * 1.75f) + 2;
            const cv::Rect sampleArea = boundedVisionRect(
                cv::Rect(cvFloor(center.x) - sampleMargin,
                         cvFloor(center.y) - sampleMargin,
                         sampleMargin * 2 + 1,
                         sampleMargin * 2 + 1),
                gray_.size()) & search;
            if (sampleArea.width < 8 || sampleArea.height < 8) continue;

            double innerSum = 0.0;
            double innerSquareSum = 0.0;
            double ringSum = 0.0;
            int innerCount = 0;
            int ringCount = 0;
            for (int y = sampleArea.y;
                 y < sampleArea.y + sampleArea.height; ++y) {
                const uchar* row = gray_.ptr<uchar>(y);
                for (int x = sampleArea.x;
                     x < sampleArea.x + sampleArea.width; ++x) {
                    const float nx = (x + 0.5f - center.x) / halfWidth;
                    const float ny = (y + 0.5f - center.y) / halfHeight;
                    const float normalizedDistance2 = nx * nx + ny * ny;
                    if (normalizedDistance2 <= 0.55f * 0.55f) {
                        const float value = row[x];
                        innerSum += value;
                        innerSquareSum += value * value;
                        ++innerCount;
                    } else if (normalizedDistance2 >= 1.15f * 1.15f &&
                               normalizedDistance2 <= 1.65f * 1.65f) {
                        ringSum += row[x];
                        ++ringCount;
                    }
                }
            }
            if (innerCount < 20 || ringCount < 30) continue;

            const float innerMean = static_cast<float>(
                innerSum / innerCount);
            const float ringMean = static_cast<float>(ringSum / ringCount);
            const float variance = std::max(
                0.0f, static_cast<float>(innerSquareSum / innerCount -
                                          innerMean * innerMean));
            const float innerStd = std::sqrt(variance);
            const float contrast = ringMean - innerMean;
            if (!std::isfinite(contrast) || !std::isfinite(innerStd)) {
                continue;
            }

            // 轮廓点中有多少点确实落在梯度边缘上。边缘支持用于和现有
            // Hough候选统一外观门限，不把纯黑色矩形直接当作钢球。
            int boundarySamples = 0;
            int supportedBoundary = 0;
            const std::size_t contourStep = std::max<std::size_t>(
                1, contour.size() / 80);
            for (std::size_t index = 0; index < contour.size();
                 index += contourStep) {
                const cv::Point& localPoint = contour[index];
                const int x = search.x + localPoint.x;
                const int y = search.y + localPoint.y;
                if (x < 0 || x >= gradient_.cols ||
                    y < 0 || y >= gradient_.rows) {
                    continue;
                }
                ++boundarySamples;
                supportedBoundary +=
                    gradient_.at<float>(y, x) >=
                    BALL_CFG_REFINE_GRADIENT_MIN;
            }
            if (boundarySamples < 5) continue;
            const float edgeSupport = static_cast<float>(
                supportedBoundary) / boundarySamples;

            const float temporalScore = tracking ?
                std::exp(-static_cast<float>(cv::norm(center - expected)) /
                         std::max(12.0f, BALL_RADIUS_EXPECTED * 1.8f)) :
                (pending_.valid ?
                    std::exp(-static_cast<float>(cv::norm(
                        center - pending_.center)) /
                             std::max(12.0f, BALL_RADIUS_EXPECTED * 1.8f)) :
                    1.0f);
            const float radiusScore = std::exp(
                -std::abs(radius - BALL_RADIUS_EXPECTED) /
                std::max(2.0f, BALL_RADIUS_EXPECTED * 0.30f));
            const float contrastScore = unitScore(
                (contrast - 3.0f) / 30.0f);
            const float textureScore = unitScore(
                (innerStd - 2.0f) / 24.0f);
            const float shapeScore =
                0.34f * unitScore((circularity - minimumCircularity) /
                                   std::max(0.05f, 1.0f - minimumCircularity)) +
                0.26f * unitScore((aspect - minimumAspect) /
                                   std::max(0.05f, 1.0f - minimumAspect)) +
                0.24f * unitScore((solidity - minimumSolidity) /
                                   std::max(0.05f, 1.0f - minimumSolidity)) +
                0.16f * unitScore(edgeSupport);
            const float axisScore = axisGateEnabled_ ?
                std::exp(-axisDistance /
                         std::max(2.0f, AXIS_GATE_PX * 0.48f)) : 1.0f;

            const float score =
                0.28f * contrastScore +
                0.20f * shapeScore +
                0.13f * radiusScore +
                0.11f * textureScore +
                0.12f * unitScore(edgeSupport) +
                0.10f * temporalScore +
                0.06f * axisScore;
            if (score < MIN_DETECTION_SCORE) continue;

            Candidate candidate;
            candidate.valid = true;
            candidate.center = center;
            candidate.radius = radius;
            candidate.score = score;
            candidate.contrast = contrast;
            candidate.edgeSupport = edgeSupport;
            candidate.ringMean = ringMean;
            candidate.innerStd = innerStd;
            candidate.fitQuality = shapeScore;
            candidate.axisDistance = axisDistance;
            candidate.contourFallback = true;
            addCandidate(candidates, candidate);
        }
        return candidates;
    }

    static bool sameCircle(const Candidate& first,
                           const Candidate& second)
    {
        return cv::norm(first.center - second.center) <
               std::max(6.0f,
                        0.35f * (first.radius + second.radius));
    }

    static void addCandidate(std::vector<Candidate>& candidates,
                             const Candidate& candidate)
    {
        if (!candidate.valid) return;
        for (Candidate& existing : candidates) {
            if (sameCircle(existing, candidate)) {
                if (candidate.score > existing.score) existing = candidate;
                return;
            }
        }
        candidates.push_back(candidate);
    }

    Candidate chooseBestCandidate(std::vector<Candidate>& candidates,
                                  const cv::Point2f& expected,
                                  bool tracking,
                                  float trackingGate) const
    {
        // 跟踪态必须先通过预测距离硬门限。捕获态若上一帧已有pending，
        // 也只保留与pending在位置和半径上连续的候选；这使多个固定暗斑
        // 不能在帧间轮流成为第一名并凑够锁定帧数。
        candidates.erase(
            std::remove_if(
                candidates.begin(), candidates.end(),
                [&](const Candidate& candidate) {
                    if (!candidate.valid) return true;
                    if (tracking) {
                        return cv::norm(candidate.center - expected) >
                               trackingGate;
                    }
                    if (!pending_.valid) {
                        if (!acquireAnchorConfigured_) return false;
                        const float firstGate = hasTrackHistory_ ?
                            TRACK_GATE_MAX_PX : INITIAL_ACQUIRE_GATE_PX;
                        return cv::norm(
                            candidate.center - acquireAnchor_) > firstGate;
                    }
                    return cv::norm(candidate.center - pending_.center) >
                               ACQUIRE_GATE_PX ||
                           std::abs(candidate.radius - pending_.radius) >
                               std::max(2.5f,
                                        BALL_RADIUS_EXPECTED * 0.30f);
                }),
            candidates.end());
        if (candidates.empty()) return Candidate{};

        for (Candidate& candidate : candidates) {
            const cv::Point2f reference = tracking ?
                expected :
                (pending_.valid ? pending_.center :
                 (acquireAnchorConfigured_ ? acquireAnchor_ : expected));
            const float distance = static_cast<float>(
                cv::norm(candidate.center - reference));
            const float temporalScore = std::exp(
                -distance /
                std::max(12.0f, BALL_RADIUS_EXPECTED * 1.8f));

            if (tracking) {
                const float radiusContinuity = std::exp(
                    -std::abs(candidate.radius - radius_) /
                    std::max(2.0f, BALL_RADIUS_EXPECTED * 0.25f));
                // 锁定后预测位置权重较高，防止运动模糊时跳向附近螺丝。
                candidate.score =
                    0.68f * candidate.score +
                    0.25f * temporalScore +
                    0.07f * radiusContinuity;
            } else if (pending_.valid) {
                const float radiusContinuity = std::exp(
                    -std::abs(candidate.radius - pending_.radius) /
                    std::max(2.0f, BALL_RADIUS_EXPECTED * 0.25f));
                // 尚未锁定时也利用上一帧pending，避免每帧在不同固定物间跳转。
                candidate.score =
                    0.80f * candidate.score +
                    0.16f * temporalScore +
                    0.04f * radiusContinuity;
            }
        }

        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& first, const Candidate& second) {
            return first.score > second.score;
        });

        const Candidate best = candidates.front();
        if (tracking && candidates.size() > 1) {
            const Candidate& second = candidates[1];
            const bool separateObjects =
                cv::norm(best.center - second.center) >
                std::max(7.0f,
                         0.35f * (best.radius + second.radius));
            const bool ambiguous =
                second.score > best.score - AMBIGUITY_GAP;
            if (separateObjects && ambiguous) {
                // 两个远距离目标同样可信时宁可漏一帧，不随机切换目标。
                return Candidate{};
            }
        }
        return best;
    }

    void prepareFrame(const cv::Mat& frame, const cv::Rect& detectionArea)
    {
        // 灰度、CLAHE、模糊、Sobel和Canny都只处理水管ROI及钢球半径
        // 安全边界。各缓存仍保留整帧坐标系，后面的圆心和轴线无需反复
        // 做局部/全局坐标转换，但真正参与计算的像素只有processingArea。
        const int margin = cvCeil(BALL_RADIUS_MAX * 2.0f) + 4;
        const cv::Rect processingArea = boundedVisionRect(
            cv::Rect(detectionArea.x - margin,
                     detectionArea.y - margin,
                     detectionArea.width + margin * 2,
                     detectionArea.height + margin * 2),
            frame.size());
        if (processingArea.width <= 0 || processingArea.height <= 0) return;

        gray_.create(frame.size(), CV_8U);
        enhanced_.create(frame.size(), CV_8U);
        smooth_.create(frame.size(), CV_8U);
        gradientX_.create(frame.size(), CV_32F);
        gradientY_.create(frame.size(), CV_32F);
        gradient_.create(frame.size(), CV_32F);
        edgeMap_.create(frame.size(), CV_8U);

        cv::Mat grayArea = gray_(processingArea);
        cv::Mat enhancedArea = enhanced_(processingArea);
        cv::Mat smoothArea = smooth_(processingArea);
        cv::Mat gradientXArea = gradientX_(processingArea);
        cv::Mat gradientYArea = gradientY_(processingArea);
        cv::Mat gradientArea = gradient_(processingArea);
        cv::Mat edgeArea = edgeMap_(processingArea);
        cv::cvtColor(frame(processingArea), grayArea,
                     cv::COLOR_BGR2GRAY);
#if BALL_CFG_GRAY_BRIGHTNESS != 0
        {
            // 数字提亮只改变算法输入，不延长摄像头曝光，因此不降低采集帧率。
            cv::add(grayArea, cv::Scalar(BALL_CFG_GRAY_BRIGHTNESS), grayArea);
        }
#endif

        clahe_->apply(grayArea, enhancedArea);
        cv::GaussianBlur(enhancedArea, smoothArea, cv::Size(5, 5), 1.05);
        cv::Sobel(smoothArea, gradientXArea, CV_32F, 1, 0, 3);
        cv::Sobel(smoothArea, gradientYArea, CV_32F, 0, 1, 3);
        cv::magnitude(gradientXArea, gradientYArea, gradientArea);
        cv::Canny(smoothArea, edgeArea,
                  BALL_CFG_HOUGH_CANNY_HIGH * 0.45,
                  BALL_CFG_HOUGH_CANNY_HIGH, 3, true);
    }

    Candidate detect(const cv::Mat& frame, const cv::Rect& search,
                     const cv::Point2f& expected, bool tracking,
                     float trackingGate)
    {
        lastHoughCandidates_ = 0;
        lastValidCandidates_ = 0;
        lastDarkBlobCandidates_ = 0;
        if (frame.empty() ||
            search.width < cvRound(BALL_RADIUS_MAX * 2.0f) ||
            search.height < cvRound(BALL_RADIUS_MAX * 2.0f)) {
            return Candidate{};
        }

        ++frameCounter_;
        // 锁定后只处理预测搜索窗及半径安全边界，不再每帧对整段535 px水管
        // 做CLAHE、Sobel和Canny；失锁/首次捕获时search才等于完整ROI。
        prepareFrame(frame, search);

        // 暗斑轮廓是每帧快速定位主通道。它对短拖影比完整霍夫圆更稳定。
        std::vector<Candidate> blobCandidates =
            detectDarkBlobCandidates(search, expected, tracking,
                                     trackingGate);
        lastDarkBlobCandidates_ =
            static_cast<int>(blobCandidates.size());
        lastValidCandidates_ += static_cast<int>(blobCandidates.size());
        Candidate blobBest = chooseBestCandidate(
            blobCandidates, expected, tracking, trackingGate);

        // 霍夫只在首次捕获、定期复核或暗斑漏检时运行。锁定且暗斑正常时
        // 每HOUGH_INTERVAL帧做一次几何确认，大幅减少树莓派CPU负担。
        const bool periodicHough = tracking ?
            (frameCounter_ % static_cast<uint64_t>(HOUGH_INTERVAL) == 0) :
            (frameCounter_ % 2U == 0U);
        const bool runHough =
            !blobBest.valid || misses_ > 0 || periodicHough;
        if (!runHough) return blobBest;

        const cv::Mat houghInput = smooth_(search);
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(
            houghInput, circles, cv::HOUGH_GRADIENT,
            BALL_CFG_HOUGH_DP,
            std::max(8.0f, BALL_RADIUS_EXPECTED * 1.45f),
            BALL_CFG_HOUGH_CANNY_HIGH,
            tracking ? BALL_CFG_HOUGH_ACCUM_TRACK
                     : BALL_CFG_HOUGH_ACCUM_ACQUIRE,
            cvRound(BALL_RADIUS_MIN),
            cvRound(BALL_RADIUS_MAX));
        lastHoughCandidates_ = static_cast<int>(circles.size());

        std::vector<Candidate> candidates;
        // 霍夫结果通常已按票数排序。只精修前几个候选，避免宽松参数下
        // 对十几个假圆逐个做两次SVD拟合拖垮120 FPS。
        const std::size_t count = std::min(
            circles.size(), static_cast<std::size_t>(HOUGH_MAX_CANDIDATES));
        candidates.reserve(count);
        for (std::size_t index = 0; index < count; ++index) {
            const cv::Vec3f& circle = circles[index];
            const cv::Point2f initialCenter(
                circle[0] + search.x,
                circle[1] + search.y);
            const RefinedCircle refined =
                refineCircle(initialCenter, circle[2]);
            addCandidate(candidates, scoreCircle(refined));
        }
        lastValidCandidates_ += static_cast<int>(candidates.size());
        Candidate houghBest = chooseBestCandidate(
            candidates, expected, tracking, trackingGate);

        if (!houghBest.valid) return blobBest;
        if (!blobBest.valid) return houghBest;

        const float centerDifference = static_cast<float>(
            cv::norm(houghBest.center - blobBest.center));
        if (centerDifference <= HOUGH_BLOB_AGREEMENT_PX) {
            // 两种方法确认的是同一物体：暗斑质心比不完整圆弧拟合稳定，
            // 霍夫只用于验证并辅助半径，不能再覆盖圆心。
            Candidate fused = blobBest;
            fused.radius =
                0.72f * blobBest.radius + 0.28f * houghBest.radius;
            fused.score = unitScore(
                std::max(blobBest.score, houghBest.score) + 0.04f);
            fused.fitQuality = std::max(
                blobBest.fitQuality, houghBest.fitQuality);
            fused.contourFallback = false;
            fused.houghOnly = false;
            fused.fused = true;
            return fused;
        }

        // 两种方法指向不同物体时不能固定偏信其中一种。只有暗斑明显更靠近
        // 运动预测点时才保留暗斑；其余情况宁可漏一帧，绝不让轨迹跳到管壁。
        const float blobDistance = static_cast<float>(
            cv::norm(blobBest.center - expected));
        const float houghDistance = static_cast<float>(
            cv::norm(houghBest.center - expected));
        const float arbitrationGate = tracking ? trackingGate :
            (hasTrackHistory_ ? TRACK_GATE_MAX_PX :
                                INITIAL_ACQUIRE_GATE_PX);
        // 锁定态暗斑本身已经通过预测门限、轴线和外观检查。若它仍紧贴
        // 预测点而霍夫圆偏到阴影，优先保留暗斑，避免每8帧人为制造一次漏检。
        if (tracking && blobDistance <= trackingGate * 0.45f &&
            blobBest.score + 0.02f >= houghBest.score) {
            return blobBest;
        }
        if (blobDistance + 3.0f < houghDistance &&
            blobDistance <= arbitrationGate * 0.75f) {
            return blobBest;
        }
        if (tracking && misses_ >= 2 &&
            houghDistance + 3.0f < blobDistance &&
            houghDistance <= arbitrationGate * 0.75f) {
            return houghBest;
        }
        return Candidate{};
    }

public:
    explicit SteelBallDetector(
        const cv::Rect& roi = {},
        const cv::Point2f& axisStart = {},
        const cv::Point2f& axisEnd = {},
        bool axisConfigured = false,
        const cv::Point2f& initialAcquireAnchor = {},
        bool initialAcquireAnchorConfigured = false)
        : configuredRoi_(roi),
          axisStart_(axisStart),
          axisEnd_(axisEnd),
          axisGateEnabled_(
              BALL_CFG_USE_AXIS_GATE != 0 &&
              axisConfigured &&
              cv::norm(axisEnd - axisStart) >= 30.0f),
          acquireAnchor_(initialAcquireAnchor),
          acquireAnchorConfigured_(initialAcquireAnchorConfigured),
          clahe_(cv::createCLAHE(BALL_CFG_CLAHE_CLIP_LIMIT,
                                 cv::Size(8, 8)))
    {
    }

    void reset(const cv::Point2f& acquireAnchor,
               bool anchorConfigured = true)
    {
        locked_ = false;
        hasTrackHistory_ = false;
        misses_ = 0;
        acquireCount_ = 0;
        lastFrameUs_ = 0;
        center_ = acquireAnchor;
        velocity_ = {};
        radius_ = BALL_RADIUS_EXPECTED;
        confidence_ = 0.0f;
        pending_ = {};
        acquireAnchor_ = acquireAnchor;
        acquireAnchorConfigured_ = anchorConfigured;
    }

    Candidate detectImage(const cv::Mat& frame)
    {
        const cv::Rect search = baseRoi(frame.size());
        return detect(
            frame, search,
            cv::Point2f(search.x + search.width * 0.5f,
                        search.y + search.height * 0.5f),
            false, ACQUIRE_GATE_PX);
    }

    Result update(const cv::Mat& frame)
    {
        const int64_t timestampUs = visionNowMicroseconds();
        const float dt = lastFrameUs_ ?
            clampVision(
                static_cast<float>(timestampUs - lastFrameUs_) / 1000000.0f,
                0.0025f, 0.050f) :
            1.0f / CAMERA_FPS;
        lastFrameUs_ = timestampUs;

        const cv::Rect allowed = baseRoi(frame.size());
        cv::Point2f predicted = center_ + velocity_ * dt;
        cv::Rect search = allowed;
        float trackingGate = ACQUIRE_GATE_PX;

        if (locked_) {
            trackingGate = clampVision(
                TRACK_GATE_MIN_PX +
                    static_cast<float>(cv::norm(velocity_)) * dt * 1.15f +
                    300.0f * dt + misses_ * 14.0f,
                TRACK_GATE_MIN_PX,
                TRACK_GATE_MAX_PX);
            const float objectHalfWidth = std::max(
                BALL_RADIUS_MAX + 4.0f,
                DARK_BLOB_WIDTH_MAX * 0.5f + 4.0f);
            const int halfSize = cvCeil(
                trackingGate + objectHalfWidth);
            search = boundedVisionRect(
                cv::Rect(cvRound(predicted.x) - halfSize,
                         cvRound(predicted.y) - halfSize,
                         halfSize * 2 + 1,
                         halfSize * 2 + 1),
                frame.size()) & allowed;

            if (misses_ >= 5) {
                // 运动模糊连续丢几帧后，预测圆心可能已落后真实钢球。
                // 暂时扩大到整个第3题ROI；候选仍受逐步扩大的预测门限、
                // 真实尺寸、轴线、外观和半径连续性共同约束。
                trackingGate = TRACK_GATE_MAX_PX;
                search = allowed;
            }
        } else {
            predicted = acquireAnchorConfigured_ ?
                acquireAnchor_ :
                cv::Point2f(
                    allowed.x + allowed.width * 0.5f,
                    allowed.y + allowed.height * 0.5f);
        }

        Candidate candidate = detect(
            frame, search, predicted, locked_, trackingGate);
        if (locked_ && candidate.valid &&
            (cv::norm(candidate.center - predicted) > trackingGate ||
             candidate.radius < radius_ * 0.78f ||
             candidate.radius > radius_ * 1.28f)) {
            // 锁定后半径限制比原版更严格，防止突然切换到另一种尺寸的圆。
            candidate = Candidate{};
        }

        if (locked_ && candidate.valid && candidate.houghOnly &&
            misses_ < 2) {
            // 暗斑短暂漏一两帧时先沿用预测位置，不立即切换到未经验证的霍夫圆。
            // 连续两帧都没有暗斑后才允许霍夫备用，兼顾运动模糊时不断轨。
            candidate = Candidate{};
        }

        Result result;
        if (locked_) {
            if (candidate.valid) {
                const cv::Point2f residual = candidate.center - predicted;
                // 候选已经通过尺寸、轴线、预测距离和双检测器仲裁；适当提高
                // 位置吸收比例以减少高速运动时的跟踪滞后。
                center_ = predicted + residual * 0.72f;
                velocity_ += residual * (0.08f / dt);
                const float speed = static_cast<float>(cv::norm(velocity_));
                if (speed > 3500.0f) {
                    velocity_ *= 3500.0f / speed;
                }
                radius_ = 0.78f * radius_ + 0.22f * candidate.radius;
                confidence_ =
                    0.62f * confidence_ + 0.38f * candidate.score;
                misses_ = 0;
                acquireAnchor_ = center_;
                acquireAnchorConfigured_ = true;
                hasTrackHistory_ = true;
                result.measured = true;
            } else {
                // 漏检帧不输出预测圆心给控制器，只用它维持下一帧的局部搜索窗。
                center_ = predicted;
                velocity_ *= 0.80f;
                confidence_ *= 0.72f;
                if (++misses_ > MAX_MISSES) {
                    locked_ = false;
                    acquireCount_ = 0;
                    pending_ = {};
                    acquireAnchor_ = center_;
                    acquireAnchorConfigured_ = true;
                    velocity_ = {};
                    confidence_ = 0.0f;
                }
            }
        } else if (candidate.valid &&
                   candidate.score >= MIN_ACQUIRE_SCORE &&
                   candidate.contrast >= ACQUIRE_CONTRAST_MIN &&
                   candidate.edgeSupport >= ACQUIRE_EDGE_SUPPORT_MIN &&
                   candidate.ringMean >= ACQUIRE_RING_MEAN_MIN &&
                   candidate.innerStd >= ACQUIRE_INNER_STD_MIN) {
            if (pending_.valid &&
                cv::norm(candidate.center - pending_.center) <=
                    ACQUIRE_GATE_PX &&
                std::abs(candidate.radius - pending_.radius) <=
                    std::max(2.5f, BALL_RADIUS_EXPECTED * 0.30f)) {
                pending_.center =
                    pending_.center * 0.35f + candidate.center * 0.65f;
                pending_.radius =
                    pending_.radius * 0.45f + candidate.radius * 0.55f;
                pending_.score = std::max(
                    pending_.score, candidate.score);
                // 只要连续确认期间至少有一帧被霍夫圆确认，就按普通
                // ACQUIRE_FRAMES锁定；全部依赖轮廓后备时则额外确认两帧。
                pending_.contourFallback =
                    pending_.contourFallback &&
                    candidate.contourFallback;
                pending_.houghOnly =
                    pending_.houghOnly && candidate.houghOnly;
                pending_.fused = pending_.fused || candidate.fused;
                ++acquireCount_;
            } else {
                pending_ = candidate;
                acquireCount_ = 1;
            }

            const int requiredAcquireFrames =
                ACQUIRE_FRAMES + (pending_.contourFallback ? 2 : 0);
            if (acquireCount_ >= requiredAcquireFrames) {
                locked_ = true;
                misses_ = 0;
                center_ = pending_.center;
                radius_ = pending_.radius;
                confidence_ = pending_.score;
                velocity_ = {};
                acquireAnchor_ = center_;
                acquireAnchorConfigured_ = true;
                hasTrackHistory_ = true;
                result.measured = true;
                pending_ = {};
                acquireCount_ = 0;
            }
        } else {
            pending_ = {};
            acquireCount_ = 0;
        }

        result.locked = locked_;
        result.center = center_;
        result.radius = radius_;
        result.confidence = confidence_;
        result.houghCandidates = lastHoughCandidates_;
        result.validCandidates = lastValidCandidates_;
        result.darkBlobCandidates = lastDarkBlobCandidates_;
        result.missStreak = misses_;
        result.contourFallback =
            result.measured && candidate.contourFallback;
        result.fused = result.measured && candidate.fused;
        return result;
    }
};

inline void drawBall(cv::Mat& frame, const cv::Point2f& center,
                     float radius,
                     const cv::Scalar& color)
{
    const cv::Point point(cvRound(center.x), cvRound(center.y));
    cv::circle(frame, point, cvRound(radius), color, 2, cv::LINE_AA);
    cv::line(frame, {point.x - 6, point.y},
             {point.x + 6, point.y}, color, 1, cv::LINE_AA);
    cv::line(frame, {point.x, point.y - 6},
             {point.x, point.y + 6}, color, 1, cv::LINE_AA);

}

} // namespace ball_stepper
