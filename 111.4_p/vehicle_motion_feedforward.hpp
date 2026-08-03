#pragma once

#include "system_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ball_stepper {

// One new signed encoder-speed value arrives every 50 ms (20 Hz). A typical
// cruise value is about 80. This is deliberately transport-independent.
struct VehicleEncoderSample {
    double speedUnits = 0.0;
    double sampleTime = -1.0;
    uint64_t sequence = 0;
};

class VehicleEncoderSource {
public:
    virtual ~VehicleEncoderSource() = default;
    virtual bool poll(VehicleEncoderSample& sample) = 0;
};

struct VehicleMotionState {
    double rawSpeedUnits = 0.0;
    double filteredSpeedUnits = 0.0;
    double rawAccelerationUnitsS = 0.0;
    double filteredAccelerationUnitsS = 0.0;
    double feedforwardAngleDeg = 0.0;
    double sampleAgeMs = 1e9;
    bool signalReceived = false;
    bool signalFresh = false;
};

class VehicleMotionFeedforward {
    const AppConfig& config_;
    bool haveSample_ = false;
    uint64_t lastSequence_ = 0;
    double previousSampleTime_ = -1.0;
    double latestSampleTime_ = -1.0;
    double previousUpdateTime_ = -1.0;
    double rawSpeedUnits_ = 0.0;
    double filteredSpeedUnits_ = 0.0;
    double rawAccelerationUnitsS_ = 0.0;
    double filteredAccelerationUnitsS_ = 0.0;

public:
    explicit VehicleMotionFeedforward(const AppConfig& config)
        : config_(config) {}

    void reset(double now = -1.0)
    {
        haveSample_ = false;
        lastSequence_ = 0;
        previousSampleTime_ = -1.0;
        latestSampleTime_ = -1.0;
        previousUpdateTime_ = now;
        rawSpeedUnits_ = 0.0;
        filteredSpeedUnits_ = 0.0;
        rawAccelerationUnitsS_ = 0.0;
        filteredAccelerationUnitsS_ = 0.0;
    }

    bool submitEncoderSample(const VehicleEncoderSample& sample)
    {
        if (!std::isfinite(sample.speedUnits) ||
            !std::isfinite(sample.sampleTime) || sample.sampleTime < 0.0 ||
            std::abs(sample.speedUnits) >
                config_.task4VehicleEncoderMaximumAbsValue ||
            (haveSample_ && sample.sequence != 0 &&
             sample.sequence == lastSequence_)) {
            return false;
        }

        const double signedRawSpeed =
            config_.task4VehicleEncoderDirectionSign * sample.speedUnits;
        if (haveSample_) {
            const double sampleDt = sample.sampleTime - previousSampleTime_;
            if (sampleDt <= 0.0) return false;
            if (sampleDt <=
                config_.task4VehicleSampleMaximumGapMs / 1000.0) {
                rawSpeedUnits_ = signedRawSpeed;
                const double speedAlpha = 1.0 - std::exp(
                    -sampleDt / config_.task4VehicleSpeedFilterSeconds);
                const double previousFilteredSpeed = filteredSpeedUnits_;
                filteredSpeedUnits_ += speedAlpha *
                    (rawSpeedUnits_ - filteredSpeedUnits_);

                rawAccelerationUnitsS_ = std::clamp(
                    (filteredSpeedUnits_ - previousFilteredSpeed) / sampleDt,
                    -config_.task4VehicleAccelerationLimitUnitsS,
                     config_.task4VehicleAccelerationLimitUnitsS);
                const double accelerationAlpha = 1.0 - std::exp(
                    -sampleDt /
                    config_.task4VehicleAccelerationFilterSeconds);
                filteredAccelerationUnitsS_ += accelerationAlpha *
                    (rawAccelerationUnitsS_ -
                     filteredAccelerationUnitsS_);
            } else {
                rawSpeedUnits_ = signedRawSpeed;
                filteredSpeedUnits_ = signedRawSpeed;
                rawAccelerationUnitsS_ = 0.0;
                filteredAccelerationUnitsS_ = 0.0;
            }
        } else {
            // The stream should already be running while TASK4 is paused. The
            // first sample establishes the zero/steady baseline without a fake
            // startup acceleration spike.
            rawSpeedUnits_ = signedRawSpeed;
            filteredSpeedUnits_ = signedRawSpeed;
        }

        haveSample_ = true;
        if (sample.sequence != 0) lastSequence_ = sample.sequence;
        previousSampleTime_ = sample.sampleTime;
        latestSampleTime_ = sample.sampleTime;
        return true;
    }

    VehicleMotionState update(double now)
    {
        const double loopDt = previousUpdateTime_ >= 0.0 ?
            std::clamp(now - previousUpdateTime_, 0.0, 0.1) : 0.0;
        previousUpdateTime_ = now;

        VehicleMotionState state;
        state.signalReceived = haveSample_;
        if (haveSample_) {
            state.sampleAgeMs = std::max(
                0.0, (now - latestSampleTime_) * 1000.0);
            state.signalFresh = state.sampleAgeMs <=
                config_.task4VehicleInputTimeoutMs;
        }

        if (!state.signalFresh) {
            rawAccelerationUnitsS_ = 0.0;
            if (loopDt > 0.0) {
                filteredAccelerationUnitsS_ *= std::exp(
                    -loopDt /
                    config_.task4VehicleAccelerationDecaySeconds);
            }
        }

        double accelerationForAngle = filteredAccelerationUnitsS_;
        if (std::abs(accelerationForAngle) <
            config_.task4VehicleAccelerationDeadbandUnitsS) {
            accelerationForAngle = 0.0;
        }

        state.rawSpeedUnits = rawSpeedUnits_;
        state.filteredSpeedUnits = filteredSpeedUnits_;
        state.rawAccelerationUnitsS = rawAccelerationUnitsS_;
        state.filteredAccelerationUnitsS = filteredAccelerationUnitsS_;
        state.feedforwardAngleDeg = std::clamp(
            config_.task4VehicleAccelerationAngleSign *
                config_.task4VehicleFeedforwardDegPerUnitS *
                accelerationForAngle,
            -config_.task4VehicleFeedforwardLimitDeg,
             config_.task4VehicleFeedforwardLimitDeg);
        return state;
    }
};

} // namespace ball_stepper
