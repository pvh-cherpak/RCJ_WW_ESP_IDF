#include "logics_regulators.h"

#include <algorithm>
#include <cmath>

void HeadingRegulator::setTarget(float angle)
{
    constexpr float targetFilterGain = 0.35f;
    if (haveTarget)
        targetAngle += targetFilterGain * std::remainder(angle - targetAngle, 360.0f);
    else
        targetAngle = angle;
    haveTarget = true;
}

int HeadingRegulator::update(float currentAngle, std::int64_t nowUs)
{
    constexpr float startTurningAngle = 5.0f;
    constexpr float stopTurningAngle = 4.0f;
    constexpr float rateFilterTimeSeconds = 0.05f;
    constexpr float nearAngle = 12.0f;
    constexpr float farAngle = 55.0f;
    constexpr float nearProportionalGain = 0.65f;
    constexpr float farProportionalGain = 1.0f;
    constexpr float nearDampingGain = 0.12f;
    constexpr float farDampingGain = 0.06f;
    constexpr float nearMaxRotation = 22.0f;
    constexpr float farMaxRotation = 60.0f;

    if (!haveTarget)
        return 0;

    if (previousUpdateUs != 0 && nowUs > previousUpdateUs)
    {
        const float dt = (nowUs - previousUpdateUs) * 1e-6f;
        const float measuredRate = std::remainder(currentAngle - previousAngle, 360.0f) / dt;
        angularRate += (dt / (rateFilterTimeSeconds + dt)) * (measuredRate - angularRate);
    }
    previousAngle = currentAngle;
    previousUpdateUs = nowUs;

    const float error = std::remainder(targetAngle - currentAngle, 360.0f);
    const float absError = std::abs(error);
    if (turning)
        turning = absError > stopTurningAngle;
    else
        turning = absError >= startTurningAngle;

    if (!turning)
        return 0;

    // Blend both commands for a smooth transition as the robot approaches the target.
    const float farWeight = std::clamp((absError - nearAngle) / (farAngle - nearAngle), 0.0f, 1.0f);
    const float nearRotation = std::clamp(nearProportionalGain * error - nearDampingGain * angularRate,
                                          -nearMaxRotation, nearMaxRotation);
    const float farProportionalRotation = std::clamp(farProportionalGain * error,
                                                     -farMaxRotation, farMaxRotation);
    const float farRotation = std::clamp(farProportionalRotation - farDampingGain * angularRate,
                                         -farMaxRotation, farMaxRotation);
    return std::lround(nearRotation + farWeight * (farRotation - nearRotation));
}

int HeadingRegulator::updateSimplePD(float currentAngle, std::int64_t nowUs)
{
    constexpr float proportionalGain = 0.65f;
    constexpr float derivativeGain = 0.12f;
    constexpr long maxRotation = 45;

    if (!haveTarget)
        return 0;

    const float error = std::remainder(targetAngle - currentAngle, 360.0f);
    float errorRate = 0.0f;
    if (haveSimpleSample && nowUs > previousSimpleUpdateUs)
    {
        const float dt = (nowUs - previousSimpleUpdateUs) * 1e-6f;
        errorRate = std::remainder(error - previousSimpleError, 360.0f) / dt;
    }
    previousSimpleError = error;
    previousSimpleUpdateUs = nowUs;
    haveSimpleSample = true;

    const long rotation = std::lround(proportionalGain * error + derivativeGain * errorRate);
    return static_cast<int>(std::clamp(rotation, -maxRotation, maxRotation));
}

void HeadingRegulator::setGyverPIDGains(PIDGains gains)
{
    gyverGains = gains;
    haveGyverSample = false;
}

int HeadingRegulator::updateGyverPID(float currentAngle, std::int64_t nowUs)
{
    constexpr int maxRotation = 70;
    if (!haveTarget)
        return 0;

    if (!haveGyverSample)
    {
        gyverPID = gyver::GyverPID(gyverGains.kp, gyverGains.ki, gyverGains.kd, 10);
        gyverPID.setLimits(-maxRotation, maxRotation);
        haveGyverSample = true;
    }
    else
    {
        const auto elapsedMs = (nowUs - previousGyverUpdateUs) / 1000;
        if (elapsedMs <= 0)
            return std::lround(gyverPID.output);

        gyverPID.setDt(elapsedMs);
        // Unwrap the measurement so crossing ±180° cannot spike the D term.
        gyverPID.input += std::remainder(currentAngle - previousGyverAngle, 360.0f);
    }

    previousGyverAngle = currentAngle;
    previousGyverUpdateUs = nowUs;
    gyverPID.setpoint = gyverPID.input + std::remainder(targetAngle - currentAngle, 360.0f);
    return std::lround(gyverPID.getResult());
}

bool HeadingRegulator::startAutoTune(int rotationStep, std::int64_t nowUs)
{
    if (rotationStep <= 0 || rotationStep > 70 || tuneStatus == AutoTuneStatus::Running)
        return false;

    tuner = gyver::PIDtuner{};
    // Normal direction, zero base command, 10 ms sample interval.
    tuner.setParameters(false, 0, rotationStep, 500, 1.5f, 120, 10);
    tuneStartUs = nowUs;
    haveTuneStartAngle = false;
    tuneStatus = AutoTuneStatus::Running;
    return true;
}

int HeadingRegulator::updateAutoTune(float currentAngle, std::int64_t nowUs)
{
    constexpr std::int64_t timeoutUs = 15'000'000;
    constexpr float maxExcursion = 60.0f;
    if (tuneStatus != AutoTuneStatus::Running)
        return 0;

    if (!haveTuneStartAngle)
    {
        tuneStartAngle = currentAngle;
        haveTuneStartAngle = true;
    }
    const auto elapsedUs = nowUs - tuneStartUs;
    const float displacement = std::remainder(currentAngle - tuneStartAngle, 360.0f);
    if (elapsedUs < 0 || elapsedUs >= timeoutUs || std::abs(displacement) >= maxExcursion)
    {
        tuneStatus = AutoTuneStatus::Failed;
        return 0;
    }

    tuner.setInput(displacement);
    tuner.compute(elapsedUs / 1000);
    if (tuner.getAccuracy() >= 95)
    {
        const PIDGains gains{tuner.getPID_p(), tuner.getPID_i(), tuner.getPID_d()};
        if (std::isfinite(gains.kp) && gains.kp > 0.0f &&
            std::isfinite(gains.ki) && gains.ki > 0.0f &&
            std::isfinite(gains.kd) && gains.kd > 0.0f)
        {
            tunedGains = gains;
            tuneStatus = AutoTuneStatus::Complete;
            return 0;
        }
    }
    return tuner.getOutput();
}

void HeadingRegulator::stopAutoTune()
{
    tuneStatus = AutoTuneStatus::Idle;
}

bool HeadingRegulator::getAutoTuneGains(PIDGains& gains) const
{
    if (tuneStatus != AutoTuneStatus::Complete)
        return false;
    gains = tunedGains;
    return true;
}

void HeadingRegulator::reset()
{
    const PIDGains gains = gyverGains;
    *this = HeadingRegulator{};
    gyverGains = gains;
}

namespace
{
constexpr HeadingRegulator::PIDGains stepCandidates[] = {
    {0.6f, 0.0f, 0.08f},
    {0.9f, 0.0f, 0.08f},
    {1.2f, 0.0f, 0.12f},
    {1.2f, 0.0f, 0.22f},
};
constexpr int stepTrialCount = 2 * (sizeof(stepCandidates) / sizeof(stepCandidates[0]));
}

void HeadingStepAutoTuner::start(float currentAngle, std::int64_t nowUs)
{
    *this = HeadingStepAutoTuner{};
    origin = currentAngle;
    nextTrialUs = nowUs + 300'000;
    tuneStatus = HeadingRegulator::AutoTuneStatus::Running;
}

int HeadingStepAutoTuner::update(float currentAngle, std::int64_t nowUs)
{
    using Status = HeadingRegulator::AutoTuneStatus;
    constexpr float stepAngle = 20.0f;
    constexpr float maxExcursion = 35.0f;
    constexpr float tolerance = 2.0f;
    constexpr int maxRotation = 28;
    constexpr std::int64_t trialTimeoutUs = 3'000'000;
    constexpr std::int64_t holdUs = 250'000;

    if (tuneStatus != Status::Running)
        return 0;

    if (std::abs(std::remainder(currentAngle - origin, 360.0f)) >= maxExcursion)
    {
        tuneStatus = Status::Failed;
        return 0;
    }

    if (!trialActive)
    {
        if (nowUs < nextTrialUs)
            return 0;

        // Test the same pair outbound and back before moving to the next pair.
        direction = trial % 2 == 0 ? 1.0f : -1.0f;
        target = direction > 0.0f ? origin + stepAngle : origin;
        controller.reset();
        controller.setGyverPIDGains(stepCandidates[trial / 2]);
        controller.setTarget(target);
        trialStartUs = nowUs;
        insideSinceUs = 0;
        peakOvershoot = 0.0f;
        trialActive = true;
    }

    const float error = std::remainder(target - currentAngle, 360.0f);
    peakOvershoot = std::max(peakOvershoot, -direction * error);
    if (std::abs(error) > tolerance)
        insideSinceUs = 0;
    else if (insideSinceUs == 0)
        insideSinceUs = nowUs;

    const bool settled = insideSinceUs != 0 && nowUs - insideSinceUs >= holdUs;
    const auto elapsedUs = nowUs - trialStartUs;
    if (!settled && elapsedUs < trialTimeoutUs)
        return std::clamp(controller.updateGyverPID(currentAngle, nowUs), -maxRotation, maxRotation);

    candidateSettled = candidateSettled && settled;
    // Score in seconds, with a penalty of 0.25 seconds per degree of overshoot.
    candidateScore += elapsedUs * 1e-6f + 0.25f * peakOvershoot;
    if (trial % 2 != 0)
    {
        if (candidateSettled && (!haveBest || candidateScore < bestScore))
        {
            bestGains = stepCandidates[trial / 2];
            bestScore = candidateScore;
            haveBest = true;
        }
        candidateScore = 0.0f;
        candidateSettled = true;
    }

    ++trial;
    trialActive = false;
    nextTrialUs = nowUs + 300'000;
    if (trial == stepTrialCount)
        tuneStatus = haveBest ? Status::Complete : Status::Failed;

    return 0;
}

bool HeadingStepAutoTuner::getGains(HeadingRegulator::PIDGains& gains) const
{
    if (tuneStatus != HeadingRegulator::AutoTuneStatus::Complete)
        return false;
    gains = bestGains;
    return true;
}
