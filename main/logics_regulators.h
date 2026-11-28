#pragma once

#include <cstdint>
#include "GyverPID.h"
#include "PIDtuner.h"

class HeadingRegulator
{
public:
    struct PIDGains
    {
        float kp;
        float ki;
        float kd;
    };

    enum class AutoTuneStatus { Idle, Running, Complete, Failed };

    // Angles are in degrees. Call only when a new target measurement arrives.
    void setTarget(float angle);

    // Returns a signed motor rotation command. Time is monotonic, in microseconds.
    int update(float currentAngle, std::int64_t nowUs);

    float getTargetAngle() const { return targetAngle; }
    float getAngularRate() const { return angularRate; }

    // Simple PD using the target supplied by setTarget().
    int updateSimplePD(float currentAngle, std::int64_t nowUs);

    void setGyverPIDGains(PIDGains gains);
    int updateGyverPID(float currentAngle, std::int64_t nowUs);

    // The relay test turns the robot in both directions around its starting heading.
    bool startAutoTune(int rotationStep, std::int64_t nowUs);
    int updateAutoTune(float currentAngle, std::int64_t nowUs);
    void stopAutoTune();
    AutoTuneStatus autoTuneStatus() const { return tuneStatus; }
    bool getAutoTuneGains(PIDGains& gains) const;

    void reset();

private:
    float targetAngle = 0.0f;
    float previousAngle = 0.0f;
    float angularRate = 0.0f;
    std::int64_t previousUpdateUs = 0;
    bool haveTarget = false;
    bool turning = false;
    float previousSimpleError = 0.0f;
    std::int64_t previousSimpleUpdateUs = 0;
    bool haveSimpleSample = false;

    PIDGains gyverGains{0.5f, 0.0f, 0.15f};
    gyver::GyverPID gyverPID{};
    bool haveGyverSample = false;
    float previousGyverAngle = 0.0f;
    std::int64_t previousGyverUpdateUs = 0;

    gyver::PIDtuner tuner{};
    AutoTuneStatus tuneStatus = AutoTuneStatus::Idle;
    PIDGains tunedGains{};
    std::int64_t tuneStartUs = 0;
    float tuneStartAngle = 0.0f;
    bool haveTuneStartAngle = false;
};

class HeadingStepAutoTuner
{
public:
    void start(float currentAngle, std::int64_t nowUs);
    int update(float currentAngle, std::int64_t nowUs);
    HeadingRegulator::AutoTuneStatus status() const { return tuneStatus; }
    bool getGains(HeadingRegulator::PIDGains& gains) const;

private:
    HeadingRegulator controller;
    HeadingRegulator::AutoTuneStatus tuneStatus = HeadingRegulator::AutoTuneStatus::Idle;
    HeadingRegulator::PIDGains bestGains{};
    float origin = 0.0f;
    float target = 0.0f;
    float direction = 1.0f;
    float peakOvershoot = 0.0f;
    float candidateScore = 0.0f;
    float bestScore = 0.0f;
    int trial = 0;
    std::int64_t trialStartUs = 0;
    std::int64_t insideSinceUs = 0;
    std::int64_t nextTrialUs = 0;
    bool trialActive = false;
    bool candidateSettled = true;
    bool haveBest = false;
};
