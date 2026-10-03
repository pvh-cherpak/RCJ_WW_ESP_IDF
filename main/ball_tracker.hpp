#pragma once
#include <stdint.h>

struct BallTrackerConfig {
    int s_on = 12;
    int s_off = 5;
    int fresh_ms = 150;
    int coast_ms = 700;
    int vel_window_ms = 300;
    int vel_buf_len = 8;
    float vel_deadband = 20.f;
    float vel_max = 360.f;
    float coast_max_dev = 35.f;
};

enum class BallTrackState {
    NO_BALL,
    TRACKING,
    COASTING,
    HOLD,
};

struct BallEstimate {
    BallTrackState state = BallTrackState::NO_BALL;
    float local = 0.f;
    float global = 0.f;
    float vel = 0.f;
    int age_ms = 0;
    bool usable = false;
};

class BallTracker {
public:
    explicit BallTracker(const BallTrackerConfig& cfg = BallTrackerConfig());

    void update(int loc_angle, int strength, int yaw, int now_ms);
    BallEstimate get(int yaw_now, int now_ms) const;

    void reset();
    int lastAge(int now_ms) const;

private:
    static float goodAngleF(float a);

    BallTrackerConfig cfg_;

    struct Sample {
        int t = 0;
        float g = 0.f;
    };
    static const int kBufMax = 10;
    Sample buf_[kBufMax];
    int buf_n_ = 0;

    bool has_last_ = false;
    float last_g_ = 0.f;
    int last_t_ = 0;
    bool armed_ = false;

    float vel_ = 0.f;

    void pushSample(int t, float g);
    float estimateVel(int now_ms) const;
};
