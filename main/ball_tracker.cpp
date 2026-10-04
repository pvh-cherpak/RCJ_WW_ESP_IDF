#include "ball_tracker.hpp"

BallTracker::BallTracker(const BallTrackerConfig& cfg)
    : cfg_(cfg) {
}

float BallTracker::goodAngleF(float a) {
    while (a > 180.f) {
        a -= 360.f;
    }
    while (a < -180.f) {
        a += 360.f;
    }
    return a;
}

void BallTracker::reset() {
    buf_n_ = 0;
    has_last_ = false;
    last_g_ = 0.f;
    last_t_ = 0;
    armed_ = false;
    vel_ = 0.f;
}

int BallTracker::lastAge(int now_ms) const {
    if (!has_last_) {
        return -1;
    }
    int age = now_ms - last_t_;
    return age < 0 ? 0 : age;
}

void BallTracker::pushSample(int t, float g) {
    int cap = cfg_.vel_buf_len;
    if (cap < 2) {
        cap = 2;
    }
    if (cap > kBufMax) {
        cap = kBufMax;
    }
    if (buf_n_ < cap) {
        buf_[buf_n_].t = t;
        buf_[buf_n_].g = g;
        buf_n_++;
    } else {
        for (int i = 1; i < cap; i++) {
            buf_[i - 1] = buf_[i];
        }
        buf_[cap - 1].t = t;
        buf_[cap - 1].g = g;
    }
}

float BallTracker::estimateVel(int now_ms) const {
    if (buf_n_ < 2) {
        return 0.f;
    }
    int newest_idx = buf_n_ - 1;
    int oldest_idx = newest_idx;
    for (int i = newest_idx; i >= 0; i--) {
        int age = now_ms - buf_[i].t;
        if (age < 0) {
            continue;
        }
        if (age <= cfg_.vel_window_ms) {
            oldest_idx = i;
        } else {
            break;
        }
    }
    if (oldest_idx == newest_idx) {
        return 0.f;
    }
    int dt_ms = buf_[newest_idx].t - buf_[oldest_idx].t;
    if (dt_ms <= 0) {
        return 0.f;
    }
    float d = goodAngleF(buf_[newest_idx].g - buf_[oldest_idx].g);
    float v = d * 1000.f / (float)dt_ms;
    if (v > cfg_.vel_max || v < -cfg_.vel_max) {
        return 0.f;
    }
    if (v < cfg_.vel_deadband && v > -cfg_.vel_deadband) {
        return 0.f;
    }
    return v;
}

void BallTracker::update(int loc_angle, int strength, int yaw, int now_ms) {
    bool angle_ok = (loc_angle != 360) && (loc_angle >= -180) && (loc_angle <= 180);
    if (!angle_ok) {
        armed_ = false;
        return;
    }
    if (strength >= cfg_.s_on) {
        armed_ = true;
    } else if (strength < cfg_.s_off) {
        armed_ = false;
        return;
    } else if (!armed_) {
        return;
    }
    float g = goodAngleF((float)loc_angle + (float)yaw);
    last_g_ = g;
    last_t_ = now_ms;
    has_last_ = true;
    pushSample(now_ms, g);
    vel_ = estimateVel(now_ms);
}

BallEstimate BallTracker::get(int yaw_now, int now_ms) const {
    BallEstimate estim;
    if (!has_last_) {
        return estim;
    }
    int age = now_ms - last_t_;
    if (age < 0) {
        age = 0;
    }
    estim.age_ms = age;
    estim.vel = vel_;
    float pred;
    if (age < cfg_.fresh_ms) {
        estim.state = BallTrackState::TRACKING;
        pred = last_g_;
    } else if (age < cfg_.coast_ms) {
        estim.state = BallTrackState::COASTING;
        pred = last_g_ + vel_ * ((float)age / 1000.f);
        float dev = goodAngleF(pred - last_g_);
        if (dev > cfg_.coast_max_dev) {
            pred = last_g_ + cfg_.coast_max_dev;
        } else if (dev < -cfg_.coast_max_dev) {
            pred = last_g_ - cfg_.coast_max_dev;
        }
        pred = goodAngleF(pred);
    } else {
        estim.state = BallTrackState::HOLD;
        pred = last_g_;
    }
    estim.global = goodAngleF(pred);
    estim.local = goodAngleF(estim.global - (float)yaw_now);
    estim.usable = (estim.state == BallTrackState::TRACKING) ||
        (estim.state == BallTrackState::COASTING);
    return estim;
}
