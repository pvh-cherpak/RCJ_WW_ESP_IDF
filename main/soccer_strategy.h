#pragma once

#include <algorithm>
#include <cmath>

#ifndef SOCCER_COEFFICIENTS_FILE
#define SOCCER_COEFFICIENTS_FILE "soccer_coefficients.def"
#endif

namespace soccer {

struct Coefficients {
#define SOCCER_COEFFICIENT(name, initial, minimum, maximum) double name = initial;
#include SOCCER_COEFFICIENTS_FILE
#undef SOCCER_COEFFICIENT
};

inline int angle(int value)
{
    value %= 360;
    if (value > 180) value -= 360;
    if (value < -180) value += 360;
    return value;
}

struct Gate {
    int angle = 360;
    int distance = 0;
    int height = 0;
    int left = 360, right = 360, width = 0;
};

struct Input {
    int now = 0;
    int yaw = 0;
    int ballAngle = 0;
    int strength = 0;
    int line = 360;
    bool captured = false;
    Gate own;
    Gate opponent;
};

struct Command {
    int direction = 0;
    int rotation = 0;
    int speed = 0;
    int roller = -1;
    bool kick = false;
    bool clear = false;
};

struct Keeper {
    int lastBall = 0;
    int lastStrength = 0;
    int lastMove = 0;
    int previousError = 0;
    int previousGate = 0;

    Command step(const Input& in, const Coefficients& gains = {})
    {
        if (in.strength < 2)
            return {0, in.own.height > 0 ? int(angle(in.own.angle) * float(gains.keeperSearchP)) : -10};

        const int robot = angle(in.yaw - 180);
        const int ball = angle(in.ballAngle + 180);
        const int gate = angle(in.own.angle + 180);
        const int globalGate = angle(gate + robot);
        if (in.own.height <= 0) return {0, 20};

        const int line = in.line == 360 ? 360 : angle(in.line + 180);
        if (std::abs(angle(ball + robot - lastBall)) > 10 ||
            std::abs(in.strength - lastStrength) > 15) {
            lastBall = ball + robot;
            lastStrength = in.strength;
            lastMove = in.now;
        }
        if (in.now - lastMove >= 5000)
            return {180, 0, 70, -1, false, true};

        if (line != 360 && std::abs(globalGate) <= 135 &&
            ((robot > 0 && line > 0) || (robot < 0 && line < 0)))
            return {line, int(angle(gate - 180) * 0.4f), 100};

        const int distance = in.own.distance;
        int forward;
        if (std::abs(globalGate) < 135) {
            forward = distance < 10 ? 70 : distance < 12 ? 20 :
                      distance < 14 ? 0 : -(distance - 14) * 7;
        } else {
            forward = distance < 15 ? 80 : distance < 18 ? 40 :
                      distance < 20 ? 20 : distance < 25 ? 0 : -(distance - 25) * 5;
        }
        forward = std::clamp(forward, -int(gains.keeperReverseLimit), 100);
        if (std::abs(ball) > 80 && forward <= 0) {
            const int offset = std::clamp(int(in.strength * float(gains.keeperRound)), 0, 90);
            return {angle(ball + (ball > 0 ? offset : -offset) + 180),
                    int(-angle(180 - gate) * 0.25f), 50};
        }

        const int error = angle(in.ballAngle - in.yaw);
        const int lateral = std::abs(error) > 35 ? (error < 0 ? -50 : 50) :
                              float(gains.keeperBallP) * error + float(gains.keeperBallD) * (error - previousError);
        previousError = error;
        const int turn = angle(gate - 180);
        const int rotation = turn * float(gains.keeperGateP) + (turn - previousGate) * float(gains.keeperGateD);
        previousGate = turn;
        return {angle(int(std::atan2(float(lateral), float(forward)) * 57.2957795f + in.yaw)),
                rotation, std::min(100, int(std::hypot(float(lateral), float(forward)))),
                -1, in.captured && std::abs(angle(in.own.angle)) > 90};
    }
};

inline Command chase(const Input& in, bool& far, float& previous,
                     float frontGain = 0.35f, float turnGain = 0.7f, float derivative = 3.5f)
{
    if (in.strength < 5) {
        far = true;
        return {0, 20};
    }
    int error = in.ballAngle;
    if (far && in.strength < 35) error = angle(error + 90);
    else far = false;

    const int roller = std::abs(in.ballAngle) < 60 ? 30 : 0;
    if (in.line != 360)
        return {angle(in.line + 180), int(error * 0.3), 80, roller};

    const int rotation = std::abs(error) <= 15 ? frontGain * error :
                         turnGain * error + derivative * (error - previous);
    previous = error;
    return {in.ballAngle, rotation, in.strength < 65 ? 100 : 30, roller};
}

inline int angleErrorToRange(int cur, int from, int to, int perfect)
{

    int left_error = angle(from - cur);
    int right_error = angle(to - cur);
    if (left_error <= 0.0f && right_error >= 0.0f)
        return angle(perfect - cur);

    return std::abs(left_error) < std::abs(right_error) ? left_error : right_error;
}

inline int kickError(const Gate& gate, int yaw)
{
    if (gate.left > 0 || gate.right < 0)
        return gate.angle;

    bool gate_corner = std::abs(angle(gate.angle + yaw)) > 30;

    const int min_offset = 10;

    int asymmetric_offset = gate_corner
        ? std::min(15, gate.width / 2)
        : 0;

    int global_gate_angle = angle(gate.angle + yaw);

    int left_offset = min_offset;
    int right_offset = min_offset;
    if (global_gate_angle > 0)
        left_offset += asymmetric_offset;
    else
        right_offset += asymmetric_offset;

    if (gate.width >= left_offset + right_offset)
    {
        return angleErrorToRange(
            0,
            gate.left + left_offset,
            gate.right - right_offset,
            gate.angle);
    }

    int left_limit, right_limit, perfect_angle;
    if (global_gate_angle > 0)
    {
        right_limit = gate.right - min_offset;
        if (gate.width >= left_offset + min_offset) {
            left_limit = gate.left + left_offset;
        } else if (gate.width >= min_offset + min_offset) {
            left_limit = gate.left + min_offset;
        } else {
            left_limit = right_limit;
        }

        perfect_angle = right_limit;
    } else {
        left_limit = gate.left + min_offset;
        if (gate.width >= min_offset + right_offset) {
            right_limit = gate.right - right_offset;
        } else if (gate.width >= min_offset + min_offset) {
            right_limit = gate.right - min_offset;
        } else {
            right_limit = left_limit;
        }

        perfect_angle = left_limit;
    }

    return angleErrorToRange(0, left_limit, right_limit, perfect_angle);
}

inline int kickRotation(int err)
{
    int rotation_speed = err * 0.2f;
    if (std::abs(rotation_speed) >= 15.0f)
        return static_cast<int>(rotation_speed);
    if (rotation_speed == 0.0f)
        return 0;
    return rotation_speed < 0 ? -10 : 10;
}
inline Command clearBall(const Input& in, int offset = 0, float roundGain = 2.5f)
{
    const int gate = -in.yaw;
    const int rotation = angle(gate - offset) * 0.25f;
    if (in.line != 360) return {angle(in.line + 180), rotation, 50};
    const int error = angle(in.ballAngle - gate);
    if (std::abs(error) <= 15 || in.captured)
        return {gate, rotation, 80, -1, in.opponent.distance < 50 && in.captured};

    const int curve = std::clamp(int(in.strength * roundGain), 0, 90);
    return {angle(in.ballAngle + (error > 0 ? curve : -curve)), rotation,
            std::abs(in.ballAngle) < 40 ? 40 : 80};
}

struct Forward {
    enum Phase { Seek, Seat, Settle, Approach, BackIn, StopForSpin, TurnForSpin, Spin, Align, Recover };
    Phase phase = Seek;
    int since = 0;
    int spinYaw = 0;
    int spinSign = 1;
    bool far = false;
    float previous = 0;
    Command last;

    Command step(const Input& in, const Coefficients& gains = {})
    {
        const auto& gate = in.opponent;
        const int age = in.now - since;
        const bool onLine = in.line != 360;
        const int retreat = angle(in.line + 180);
        const auto enter = [&](Phase next) { phase = next; since = in.now; };
        const auto hold = [&] {
            Command command = last;
            command.kick = false;
            command.roller = -1;
            return command;
        };

        if (phase == Align && (!in.captured || onLine || age > 2000)) {
            enter(Seek);
            return last = {};
        }
        if (phase == Spin && !in.captured) enter(Recover);
        if (phase == Recover) {
            if (in.now - since < 100) return hold();
            enter(Seek);
        }
        if (!in.captured && phase != Seat && phase != Settle) enter(Seek);
        if (phase == Seek) {
            if (!in.captured || in.strength < 5)
                return last = chase(in, far, previous, gains.forwardFrontP, gains.forwardTurnP, gains.forwardTurnD);
            enter(Seat);
            return last = {0, 0, 20, 30};
        }
        if (phase == Seat) {
            if (!onLine && age < 300) return hold();
            enter(Settle);
            if (onLine) return last = {retreat, 0, 80};
            return hold();
        }
        if (phase == Settle) {
            if (age < 200) return hold();
            enter(Approach);
            if (!in.captured) {
                enter(Seek);
                return last = chase(in, far, previous, gains.forwardFrontP, gains.forwardTurnP, gains.forwardTurnD);
            }
        }
        if (phase == StopForSpin) {
            if (age < 100) return hold();
            enter(TurnForSpin);
        }
        if (phase == TurnForSpin) {
            if (std::abs(gate.angle) < 160)
                return last = {0, std::clamp(angle(gate.angle + 180), -20, 20) / 2};
            spinYaw = in.yaw;
            spinSign = std::abs(angle(gate.angle + in.yaw)) < 30 ?
                       ((in.now / 10) % 2 ? -1 : 1) : (in.yaw < 0 ? -1 : 1);
            enter(Spin);
        }
        if (phase == Spin) {
            const int turned = std::abs(angle(in.yaw - spinYaw));
            return last = {onLine ? retreat : 0, onLine ? 0 : spinSign * (turned < 50 ? 18 : 80),
                           onLine ? 80 : 0, turned < 60 ? 180 : -1, turned >= 60};
        }
        if (phase == Approach && gate.angle != 360 && gate.left != 360 && gate.right != 360 &&
            gate.distance >= 0 && gate.distance <= 70 &&
            gate.width > (std::abs(angle(gate.angle + in.yaw)) > 30 ? 20 : 40)) {
            enter(Align);
        }
        if (phase == Align) {
            if (onLine) {
                enter(Seek);
                return last = {};
            }
            if (gate.angle == 360 || gate.left == 360 || gate.right == 360 || gate.width <= 0) return hold();
            const int error = kickError(gate, in.yaw);
            if (std::abs(error) <= 5) {
                enter(Seek);
                return last = {0, 0, 0, -1, true};
            }
            return last = {0, std::clamp(kickRotation(error), -20, 20)};
        }
        if (phase == BackIn && std::abs(gate.angle) < 130) {
            const int turn = angle(gate.angle + 180) * (std::abs(gate.angle) < 90 ? 0.3f : 0.5f);
            return last = {retreat, std::clamp(turn, -10, 10), onLine ? 60 : 0};
        }
        if (onLine) {
            enter(Approach);
            return last = {retreat, 0, 80};
        }
        if (gate.angle == 360) {
            enter(Approach);
            return last = {0, 0, 0};
        }
        if (std::abs(gate.angle) < 130) {
            enter(BackIn);
            const int turn = angle(gate.angle + 180) * (std::abs(gate.angle) < 90 ? 0.3f : 0.5f);
            return last = {retreat, std::clamp(turn, -10, 10)};
        }
        if (gate.distance <= 30) {
            enter(StopForSpin);
            return last = {0, 0, 0, 30};
        }
        enter(Approach);
        const int turn = angle(gate.angle + 180) * 0.3f;
        return last = {gate.angle, std::clamp(turn, -10, 10), 75};
    }
};

}
