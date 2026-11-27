#pragma once

#include "soccer_strategy.h"
#include <array>
#include <cstdint>
#include <cmath>
#include <string>

namespace sim {
constexpr double dt = .005;
constexpr double pi = 3.141592653589793;
constexpr double ballRadius = .022;
constexpr int historySteps = 6000;

struct Vec {
    double x = 0, y = 0;
    Vec operator+(Vec b) const { return {x + b.x, y + b.y}; }
    Vec operator-(Vec b) const { return {x - b.x, y - b.y}; }
    Vec operator*(double k) const { return {x * k, y * k}; }
    Vec& operator+=(Vec b) { return *this = *this + b; }
};
inline double dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y; }
inline double length(Vec a) { return std::hypot(a.x, a.y); }
inline Vec unit(double a) { return {std::cos(a), std::sin(a)}; }
inline Vec perpendicular(Vec a) { return {-a.y, a.x}; }
inline Vec limit(Vec a, double max) { return a * std::min(1., max / std::max(length(a), 1e-12)); }

struct Profile {
    double halfLength = 1.215, halfWidth = .91;
    double lineX = 1.095, lineY = .79;
    double goalHalf = .30, goalDepth = .12, penaltyDepth = .23, penaltyHalf = .40;
    double neutralX = .45, neutralY = .40;
    double radius = .09, maxSpeed = 1.6, acceleration = 5;
    double turnRate = 6.5, ballDrag = .85, spinDrag = 1.5;
    double floorGrip = 3.5, wallBounce = .48, bodyBounce = .4;
    double cameraUnitsPerMeter = 60, locatorFalloff = 8;
    soccer::Coefficients strategy;
    double rollerSpeed = 8, rollerGrip = 12, rollerWidth = .038, rollerReach = .045, kickSpeed = 4;
};

struct Settings {
    double cameraAngleBias = 0, cameraAngleNoise = 2, cameraRangeNoise = .06;
    double cameraDrop = .02;
    double cameraFps = 30;
    int cameraReadMs = 40, locatorReadMs = 5;
    double locatorAngleBias = 0, locatorAngleNoise = 2, locatorStrengthNoise = 2, locatorDrop = .04;
    bool camera = true, locator = true;
};
enum class Role { Keeper, Forward };
struct Body { Vec p, v; };
struct Robot {
    Body body;
    double heading = 0;
    int side = -1;
    Role role = Role::Keeper;
    soccer::Keeper keeper;
    soccer::Forward forward;
    soccer::Input observed;
    soccer::Command command;
    bool wall = false, penalty = false;
    int roller = 0, lastKick = -100, clearStart = -10000;
    int cameraTime = -1, locatorTime = -1;
    double nextCamera = 0;
    int locatorDue = -1, locatorSampled = 0, pendingAngle = 0, pendingStrength = 0;
    struct Frame { soccer::Gate own, opponent; int due = 0, sampled = 0; };
    std::array<Frame, 64> frames{};
    int frameStart = 0, frameCount = 0;
};
struct Action {
    Settings settings;
    Vec target;
    int held = -1; // 0 is the ball; 1–4 are robots.
    bool place = false;
};
struct Event {
    enum Kind { Goal, Wall, Penalty, Kick };
    Kind kind = Goal;
    int object = 0, tick = 0;
};
struct World {
    Profile profile;
    Body ball;
    Vec spin;
    std::array<Robot, 4> robots{};
    std::array<int, 2> score{};
    std::array<Event, 32> events{};
    int tick = 0, count = 1, eventCount = 0, goalContact = 0;
    std::uint32_t random = 35787;

    void step(const Action& action);
    void place(int held, Vec p);
    Body& body(int id) { return id == 0 ? ball : robots.at(id - 1).body; }
    const Body& body(int id) const { return id == 0 ? ball : robots.at(id - 1).body; }
    double radius(int id) const { return id == 0 ? ballRadius : profile.radius; }
    void emit(Event::Kind kind, int object);
    double uniform();
};
World scenario(const Profile& profile, int count);
World loadScenario(const Profile& profile, const std::string& path);
Profile loadProfile(const std::string& path);
std::string strategyProfileDefaults();
Settings loadSensors(const std::string& path);
}
