#include "world.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace sim {
namespace {
int degrees(double radians) { return int(std::lround(radians * 180 / pi)); }
double radians(int degrees) { return degrees * pi / 180; }
int bearing(Vec vector, double heading) {
    return soccer::angle(degrees(std::remainder(heading - std::atan2(vector.y, vector.x), 2 * pi)));
}
int side(double x) { return x < 0 ? -1 : 1; }

soccer::Gate observeGate(World& world, const Robot& robot, int goal, const Settings& settings)
{
    const Vec delta = Vec{goal * world.profile.lineX, 0} - robot.body.p;
    if (settings.camera && world.uniform() < settings.cameraDrop) return {};
    const double range = length(delta);
    const double angleError = settings.camera ? settings.cameraAngleBias +
        (2 * world.uniform() - 1) * settings.cameraAngleNoise : 0;
    const double rangeError = settings.camera ?
        (2 * world.uniform() - 1) * settings.cameraRangeNoise * range : 0;
    // Approximate camera distance units pending calibration against the drive camera.
    const int center = soccer::angle(bearing(delta, robot.heading) + int(std::round(angleError)));
    const int half = degrees(std::atan2(world.profile.goalHalf, std::max(.05, range)));
    return {center,
            int(std::clamp((range + rangeError) * world.profile.cameraUnitsPerMeter, 1., 999.)),
            int(std::clamp(18 / std::max(.05, range), 1., 100.)),
            center - half, center + half, 2 * half};
}

bool inRoller(const World& world, const Robot& robot)
{
    const Vec forward = unit(robot.heading);
    const Vec delta = world.ball.p - robot.body.p;
    const double front = dot(delta, forward);
    const double reach = world.profile.radius + ballRadius;
    return front > reach - .026 && front < reach + world.profile.rollerReach &&
           std::abs(dot(delta, perpendicular(forward))) < world.profile.rollerWidth;
}

void sampleCamera(World& world, Robot& robot, const Settings& config)
{
    if (world.tick >= robot.nextCamera) {
        Robot::Frame frame;
        frame.sampled = world.tick;
        frame.due = world.tick + (config.cameraReadMs + 4) / 5;
        frame.own = observeGate(world, robot, robot.side, config);
        frame.opponent = observeGate(world, robot, -robot.side, config);
        if (robot.frameCount < int(robot.frames.size())) {
            robot.frames[(robot.frameStart + robot.frameCount) % robot.frames.size()] = frame;
            ++robot.frameCount;
        }
        robot.nextCamera += 200 / config.cameraFps;
        if (robot.nextCamera <= world.tick) robot.nextCamera = world.tick + 200 / config.cameraFps;
    }
    while (robot.frameCount && robot.frames[robot.frameStart].due <= world.tick) {
        const auto& frame = robot.frames[robot.frameStart];
        robot.observed.own = frame.own;
        robot.observed.opponent = frame.opponent;
        robot.cameraTime = frame.sampled;
        robot.frameStart = (robot.frameStart + 1) % robot.frames.size();
        --robot.frameCount;
    }

}

void sense(World& world, Robot& robot, const Settings& config)
{
    auto& input = robot.observed;
    input.now = world.tick * 5;
    input.yaw = degrees(std::remainder(robot.heading - (robot.side < 0 ? 0 : pi), 2 * pi));
    const auto publishLocator = [&] {
        input.strength = robot.pendingStrength;
        input.ballAngle = robot.pendingAngle;
        robot.locatorTime = robot.locatorSampled;
        robot.locatorDue = -1;
    };
    if (robot.locatorDue >= 0 && robot.locatorDue <= world.tick) publishLocator();
    if (robot.locatorDue < 0) {
        const Vec delta = world.ball.p - robot.body.p;
        const int trueAngle = bearing(delta, robot.heading);
        const double idealStrength = 100 / (1 + world.profile.locatorFalloff * dot(delta, delta));
        double strength = idealStrength;
        double error = 0;
        if (config.locator) {
            strength = idealStrength * (0.86 + .14 * std::cos(radians(trueAngle))) +
                (2 * world.uniform() - 1) * config.locatorStrengthNoise;
            const double weakness = 1 - std::clamp(strength / 100, 0., 1.);
            error = config.locatorAngleBias +
                (2 * world.uniform() - 1) * config.locatorAngleNoise * (1 + 3 * weakness);
            if (world.uniform() < config.locatorDrop * weakness) strength = 0;
        }
        robot.pendingStrength = int(std::clamp(strength, 0., 100.));
        robot.pendingAngle = soccer::angle(trueAngle + int(std::round(error)));
        robot.locatorSampled = world.tick;
        robot.locatorDue = world.tick + (config.locatorReadMs + 4) / 5;
        if (robot.locatorDue == world.tick) publishLocator();
    }
    input.captured = inRoller(world, robot);

    const auto& p = world.profile;
    const Vec pos = robot.body.p;
    Vec line;
    if (std::abs(std::abs(pos.x) - p.lineX) < p.radius) line.x = side(pos.x);
    if (std::abs(std::abs(pos.y) - p.lineY) < p.radius) line.y = side(pos.y);
    const double edge = p.lineX - p.penaltyDepth;
    if (std::abs(std::abs(pos.x) - edge) < p.radius &&
        std::abs(pos.y) < p.penaltyHalf) line.x = side(pos.x);
    if (std::abs(pos.x) > edge && std::abs(std::abs(pos.y) - p.penaltyHalf) < p.radius)
        line.y = -side(pos.y);
    input.line = length(line) > 0 ? bearing(line, robot.heading) : 360;
}

void control(Robot& robot, const Profile& profile)
{
    const auto& input = robot.observed;
    if (robot.role == Role::Keeper) {
        if (input.now - robot.clearStart < 100) return;
        if (input.now - robot.clearStart < 10100) {
            robot.command = input.opponent.height > 0 ? soccer::clearBall(input, 0, profile.strategy.clearRound) : soccer::Command{0, 20};
            return;
        }
        if (robot.command.clear) robot.keeper.lastMove = input.now;
        robot.command = robot.keeper.step(input, profile.strategy);
        if (robot.command.clear) robot.clearStart = input.now;
    } else {
        robot.command = robot.forward.step(input, profile.strategy);
    }
}

void contact(World& world, int a, int b)
{
    auto& left = world.body(a);
    auto& right = world.body(b);
    const Vec delta = right.p - left.p;
    const double distance = length(delta);
    const double overlap = world.radius(a) + world.radius(b) - distance;
    if (overlap <= 0) return;
    const Vec normal = distance > 1e-9 ? delta * (1 / distance) : Vec{1, 0};
    const double weight = a == 0 ? .95 : .5;
    left.p += normal * (-overlap * weight);
    right.p += normal * (overlap * (1 - weight));
    const double approach = dot(right.v - left.v, normal);
    if (approach < 0) {
        left.v += normal * (approach * weight * (1 + world.profile.bodyBounce));
        right.v += normal * (-approach * (1 - weight) * (1 + world.profile.bodyBounce));
    }
}

bool walls(Body& body, double x, double y, double bounce)
{
    const bool hit = std::abs(body.p.x) >= x || std::abs(body.p.y) >= y;
    if (std::abs(body.p.x) >= x) {
        body.v.x = -side(body.p.x) * std::max(0., side(body.p.x) * body.v.x) * bounce;
        body.p.x = side(body.p.x) * x;
    }
    if (std::abs(body.p.y) >= y) {
        body.v.y = -side(body.p.y) * std::max(0., side(body.p.y) * body.v.y) * bounce;
        body.p.y = side(body.p.y) * y;
    }
    return hit;
}
}

double World::uniform()
{
    random ^= random << 13;
    random ^= random >> 17;
    random ^= random << 5;
    return double(random) / 4294967296.0;
}

void World::emit(Event::Kind kind, int object)
{
    events[eventCount % events.size()] = {kind, object, tick};
    ++eventCount;
}

void World::place(int held, Vec p)
{
    if (held < 0 || held > count) return;
    Body candidate{p, {}};
    const double x = held ? profile.halfLength :
        (std::abs(p.y) < profile.goalHalf - ballRadius ?
         profile.lineX + profile.goalDepth : profile.halfLength);
    walls(candidate, x - radius(held), profile.halfWidth - radius(held), profile.wallBounce);
    for (int i = 0; i <= count; ++i)
        if (i != held && length(candidate.p - body(i).p) < radius(held) + radius(i))
            return;
    body(held) = candidate;
    if (!held) {
        spin = {};
        goalContact = 0;
    }
}

void World::step(const Action& action)
{
    if (action.place) {
        place(action.held, action.target);
        return;
    }
    for (int i = 0; i < count; ++i) sampleCamera(*this, robots[i], action.settings);
    if (tick % 2 == 0) {
        for (int i = 0; i < count; ++i) {
            sense(*this, robots[i], action.settings);
            control(robots[i], profile);
        }
    }
    for (int i = 0; i < count; ++i) {
        auto& robot = robots[i];
        const auto cmd = robot.command;
        if (cmd.roller >= 0) robot.roller = cmd.roller;
        if (action.held != i + 1) {
            const Vec velocity = unit(robot.heading - radians(cmd.direction)) *
                (profile.maxSpeed * std::clamp(cmd.speed, 0, 100) / 100);
            robot.body.v += limit(velocity - robot.body.v, profile.acceleration * dt);
            robot.heading = std::remainder(robot.heading - std::clamp(cmd.rotation, -100, 100) *
                                           (profile.turnRate / 100) * dt, 2 * pi);
        }
        if (!inRoller(*this, robot)) continue;
        const Vec forward = unit(robot.heading);
        if (cmd.kick && tick - robot.lastKick >= 60 && action.held != i + 1) {
            ball.v = robot.body.v + forward * profile.kickSpeed;
            robot.lastKick = tick;
            emit(Event::Kick, i + 1);
            continue;
        }
        if (robot.roller == 0 || action.held == i + 1) continue;
        const Vec cross = perpendicular(forward);
        const double surface = dot(spin, cross) * ballRadius;
        const Vec relative = ball.v - robot.body.v;
        const double slip = -profile.rollerSpeed * robot.roller / 100. - surface -
                            dot(relative, forward);
        const double traction = std::clamp(slip * 12, -profile.rollerGrip, profile.rollerGrip);
        ball.v += forward * (traction * dt);
        spin += cross * (traction * dt * 2.5 / ballRadius);
        ball.v += limit(cross * (-dot(relative, cross) * 10), profile.rollerGrip / 2) * dt;
    }
    const Vec rolling = perpendicular(spin) * (-ballRadius);
    const Vec floorGrip = limit((rolling - ball.v) * 5, profile.floorGrip);
    ball.v += floorGrip * dt;
    spin += perpendicular(floorGrip) * (-dt * 2.5 / ballRadius);
    ball.v = ball.v * (std::exp(-profile.ballDrag * dt));
    spin = spin * (std::exp(-profile.spinDrag * dt));

    if (action.held >= 0 && action.held <= count) {
        auto& held = body(action.held);
        held.v += limit((action.target - held.p) * 90 - held.v * 13,
                        action.held == 0 ? 48 : 23) * dt;
    }
    for (int i = 0; i <= count; ++i) body(i).p += body(i).v * dt;
    for (int a = 0; a <= count; ++a)
        for (int b = a + 1; b <= count; ++b) contact(*this, a, b);

    if (std::abs(ball.p.x) > profile.lineX &&
        std::abs(ball.p.y) > profile.goalHalf - ballRadius &&
        std::abs(ball.p.y - ball.v.y * dt) <= profile.goalHalf - ballRadius) {
        ball.p.y = side(ball.p.y) * (profile.goalHalf - ballRadius);
        ball.v.y *= -profile.wallBounce;
    }
    const double goalBack = profile.lineX + profile.goalDepth - ballRadius;
    if (std::abs(ball.p.x) < profile.lineX - ballRadius) goalContact = 0;
    if (std::abs(ball.p.y) <= profile.goalHalf - ballRadius &&
        std::abs(ball.p.x) >= goalBack) {
        const int goal = side(ball.p.x);
        if (goal != goalContact) {
            ++score[goal > 0 ? 0 : 1];
            emit(Event::Goal, goal);
            goalContact = goal;
        }
        walls(ball, goalBack, profile.halfWidth - ballRadius, profile.wallBounce);
    } else {
        walls(ball, profile.halfLength - ballRadius, profile.halfWidth - ballRadius, profile.wallBounce);
    }
    for (int i = 0; i < count; ++i) {
        auto& robot = robots[i];
        const bool wall = walls(robot.body, profile.halfLength - profile.radius,
                                profile.halfWidth - profile.radius, profile.wallBounce);
        const Vec p = robot.body.p;
        const bool penalty = std::abs(p.x) - profile.radius > profile.lineX - profile.penaltyDepth &&
                             std::abs(p.y) + profile.radius < profile.penaltyHalf;
        if (wall && !robot.wall) emit(Event::Wall, i + 1);
        if (penalty && !robot.penalty) emit(Event::Penalty, i + 1);
        robot.wall = wall;
        robot.penalty = penalty;
    }
    ++tick;
}

World scenario(const Profile& profile, int count)
{
    World world;
    world.profile = profile;
    world.count = std::clamp(count, 1, 4);
    for (int i = 0; i < world.count; ++i) {
        auto& robot = world.robots[i];
        robot.side = i < 2 ? -1 : 1;
        robot.role = i % 2 == 0 ? Role::Keeper : Role::Forward;
        robot.heading = robot.side < 0 ? 0 : pi;
        robot.body.p = {robot.side * profile.lineX * (i % 2 ? .36 : .72),
                        i % 2 ? robot.side * .20 : 0};
        robot.clearStart = -10100;
    }
    return world;
}

World loadScenario(const Profile& profile, const std::string& path)
{
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Cannot open scenario " + path);
    World result;
    result.profile = profile;
    result.count = 0;
    std::string line;
    bool hasBall = false;
    std::array<int, 2> teams{};
    const auto inside = [&](Vec p, double radius) {
        return std::isfinite(p.x) && std::isfinite(p.y) &&
               std::abs(p.x) + radius < profile.halfLength &&
               std::abs(p.y) + radius < profile.halfWidth;
    };
    while (std::getline(file, line)) {
        std::istringstream record(line.substr(0, line.find('#')));
        std::string type;
        if (!(record >> type)) continue;
        if (type == "ball" && !hasBall &&
            (record >> result.ball.p.x >> result.ball.p.y) && inside(result.ball.p, ballRadius)) {
            hasBall = true;
            continue;
        }
        Robot robot;
        std::string team, role;
        double heading;
        if (type != "robot" || result.count == 4 ||
            !(record >> team >> role >> robot.body.p.x >> robot.body.p.y >> heading) ||
            (team != "blue" && team != "gold") || (role != "keeper" && role != "forward") ||
            !std::isfinite(heading) || !inside(robot.body.p, profile.radius))
            throw std::runtime_error("Invalid scenario: " + line);
        robot.side = team == "blue" ? -1 : 1;
        robot.role = role == "keeper" ? Role::Keeper : Role::Forward;
        robot.heading = std::remainder(heading, 360.) * pi / 180;
        robot.clearStart = -10100;
        if (++teams[robot.side > 0] > 2) throw std::runtime_error("Maximum two robots per team");
        result.robots[result.count++] = robot;
    }
    if (!hasBall || result.count == 0) throw std::runtime_error("Scenario needs a ball and at least one robot");
    for (int a = 0; a <= result.count; ++a)
        for (int b = a + 1; b <= result.count; ++b)
            if (length(result.body(a).p - result.body(b).p) < result.radius(a) + result.radius(b))
                throw std::runtime_error("Scenario contains overlapping bodies");
    return result;
}

namespace {
struct StrategyField {
    const char* name;
    double soccer::Coefficients::*member;
    double minimum, maximum;
};

constexpr StrategyField strategyFields[] = {
#define SOCCER_COEFFICIENT(name, initial, minimum, maximum) {#name, &soccer::Coefficients::name, minimum, maximum},
#include SOCCER_COEFFICIENTS_FILE
#undef SOCCER_COEFFICIENT
};
}

std::string strategyProfileDefaults()
{
    const soccer::Coefficients defaults;
    std::ostringstream entries;
    for (const auto& field : strategyFields)
        entries << field.name << ' ' << defaults.*field.member << '\n';
    return entries.str();
}

Profile loadProfile(const std::string& path)
{
    Profile profile;
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Cannot open profile " + path);
    const std::unordered_map<std::string, double Profile::*> keys{
        {"halfLength", &Profile::halfLength}, {"halfWidth", &Profile::halfWidth},
        {"lineX", &Profile::lineX}, {"lineY", &Profile::lineY}, {"goalHalf", &Profile::goalHalf},
        {"goalDepth", &Profile::goalDepth}, {"penaltyDepth", &Profile::penaltyDepth},
        {"penaltyHalf", &Profile::penaltyHalf}, {"neutralX", &Profile::neutralX},
        {"neutralY", &Profile::neutralY}, {"radius", &Profile::radius},
        {"maxSpeed", &Profile::maxSpeed}, {"acceleration", &Profile::acceleration},
        {"rollerSpeed", &Profile::rollerSpeed}, {"rollerGrip", &Profile::rollerGrip},
        {"rollerWidth", &Profile::rollerWidth}, {"rollerReach", &Profile::rollerReach},
        {"kickSpeed", &Profile::kickSpeed},
        {"turnRate", &Profile::turnRate}, {"ballDrag", &Profile::ballDrag},
        {"spinDrag", &Profile::spinDrag}, {"floorGrip", &Profile::floorGrip},
        {"wallBounce", &Profile::wallBounce}, {"bodyBounce", &Profile::bodyBounce},
        {"cameraUnitsPerMeter", &Profile::cameraUnitsPerMeter}, {"locatorFalloff", &Profile::locatorFalloff}};
    const auto permitsZero = [](const std::string& key) {
        return key == "ballDrag" || key == "spinDrag" || key == "floorGrip" ||
               key == "wallBounce" || key == "bodyBounce" || key == "locatorFalloff";
    };
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream record(line.substr(0, line.find('#')));
        std::string key;
        if (!(record >> key)) continue;
        double value;
        if (!(record >> value) || !std::isfinite(value))
            throw std::runtime_error("Invalid profile entry: " + key);

        const auto field = std::find_if(std::begin(strategyFields), std::end(strategyFields),
                                        [&](const StrategyField& entry) { return entry.name == key; });
        if (field != std::end(strategyFields)) {
            if (value < field->minimum || value > field->maximum)
                throw std::runtime_error("Strategy coefficient out of range: " + key);
            profile.strategy.*field->member = value;
            continue;
        }

        if (!keys.count(key) || value < 0 || (value == 0 && !permitsZero(key)))
            throw std::runtime_error("Invalid profile entry: " + key);
        profile.*keys.at(key) = value;
    }
    if (profile.lineX >= profile.halfLength || profile.lineY >= profile.halfWidth ||
        profile.penaltyDepth >= profile.lineX || profile.goalHalf >= profile.lineY ||
        profile.penaltyHalf >= profile.lineY || profile.lineX + profile.goalDepth > profile.halfLength ||
        profile.radius >= profile.lineY || profile.goalHalf <= ballRadius)
        throw std::runtime_error("Inconsistent field geometry");
    if (profile.wallBounce > 1 || profile.bodyBounce > 1)
        throw std::runtime_error("Bounce must be 0–1");
    return profile;
}
Settings loadSensors(const std::string& path)
{
    Settings settings;
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Cannot open sensor settings " + path);
    const std::unordered_map<std::string, double Settings::*> keys{
        {"cameraFps", &Settings::cameraFps}, {"cameraAngleBias", &Settings::cameraAngleBias},
        {"cameraAngleNoise", &Settings::cameraAngleNoise}, {"cameraRangeNoise", &Settings::cameraRangeNoise},
        {"cameraDrop", &Settings::cameraDrop}, {"locatorAngleBias", &Settings::locatorAngleBias},
        {"locatorAngleNoise", &Settings::locatorAngleNoise},
        {"locatorStrengthNoise", &Settings::locatorStrengthNoise}, {"locatorDrop", &Settings::locatorDrop}};
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream record(line.substr(0, line.find('#')));
        std::string key;
        if (!(record >> key)) continue;
        double value;
        if (!(record >> value) || !std::isfinite(value))
            throw std::runtime_error("Invalid sensor setting: " + key);
        if ((key == "cameraReadMs" || key == "locatorReadMs") &&
            value >= 0 && value <= 600 && std::floor(value) == value) {
            (key == "cameraReadMs" ? settings.cameraReadMs : settings.locatorReadMs) = int(value);
        } else if (keys.count(key)) {
            settings.*keys.at(key) = value;
        } else {
            throw std::runtime_error("Invalid sensor setting: " + key);
        }
    }
    if (settings.cameraFps < 1 || settings.cameraFps > 100 ||
        settings.cameraDrop < 0 || settings.cameraDrop > 1 ||
        settings.locatorDrop < 0 || settings.locatorDrop > 1 ||
        settings.cameraAngleNoise < 0 || settings.cameraAngleNoise > 180 ||
        settings.locatorAngleNoise < 0 || settings.locatorAngleNoise > 180 ||
        settings.cameraRangeNoise < 0 || settings.cameraRangeNoise > 1 ||
        settings.locatorStrengthNoise < 0 || settings.locatorStrengthNoise > 100 ||
        std::abs(settings.cameraAngleBias) > 180 || std::abs(settings.locatorAngleBias) > 180)
        throw std::runtime_error("Sensor setting out of range");
    return settings;
}

}
