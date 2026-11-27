#include "replay.h"

#include <fstream>
#include <stdexcept>
#include <type_traits>

namespace sim {
namespace {
constexpr std::uint64_t format = 0x52434a0000000002;
template<class T> void write(std::ofstream& stream, const T& value)
{
    static_assert(std::is_trivially_copyable_v<T>);
    stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

template<class T> void read(std::ifstream& stream, T& value)
{
    static_assert(std::is_trivially_copyable_v<T>);
    if (!stream.read(reinterpret_cast<char*>(&value), sizeof(T)))
        throw std::runtime_error("Incomplete replay");
}

bool valid(const Action& a, int count)
{
    const auto& sensor = a.settings;
    return a.held >= -1 && a.held <= count && std::isfinite(a.target.x) &&
           std::isfinite(a.target.y) && sensor.cameraFps >= 1 && sensor.cameraFps <= 100 &&
           sensor.cameraReadMs >= 0 && sensor.cameraReadMs <= 600 &&
           sensor.locatorReadMs >= 0 && sensor.locatorReadMs <= 600;
}
}

void History::add(const Action& action)
{
    actions.push_back(action);
    while (actions.size() > historySteps) {
        start.step(actions.front());
        actions.pop_front();
    }
}

Replay History::snapshot() const
{
    return {start, {actions.begin(), actions.end()}};
}

void saveReplay(const Replay& replay, const std::string& path)
{
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    write(stream, format);
    write(stream, std::uint32_t(sizeof(World)));
    write(stream, std::uint32_t(sizeof(Action)));
    write(stream, std::uint32_t(replay.actions.size()));
    write(stream, replay.initial);
    for (const auto& action : replay.actions) write(stream, action);
    if (!stream) throw std::runtime_error("Cannot write replay " + path);
}

Replay loadReplay(const std::string& path)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("Cannot open replay " + path);
    std::uint64_t version;
    std::uint32_t worldSize, actionSize, count;
    read(stream, version);
    read(stream, worldSize);
    read(stream, actionSize);
    read(stream, count);
    if (version != format || worldSize != sizeof(World) ||
        actionSize != sizeof(Action) || count > historySteps)
        throw std::runtime_error("Incompatible replay");
    Replay replay;
    read(stream, replay.initial);
    const auto& initial = replay.initial;
    if (initial.count < 1 || initial.count > 4 || initial.tick < 0 ||
        initial.tick > 400000000 || initial.eventCount < 0)
        throw std::runtime_error("Invalid replay state");
    for (int i = 0; i < initial.count; ++i) {
        const auto& robot = initial.robots[i];
        if (robot.frameStart < 0 || robot.frameStart >= int(robot.frames.size()) ||
            robot.frameCount < 0 || robot.frameCount > int(robot.frames.size()))
            throw std::runtime_error("Invalid camera queue");
    }
    replay.actions.resize(count);
    for (auto& action : replay.actions) {
        read(stream, action);
        if (!valid(action, initial.count)) throw std::runtime_error("Invalid replay action");
    }
    return replay;
}

void resetControllers(World& world)
{
    for (auto& robot : world.robots) {
        robot.keeper = {};
        robot.keeper.lastMove = world.tick * 5;
        robot.clearStart = -10100;
        robot.forward = {};
        robot.command = {};
        robot.roller = 0;
        robot.lastKick = world.tick - 100;
    }
}
}
