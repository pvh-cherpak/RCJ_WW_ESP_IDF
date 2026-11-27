#pragma once

#include "world.h"

#include <deque>
#include <vector>

namespace sim {
struct Replay {
    World initial;
    std::vector<Action> actions;
};
class History {
public:
    explicit History(World initial) : start(initial) {}
    void add(const Action& action);
    Replay snapshot() const;
private:
    World start;
    std::deque<Action> actions;
};
void saveReplay(const Replay& replay, const std::string& path);
Replay loadReplay(const std::string& path);
void resetControllers(World& world);
}
