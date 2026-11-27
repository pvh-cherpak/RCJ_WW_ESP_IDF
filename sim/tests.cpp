#include "replay.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
using namespace sim;
void expect(bool condition, const char* description)
{
    if (!condition) throw std::runtime_error(description);
}
void run(World& world, const Action& action, int ticks)
{
    for (int i = 0; i < ticks; ++i) world.step(action);
}
}

int main()
{
    try {
        soccer::Input observation;
        soccer::Keeper keeper;
        expect(keeper.step(observation).rotation == -10, "missing ball triggers search");
        observation.own = {45, 21, 10};
        expect(keeper.step(observation).rotation == 18, "search rotates to visible gate");
        observation.strength = 35;
        observation.own = {};
        expect(keeper.step(observation).rotation == 20, "visible ball with missing gate");
        observation.own = {180, 19, 10};
        observation.ballAngle = 20;
        expect(keeper.step(observation).speed == 27, "keeper follows ball on center line");
        soccer::Keeper tunedKeeper;
        soccer::Coefficients gains;
        gains.keeperBallP = 0;
        gains.keeperBallD = 0;
        expect(tunedKeeper.step(observation, gains).speed == 20,
               "keeper lateral gains can be overridden");

        const auto tuningPath = std::filesystem::absolute("test-strategy.conf");
        {
            std::ofstream tuning(tuningPath);
            tuning << "keeperSearchP 0.8\n";
        }
        const auto tuning = loadProfile(tuningPath.string());
        std::filesystem::remove(tuningPath);
        soccer::Input search;
        search.own = {45, 21, 10};
        expect(keeper.step(search, tuning.strategy).rotation == 36,
               "newly declared strategy coefficient loads and changes the command");

        soccer::Forward forward;
        soccer::Input attack;
        attack.strength = 75;
        attack.captured = true;
        attack.opponent = {0, 30, 20, -25, 25, 50};
        expect(forward.step(attack).roller == 30 && forward.phase == soccer::Forward::Seat,
               "forward starts capture seating");
        attack.now = 290;
        expect(forward.step(attack).speed == 20 && forward.phase == soccer::Forward::Seat,
               "forward maintains seating motion");
        attack.now = 300;
        expect(!forward.step(attack).kick && forward.phase == soccer::Forward::Settle,
               "forward settles before attack");
        attack.now = 500;
        expect(forward.step(attack).kick, "forward kicks when goal opening is aligned");
        attack.captured = false;
        attack.strength = 4;
        expect(forward.step(attack).rotation == 20, "forward searches again after loss");

        Action action;
        action.settings.camera = false;
        action.settings.locator = false;
        action.settings.cameraReadMs = 40;
        action.settings.locatorReadMs = 15;
        auto world = scenario({}, 1);
        run(world, action, 4);
        expect(world.robots[0].observed.strength == 0, "locator reading still pending");
        run(world, action, 4);
        expect(world.robots[0].locatorTime == 0 && world.robots[0].observed.strength > 0,
               "locator publishes delayed sample");
        expect(world.robots[0].cameraTime == -1, "camera still reading");
        run(world, action, 1);
        expect(world.robots[0].cameraTime == 0, "camera publishes frame after read time");

        auto slow = action;
        slow.settings.cameraFps = 10;
        slow.settings.cameraReadMs = 0;
        auto fast = slow;
        fast.settings.cameraFps = 50;
        auto slowWorld = scenario({}, 1);
        auto fastWorld = slowWorld;
        run(slowWorld, slow, 19);
        run(fastWorld, fast, 19);
        expect(slowWorld.robots[0].cameraTime == 0 && fastWorld.robots[0].cameraTime == 16,
               "camera frame rate independent of read time");

        auto tunedProfile = Profile{};
        tunedProfile.cameraUnitsPerMeter *= 2;
        tunedProfile.locatorFalloff = 0;
        auto tunedWorld = scenario(tunedProfile, 1);
        run(tunedWorld, slow, 1);
        expect(tunedWorld.robots[0].observed.own.distance >
               slowWorld.robots[0].observed.own.distance, "camera calibration changes distance units");
        run(tunedWorld, slow, 4);
        expect(tunedWorld.robots[0].observed.strength == 100, "locator calibration changes decay");

        world = scenario({}, 1);
        action.held = 0;
        action.target = {.72, .15};
        run(world, action, 16);
        expect(world.ball.p.x > .05 && world.ball.v.x > 0, "mouse drag accelerates the ball");
        action.held = -1;
        world.step(action);
        expect(world.ball.v.x > 0, "release preserves momentum");
        world.place(0, {.7, .3});
        expect(length(world.ball.v) == 0, "paused placement resets velocity");
        const auto before = world.ball.p;
        world.place(0, world.robots[0].body.p);
        expect(length(world.ball.p - before) == 0, "paused placement rejects overlap");

        world = scenario({}, 1);
        world.ball = {{1.11, 0}, {3, 0}};
        run(world, action, 20);
        expect(world.score[0] == 1, "goal on contact with rear wall");
        run(world, action, 20);
        expect(world.score[0] == 1, "goal is not counted repeatedly");

        world = scenario({}, 2);
        auto& striker = world.robots[1];
        world.ball.p = striker.body.p + unit(striker.heading) * .12;
        run(world, action, 10);
        expect(length(world.spin) > 1, "roller spins the ball");
        const double spin = length(world.spin);
        world.count = 0; // Isolate floor friction after releasing the roller.
        world.ball.p = {.4, .5};
        world.ball.v = {};
        run(world, action, 400);
        expect(length(world.spin) < spin, "spin decays outside contact");

        world = scenario({}, 4);
        History history(world);
        action.settings = {};
        for (int i = 0; i < historySteps + 50; ++i) {
            action.held = i % 100 < 30 ? 0 : -1;
            action.target = {0, .25 * std::sin(i * .01)};
            history.add(action);
            world.step(action);
        }
        auto replay = history.snapshot();
        expect(replay.initial.tick == 50 && replay.actions.size() == historySteps, "history is bounded");
        const auto path = std::filesystem::absolute("test-sandbox.rcj");
        saveReplay(replay, path.string());
        replay = loadReplay(path.string());
        std::filesystem::remove(path);
        auto recovered = replay.initial;
        for (const auto& item : replay.actions) recovered.step(item);
        expect(recovered.tick == world.tick && recovered.random == world.random &&
               length(recovered.ball.p - world.ball.p) == 0 &&
               length(recovered.spin - world.spin) == 0 &&
               recovered.score == world.score && recovered.eventCount == world.eventCount,
               "saved replay reproduces world");
        for (int i = 0; i < world.count; ++i) {
            auto& a = recovered.robots[i];
            auto& b = world.robots[i];
            expect(length(a.body.p - b.body.p) == 0 &&
                   a.observed.ballAngle == b.observed.ballAngle &&
                   a.observed.strength == b.observed.strength &&
                   a.command.speed == b.command.speed && a.command.rotation == b.command.rotation &&
                   a.command.direction == b.command.direction, "replay reproduces controller");
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
