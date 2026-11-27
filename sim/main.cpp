#include "replay.h"

#include <iomanip>
#include <iostream>
#include <stdexcept>

#ifdef RCJ_SIM_GUI
int openWindow(sim::World initial, sim::Settings settings, const std::string& output,
               const sim::Replay* replay);
#endif

int main(int argc, char** argv)
{
    try {
        sim::Profile profile;
        sim::Settings settings;
        int count = 1;
        std::string replayPath;
        std::string scenarioPath;
        std::string output = "soccer.rcj";
        bool headless = false;
        bool comparison = false;
        for (int i = 1; i < argc; ++i) {
            const std::string flag = argv[i];
            const auto value = [&]() -> std::string {
                if (++i >= argc) throw std::runtime_error("Missing argument for " + flag);
                return argv[i];
            };
            if (flag == "--profile") profile = sim::loadProfile(value());
            else if (flag == "--sensors") settings = sim::loadSensors(value());
            else if (flag == "--scenario") scenarioPath = value();
            else if (flag == "--robots") {
                const auto input = value();
                if (input.size() != 1 || input[0] < '1' || input[0] > '4')
                    throw std::runtime_error("Choose 1-4 robots");
                count = input[0] - '0';
            } else if (flag == "--replay") replayPath = value();
            else if (flag == "--output") output = value();
            else if (flag == "--compare") comparison = true;
            else if (flag == "--headless") headless = true;
            else if (flag == "--strategy-keys") {
                std::cout << sim::strategyProfileDefaults();
                return 0;
            } else if (flag == "--help") {
                std::cout << "rcj-sim [--robots 1-4] [--profile FILE] [--sensors FILE] [--scenario FILE]\n"
                          << "        [--output FILE] [--replay FILE [--compare] [--headless]]\n"
                          << "rcj-sim --strategy-keys\n";
                return 0;
            } else throw std::runtime_error("Unknown option " + flag);
        }

        if ((headless || comparison) && replayPath.empty())
            throw std::runtime_error("--headless and --compare require --replay");
        sim::Replay replay;
        if (!replayPath.empty()) {
            replay = sim::loadReplay(replayPath);
            if (comparison) sim::resetControllers(replay.initial);
        }
        if (headless) {
            auto world = replay.initial;
            for (const auto& action : replay.actions) world.step(action);
            std::cout << std::setprecision(17) << "tick " << world.tick
                      << "\nscore " << world.score[0] << ' ' << world.score[1]
                      << "\nball " << world.ball.p.x << ' ' << world.ball.p.y
                      << "\nspin " << world.spin.x << ' ' << world.spin.y << '\n';
            for (int i = 0; i < world.count; ++i) {
                const auto& robot = world.robots[i];
                std::cout << "robot " << i + 1 << ' ' << robot.body.p.x << ' ' << robot.body.p.y
                          << " locator " << robot.observed.ballAngle << ' ' << robot.observed.strength
                          << " drive " << robot.command.direction << ' ' << robot.command.rotation
                          << ' ' << robot.command.speed << '\n';
            }
            for (int i = std::max(0, world.eventCount - int(world.events.size())); i < world.eventCount; ++i) {
                const auto& event = world.events[i % world.events.size()];
                std::cout << "event " << event.tick << ' ' << event.kind << ' ' << event.object << '\n';
            }
            return 0;
        }
#ifdef RCJ_SIM_GUI
        return openWindow(scenarioPath.empty() ? sim::scenario(profile, count) :
                          sim::loadScenario(profile, scenarioPath), settings, output,
                          replayPath.empty() ? nullptr : &replay);
#else
        throw std::runtime_error("Window disabled. Run with --replay FILE --headless.");
#endif
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
