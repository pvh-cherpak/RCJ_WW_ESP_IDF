#include "replay.h"

#include <raylib.h>

#include <utility>

namespace {
using namespace sim;

constexpr Color ink{225, 235, 240, 255};
constexpr Color muted{155, 180, 192, 255};
constexpr Color bg{17, 31, 39, 255};
constexpr Color grass{31, 94, 70, 255};
constexpr Color whiteLine{206, 226, 206, 255};
constexpr Color blue{97, 185, 250, 255};
constexpr Color gold{250, 201, 90, 255};
Font interfaceFont;

struct Layout {
    float scale;
    Vector2 center;
    explicit Layout(const Profile& profile)
        : scale(float(std::min(840 / (2 * profile.halfLength), 610 / (2 * profile.halfWidth)))),
          center{442, 378} {}
    Vector2 point(Vec p) const { return {center.x + float(p.x) * scale, center.y - float(p.y) * scale}; }
    Vec world(Vector2 p) const { return {(p.x - center.x) / scale, (center.y - p.y) / scale}; }
    Rectangle box(double x1, double y1, double x2, double y2) const {
        const auto top = point({x1, y2});
        return {top.x, top.y, float(x2 - x1) * scale, float(y2 - y1) * scale};
    }
};

void label(const std::string& text, int x, int y, int size = 18, Color color = ink)
{
    DrawTextEx(interfaceFont, text.c_str(), {float(x), float(y)}, float(size), 1, color);
}

void arrow(const Layout& layout, Vec start, Vec delta, Color color)
{
    const auto a = layout.point(start);
    const auto b = layout.point(start + delta);
    DrawLineEx(a, b, 2, color);
    if (length(delta) > .002) DrawCircleV(b, 3, color);
}

void drawWorld(const World& world, const Layout& layout, int selected)
{
    const auto& p = world.profile;
    DrawRectangleRec(layout.box(-p.halfLength, -p.halfWidth, p.halfLength, p.halfWidth), grass);
    DrawRectangleLinesEx(layout.box(-p.halfLength, -p.halfWidth, p.halfLength, p.halfWidth), 2, muted);
    DrawRectangleLinesEx(layout.box(-p.lineX, -p.lineY, p.lineX, p.lineY), 1, whiteLine);
    DrawCircleLinesV(layout.center, .25f * layout.scale, whiteLine);
    arrow(layout, {0, -p.lineY}, {0, 2 * p.lineY}, whiteLine);
    DrawRectangleLinesEx(layout.box(-p.lineX, -p.penaltyHalf, -p.lineX + p.penaltyDepth, p.penaltyHalf), 1, whiteLine);
    DrawRectangleLinesEx(layout.box(p.lineX - p.penaltyDepth, -p.penaltyHalf, p.lineX, p.penaltyHalf), 1, whiteLine);
    DrawRectangleLinesEx(layout.box(-p.lineX - p.goalDepth, -p.goalHalf, -p.lineX, p.goalHalf), 3, blue);
    DrawRectangleLinesEx(layout.box(p.lineX, -p.goalHalf, p.lineX + p.goalDepth, p.goalHalf), 3, gold);
    for (int x : {-1, 1})
        for (int y : {-1, 1})
            DrawCircleV(layout.point({x * p.neutralX, y * p.neutralY}), 3, whiteLine);
    DrawCircleV(layout.point(world.ball.p), float(ballRadius) * layout.scale, ORANGE);
    arrow(layout, world.ball.p, limit(world.ball.v * .09, .25), ORANGE);

    for (int i = 0; i < world.count; ++i) {
        const auto& robot = world.robots[i];
        const auto center = layout.point(robot.body.p);
        const auto color = robot.side < 0 ? blue : gold;
        DrawCircleV(center, float(p.radius) * layout.scale, color);
        if (i == selected) DrawCircleLinesV(center, float(p.radius) * layout.scale + 3, WHITE);
        arrow(layout, robot.body.p, unit(robot.heading) * p.radius, bg);
        label(std::to_string(i + 1), int(center.x) - 4, int(center.y) - 7, 13, bg);
        if (i != selected) continue;
        const auto& input = robot.observed;
        arrow(layout, robot.body.p, world.ball.p - robot.body.p, Fade(ORANGE, .38f));
        if (input.strength > 0)
            arrow(layout, robot.body.p, unit(robot.heading - input.ballAngle * pi / 180) * .56, ORANGE);
        if (input.own.height)
            arrow(layout, robot.body.p, unit(robot.heading - input.own.angle * pi / 180) *
                  std::min(.85, input.own.distance / p.cameraUnitsPerMeter), color);
        if (input.opponent.height)
            arrow(layout, robot.body.p, unit(robot.heading - input.opponent.angle * pi / 180) *
                  std::min(.85, input.opponent.distance / p.cameraUnitsPerMeter), robot.side < 0 ? gold : blue);
        if (robot.command.speed)
            arrow(layout, robot.body.p, unit(robot.heading - robot.command.direction * pi / 180) *
                  (std::clamp(robot.command.speed, 0, 100) * .004), WHITE);
    }
}

void slider(const char* title, double& value, double low, double high, int y, bool editable)
{
    const Rectangle track{904, float(y + 21), 250, 10};
    if (editable && IsMouseButtonDown(MOUSE_BUTTON_LEFT) &&
        CheckCollisionPointRec(GetMousePosition(), {904, float(y + 14), 250, 24}))
        value = low + (high - low) * std::clamp((GetMouseX() - track.x) / track.width, 0.f, 1.f);
    label(std::string(title) + "  " + TextFormat("%.1f", value), 904, y, 15, muted);
    DrawRectangleRec(track, Color{47, 73, 86, 255});
    DrawCircle(int(track.x + track.width * (value - low) / (high - low)), int(track.y + 5), 6, blue);
}

void sensorsPanel(const World& world, int selected, Settings& settings, bool editable)
{
    const auto& robot = world.robots[selected];
    const auto& input = robot.observed;
    label(robot.role == Role::Keeper ? "Goalkeeper" : "Forward", 904, 88, 24);
    label(TextFormat("Locator  %d deg  |  %d", input.ballAngle, input.strength), 904, 124);
    label(TextFormat("Camera  %d deg  |  %d", input.own.angle, input.own.distance), 904, 149);
    label(TextFormat("Age  C %d ms  L %d ms",
                     robot.cameraTime < 0 ? -1 : (world.tick - robot.cameraTime) * 5,
                     robot.locatorTime < 0 ? -1 : (world.tick - robot.locatorTime) * 5), 904, 174, 16, muted);
    label(TextFormat("Drive  %d / %d / %d", robot.command.direction, robot.command.speed,
                     robot.command.rotation), 904, 205);
    label(TextFormat("Roller  %d%%   Spin  %.0f rad/s", robot.roller, length(world.spin)), 904, 229, 16);
    label(settings.camera ? "C  Camera noise on" : "C  Camera noise off", 904, 277);
    slider("FPS", settings.cameraFps, 1, 100, 305, editable);
    double cameraRead = settings.cameraReadMs;
    slider("Camera read ms", cameraRead, 0, 600, 348, editable);
    settings.cameraReadMs = int(std::lround(cameraRead));
    slider("Angle noise, deg", settings.cameraAngleNoise, 0, 180, 391, editable);
    slider("Range noise, fraction", settings.cameraRangeNoise, 0, 1, 434, editable);
    label(settings.locator ? "L  Locator noise on" : "L  Locator noise off", 904, 479);
    double locatorRead = settings.locatorReadMs;
    slider("Locator read ms", locatorRead, 0, 600, 507, editable);
    settings.locatorReadMs = int(std::lround(locatorRead));
    slider("Angle noise, deg", settings.locatorAngleNoise, 0, 180, 550, editable);
    slider("Strength noise", settings.locatorStrengthNoise, 0, 100, 593, editable);
}

std::string lastEvent(const World& world)
{
    if (world.eventCount == 0) return {};
    const auto& event = world.events[(world.eventCount - 1) % world.events.size()];
    switch (event.kind) {
        case Event::Goal: return event.object > 0 ? "Blue goal" : "Gold goal";
        case Event::Wall: return "Wall contact, robot " + std::to_string(event.object);
        case Event::Penalty: return "Penalty area, robot " + std::to_string(event.object);
        case Event::Kick: return "Kick, robot " + std::to_string(event.object);
    }
    return {};
}
}

int openWindow(sim::World initial, sim::Settings settings, const std::string& output,
               const sim::Replay* replay)
{
    sim::World world = replay ? replay->initial : initial;
    sim::History history(world);
    InitWindow(1180, 755, "RCJ Soccer Simulator");
    if (!IsWindowReady()) return 1;
    const std::string fontPath = std::string(GetApplicationDirectory()) + "comic.ttf";
    interfaceFont = FileExists(fontPath.c_str()) ? LoadFontEx(fontPath.c_str(), 48, nullptr, 0)
                                                 : GetFontDefault();
    const bool customFont = interfaceFont.texture.id != GetFontDefault().texture.id;
    if (customFont) SetTextureFilter(interfaceFont.texture, TEXTURE_FILTER_BILINEAR);
    else TraceLog(LOG_WARNING, "Comic Sans MS missing: place comic.ttf next to rcj-sim");
    SetTargetFPS(60);
    bool paused = false;
    int held = -1;
    int selected = 0;
    double elapsed = 0;
    std::size_t position = 0;
    std::string status;
    while (!WindowShouldClose()) {
        const Layout layout(world.profile);
        if (IsKeyPressed(KEY_SPACE)) { paused = !paused; elapsed = 0; }
        if (IsKeyPressed(KEY_R)) {
            world = replay ? replay->initial : initial;
            position = 0;
            history = sim::History(world);
            selected = 0;
            held = -1;
            elapsed = 0;
            status.clear();
        }
        if (!replay) {
            if (IsKeyPressed(KEY_C)) settings.camera = !settings.camera;
            if (IsKeyPressed(KEY_L)) settings.locator = !settings.locator;
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && GetMouseX() < 884) {
                const auto target = layout.world(GetMousePosition());
                for (int id = 0; id <= world.count; ++id) {
                    if (length(target - world.body(id).p) > world.radius(id)) continue;
                    held = id;
                    if (id > 0) selected = id - 1;
                    break;
                }
            }
            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) held = -1;
            if (paused && held >= 0) {
                const sim::Action action{settings, layout.world(GetMousePosition()), held, true};
                if (length(action.target - world.body(held).p) > .0001) {
                    world.step(action);
                    history.add(action);
                }
            }
        }
        if (IsKeyPressed(KEY_TAB)) selected = (selected + 1) % world.count;
        if (IsKeyPressed(KEY_F5) && !replay) {
            try {
                sim::saveReplay(history.snapshot(), output);
                status = "Saved " + output;
            } catch (const std::exception& error) {
                status = error.what();
            }
        }
        if (paused && IsKeyPressed(KEY_N)) elapsed = 2 * sim::dt;
        if (!paused) elapsed += std::min(double(GetFrameTime()), .1);
        while (elapsed >= sim::dt) {
            elapsed -= sim::dt;
            if (replay && position >= replay->actions.size()) {
                paused = true;
                status = "Replay finished";
                elapsed = 0;
                break;
            }
            sim::Action action{settings, layout.world(GetMousePosition()), held, false};
            if (replay) {
                action = replay->actions[position++];
                settings = action.settings;
                if (action.place) elapsed += sim::dt;
            }
            world.step(action);
            if (!replay) history.add(action);
        }

        BeginDrawing();
        ClearBackground(bg);
        label("RCJ Soccer Simulator", 22, 18, 25);
        label(TextFormat("BLUE %d   -   %d GOLD", world.score[0], world.score[1]), 347, 24, 21);
        label(TextFormat("%.1f s   %s", world.tick * sim::dt, paused ? "Paused" : "Running"), 660, 24);
        drawWorld(world, layout, selected);
        sensorsPanel(world, selected, settings, !replay);
        label(lastEvent(world), 22, 693, 16);
        label(status, 500, 693, 16, muted);
        label("Drag objects   |   Space pause   |   N step   |   Tab inspect   |   R reset   |   F5 save",
              22, 721, 17, muted);
        EndDrawing();
    }
    if (customFont) UnloadFont(interfaceFont);
    CloseWindow();
    return 0;
}
