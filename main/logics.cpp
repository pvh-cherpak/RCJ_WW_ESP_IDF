#include "logics.h"
#include "logics_regulators.h"
#include "esp_timer.h"
#include "vector2.h"
#include "RealDist.h"
#include <algorithm>
#include <cmath>

//[Header("Управляющие переменные")]
float timer = 0;
int stateGame = 0;

//[Header("Игровая логика")]
float moveAngle = 0;
float deltaAngle = 0;
int speed = 0;

int lineSpeed = 50;
float ballAngle = 0;
float lineAngle = 0;
float cameraAngle = 0;

float lastLineAngle = 0;
float lastLineTime = 0;
float lastlineReset = 0;

int speedX = 0;
int speedY = 0;

float isBallDiap = 10;
float isBallStrength = 120;

//[Header("Нападающий")]
int drive2ballSpeed = 50;
int move2gateSpeed = 60;
float goRoundBallDiap = 20;
float goRoundBallCoefFw = 2.5f;
int timeBallHeld = 0;
int gateMovePause = 1000;
float goRoundObstacleCoef = 7.f;
float rotateObstacleCoef = 0.125f;

int dribblerSpeed = 50;

float fb_kp = 3;
float fb_ki = 0;
float fb_kd = 40;
float fwBallIntegral = 0;
float fwBallPrev = 0;

int maxTorqueWithBall = 20;
int maxSpeedBackWithBall = 100;

int criticalObstacleAngle = 120;
int criticalObstacleDistance = 4;

//[Header("Вратарь (простой)")]
int goalkeeperLineSpeed = 200;
int goalkeeperVertSpeed = 100;
float horizCoef = 180;
float mainSpeedY = 0;
int minSpeedY = -100;
float accelerationY = -2;

int timeToPush = 1000;
int stuckTime = 0;
int ballSide = 1;

//[Header("Вратарь (следование по линии)")]
float rotateSlowingCoef = 1;
float considerLineTime = 10;
float lineSensorPriority[16] = {0, 0.8, 0.2, 0.2, 0.2, 0.6, 0.8, 0, 0, 0.8, 0.6, 0.2, 0.2, 0.2, 0.8, 0};
int activeSensors = 8;
int maxGoalkeeperAngle = 80;
float goRoundBallCoefGk = 0.72f; //0.62

// Вратарь (крутой)

// const float g_kp = 50; // хз что это, не используется
// const float g_ki = 0;
// const float g_kd = 30;

float leftIntegral = 0;
float leftPrev = 0;
float rightIntegral = 0;
float rightPrev = 0;

// PID мяча осторожный (близко)
const float gb_kp = 1.8;
const float gb_ki = 0;
const float gb_kd = 0.1;
float gkBallIntegral = 0;
float gkBallPrev = 0;

// PID мяча молния маквин (подалбше)
const float gb_kp2 = 3;
const float gb_ki2 = 0;
const float gb_kd2 = 3;
float gkBallIntegral2 = 0;
float gkBallPrev2 = 0;

float gkReactOnBallDiap = 0;

// Вратарь, возвращение к воротам по камере с PID (не используется)
int lastGateReset = 5000;

float gate_kp = -10;
float gate_kd = 10;
float gate_ki = 0;

float gateIntegral = 0;
float gatePrev = 0;

float limitGateIntegral = 150;
float limitGateSpeed = 90;

// Launch killer feature
int ballSeenTime = 0;
float ballMoveTime = 0;
float lastBallAngle = 0;
const float ballNoMotionDiap = 10;
float lastMoveBallStrength = 0;
const float ballNoMotionStrength = 15;

// Killer feature
float kfTimer = 0;
const float maxKfTime = 10000;
float prevBallStrength = 0;

// Paradox
const int kParadoxStartTime = 1000;
const int kParadoxReset = 300;
int paradoxFirstTime = 0;
int paradoxLastTime = 0;
int paradoxAngle = 360;
const int kParadoxDiap = 30;

int millis()
{
    return micros() / 1000;
}

int constrain(int val, int minim, int maxim)
{
    if (val < minim)
        return minim;
    if (val > maxim)
        return maxim;
    return val;
}

int goodAngle(int angle)
{
    angle %= 360;
    if (angle < -180)
        angle += 360;
    if (angle > 180)
        angle -= 360;
    return angle;
}

void make_pause(int ms)
{
    // vTaskDelay(ms / portTICK_PERIOD_MS);
    int start = millis();
    while (millis() - start < ms)
    {
        sensor.update();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void projectSpeedOnLineXY(int speedX, int speedY, float lineX, float lineY, float &resSpeedX, float &resSpeedY)
{
    if (lineX * speedX + lineY * speedY > 0)
    {
        lineX += lineY;
        lineY = lineX - lineY;
        lineX -= lineY;
        lineX = -lineX;
        float resSpeed = lineX * speedX + lineY * speedY;
        resSpeedX = resSpeed * lineX;
        resSpeedY = resSpeed * lineY;
    }
    else
    {
        resSpeedX = speedX;
        resSpeedY = speedY;
    }
}

bool gates_on_side(int color)
{
    int yaw = sensor.IMU.getYaw();
    int global_left_angle = (int)goodAngle(
        sensor.Cam.gate(color).left_angle + yaw);
    int global_right_angle = (int)goodAngle(
        sensor.Cam.gate(color).right_angle + yaw);
    return global_left_angle * global_right_angle > 0;
}

void projectSpeedOnLine(float speed, float moveAngle, float lineX, float lineY, float &resSpeedX, float &resSpeedY)
{
    int speedX = (int)(speed * sin(moveAngle * DEG_TO_RAD));
    int speedY = (int)(speed * cos(moveAngle * DEG_TO_RAD));
    projectSpeedOnLineXY(speedX, speedY, lineX, lineY, resSpeedX, resSpeedY);
}

void calibrateDistOffset(int color)
{
    sensor.update();
    while (abs(sensor.IMU.getYaw()) > 10)
    {
        // ESP_LOGI("GP", "to 0");
        vTaskDelay(10 / portTICK_PERIOD_MS);
        sensor.update();
        drv.drive(0, -sensor.IMU.getYaw(), 0);
    }

    std::vector<int> angles, dist;

    while (abs(sensor.IMU.getYaw()) < 170)
    {
        // ESP_LOGI("GP", "to 170");
        vTaskDelay(10 / portTICK_PERIOD_MS);
        sensor.update();
        if (sensor.Cam.gate(color).clos_angle != 360 && sensor.Cam.gate(color).distance > 3)
        {
            angles.push_back(sensor.Cam.gate(color).clos_angle);
            dist.push_back(sensor.Cam.gate(color).distance);
        }
        drv.drive(0, 15, 0);
    }

    while (abs(sensor.IMU.getYaw()) > 5)
    {
        // ESP_LOGI("GP", "back to 0");
        vTaskDelay(10 / portTICK_PERIOD_MS);
        sensor.update();
        if (sensor.Cam.gate(color).clos_angle != 360 && sensor.Cam.gate(color).distance > 3)
        {
            angles.push_back(sensor.Cam.gate(color).clos_angle);
            dist.push_back(sensor.Cam.gate(color).distance);
        }
        drv.drive(0, 15, 0);
    }

    drv.drive(0, 0, 0, 0);

    float min_x = 0, max_x = 0, min_y = 0, max_y = 0;
    int min_dist = 1000, max_dist = 0;
    for (int i = 0; i < dist.size(); ++i)
    {
        Vector2 vec(angles[i]);
        vec = vec * dist[i];
        min_x = std::min(min_x, vec.x);
        max_x = std::max(max_x, vec.x);
        min_y = std::min(min_y, vec.y);
        max_y = std::max(max_y, vec.y);
        min_dist = std::min(min_dist, dist[i]);
        max_dist = std::max(max_dist, dist[i]);
        ESP_LOGI("GP", "%d:  %d;%d  (%d;%d)", i, dist[i], angles[i], (int)vec.x, (int)vec.y);
    }

    ESP_LOGI("GP", "dist:  %d..%d", (int)min_dist, (int)max_dist);
    ESP_LOGI("GP", "dist offset:  x %d..%d y %d..%d", (int)min_x, (int)max_x, (int)min_y, (int)max_y);

    sensor.Cam.dist_offset_x -= (min_x + max_x) / 2;
    sensor.Cam.dist_offset_y -= (min_y + max_y) / 2;

    set_OpenMV_offset(sensor.Cam.dist_offset_x, sensor.Cam.dist_offset_y);
}

float pixel_dist_to_real(float x)
{
    return 3.91243e-6 * x * x * x * x - 0.000655948 * x * x * x + 0.05185 * x * x - 1.0362 * x + 52.2298;
}

bool getRayIntersection(float x1, float y1, float ang1, float x2, float y2, float ang2, float &out_x, float &out_y)
{
    float a1 = cos(ang1 * DEG_TO_RAD);
    float b1 = -sin(ang1 * DEG_TO_RAD);
    float c1 = -a1 * x1 - b1 * y1;

    float a2 = cos(ang2 * DEG_TO_RAD);
    float b2 = -sin(ang2 * DEG_TO_RAD);
    float c2 = -a2 * x2 - b2 * y2;

    float vp = a1 * b2 - a2 * b1;

    if (abs(vp) < 0.01)
        return false;

    out_x = -(c1 * b2 - c2 * b1) / vp;
    out_y = -(a1 * c2 - a2 * c1) / vp;

    if (abs(out_x) > 100 || abs(out_y) > 120 || (abs(out_x) < 1 && abs(out_y) > 99))
        return false;

    return true;
}

int getGlobalPosition_2gates(float &x, float &y, int color)
{
    int our_gate = sensor.Cam.GlobalYellow.center_angle;
    int other_gate = sensor.Cam.GlobalBlue.center_angle;
    if (color == 1)
        std::swap(our_gate, other_gate);
    menu.writeLineClean(0, "Angles " + std::to_string((int)(our_gate)) + " " + std::to_string((int)(other_gate)));
    menu.writeLineClean(1, "mpu " + std::to_string((int)(sensor.IMU.getYaw())));

    if (our_gate == 360 || other_gate == 360)
        return 1;

    float our_x = 0, our_y = -100;
    float other_x = 0, other_y = 100;

    // std::vector<float> xs = { -30,  30, -30,  30, 0, 0 };
    // std::vector<float> ys = { -100,-100, 100, 100, -100, 100 };
    // std::vector<float> angs = { (float)sensor.Cam.GlobalYellow.left_angle, (float)sensor.Cam.GlobalYellow.right_angle,
    //                             (float)sensor.Cam.GlobalBlue.left_angle,   (float)sensor.Cam.GlobalBlue.right_angle,
    //                             (float)sensor.Cam.GlobalYellow.center_angle,   (float)sensor.Cam.GlobalBlue.center_angle
    //                         };

    std::vector<float> xs = {0, 0};
    std::vector<float> ys = {-100, 100};
    std::vector<float> angs = {(float)sensor.Cam.GlobalYellow.center_angle, (float)sensor.Cam.GlobalBlue.center_angle};

    if (color == 1)
    {
        std::swap(angs[0], angs[2]);
        std::swap(angs[1], angs[3]);
        std::swap(angs[4], angs[5]);
    }

    std::vector<float> ans_x, ans_y;

    ESP_LOGI("GP", "Angles: %d, %d, %d, %d", (int)angs[0], (int)angs[2], (int)angs[1], (int)angs[3]);

    for (int i = 0; i < xs.size(); ++i)
    {
        for (int j = i + 1; j < xs.size(); ++j)
        {
            if (getRayIntersection(xs[i], ys[i], angs[i], xs[j], ys[j], angs[j], x, y))
            {
                ans_x.push_back(x);
                ans_y.push_back(y);
                ESP_LOGI("GP", "%d,%d:  %d;%d", i, j, (int)x, (int)y);
            }
        }
    }

    if (ans_x.empty())
        return 2;

    for (int i = 0; i <= ans_x.size() / 2; ++i)
    {
        int min_index = 0;
        for (int j = 0; j < ans_x.size(); ++j)
            if (ans_x[j] < ans_x[min_index])
                min_index = j;
        if (ans_x.size() % 2 == 0 && i == ans_x.size() / 2)
            x = (x + ans_x[min_index]) / 2;
        else
            x = ans_x[min_index];
        ans_x[min_index] = 1e9;
    }

    for (int i = 0; i <= ans_y.size() / 2; ++i)
    {
        int min_index = 0;
        for (int j = 0; j < ans_y.size(); ++j)
            if (ans_y[j] < ans_y[min_index])
                min_index = j;
        if (ans_y.size() % 2 == 0 && i == ans_y.size() / 2)
            y = (y + ans_y[min_index]) / 2;
        else
            y = ans_y[min_index];
        ans_y[min_index] = 1e9;
    }

    ESP_LOGI("GP", "median:  %d;%d", (int)x, (int)y);

    // x = -(c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1);
    // y = -110 + 220 * our_dist / (our_dist + other_dist); // -(a1 * c2 - a2 * c1) / (a1 * b2 - a2 * b1);

    return 0;
}

int getGlobalPosition_dist(float &x, float &y, int color)
{
    int our_gate = sensor.Cam.GlobalYellow.center_angle;
    int other_gate = sensor.Cam.GlobalBlue.center_angle;
    if (color == 1)
        std::swap(our_gate, other_gate);

    // if (our_gate == 360 || other_gate == 360)
    //     return 1;

    float our_x = 0, our_y = -100;
    float other_x = 0, other_y = 100;

    menu.writeLineClean(0, "Y dist " + std::to_string((int)(sensor.Cam.Yellow.distance)) + " " + std::to_string((int)(pixel_dist_to_real(sensor.Cam.Yellow.distance))));
    menu.writeLineClean(1, "Y dist " + std::to_string((int)(sensor.Cam.Blue.distance)) + " " + std::to_string((int)(pixel_dist_to_real(sensor.Cam.Blue.distance))));

    float real_dist_y = pixel_dist_to_real(sensor.Cam.Yellow.distance);
    float real_dist_b = pixel_dist_to_real(sensor.Cam.Blue.distance);

    if (sensor.Cam.Yellow.center_angle == 360)
    {
        x = -real_dist_b * sin(sensor.Cam.GlobalBlue.center_angle * DEG_TO_RAD);
        y = (color == 1 ? -100 : 100) - real_dist_b * cos(sensor.Cam.GlobalBlue.center_angle * DEG_TO_RAD);
    }
    else if (sensor.Cam.Blue.center_angle == 360)
    {
        x = -real_dist_y * sin(sensor.Cam.GlobalYellow.center_angle * DEG_TO_RAD);
        y = -100 - real_dist_y * cos(sensor.Cam.GlobalYellow.center_angle * DEG_TO_RAD);
    }

    return 0;
}

bool isBall()
{

    if (sensor.cfg.robotType == 2)
    {
        // return sensor.DribblerMicroswitch.ballCatched();
        sensor.LightGates.ballCatched();
        // return sensor.DribblerMicroswitch.ballCatched();
        return sensor.LightGates.ballCatched();
    }
    else
        return sensor.LightGates.ballCatched();// (sensor.Locator.getStrength() >= 100 && abs(sensor.Locator.getBallAngleLocal()) <= 10); //
}


bool paradox(int color)
{
    //return false;


    ballAngle = sensor.Locator.getBallAngleLocal();
    lineAngle = sensor.LineSensor.getAngleDelayed();
    int gateAngle = (int)sensor.Cam.gate(color).center_angle;
    int yaw = sensor.IMU.getYaw();
    int globalLine = (int)goodAngle(lineAngle + yaw);
    int globalBall = (int)goodAngle(ballAngle + yaw);
    int globalGate = (int)goodAngle(gateAngle + yaw);

    // print($"G: line {globalLine};  ball {globalBall};  gate {globalGate}");

    if (millis() - paradoxLastTime > kParadoxReset) {
        paradoxFirstTime = paradoxLastTime = millis();
        paradoxAngle = 360;
    }

    if (lineAngle != 360) {
        if (abs(goodAngle(globalLine - globalBall)) > 90 ||
            (globalGate * globalBall < 0 && gates_on_side(color)))
        {
            paradoxFirstTime = paradoxLastTime = millis();
            paradoxAngle = globalLine;
        } else {
            paradoxLastTime = millis();
        }

        paradoxAngle = globalLine;

        return false;
    } else {
        if (paradoxLastTime - paradoxFirstTime > kParadoxStartTime) {
            if (abs(paradoxAngle) >= 45 && abs(paradoxAngle) <= 135) {
                bool our_gate_closer = true;
                if (sensor.Cam.gate(color).distance < sensor.Cam.gate(color ^ 1).distance) {
                    our_gate_closer = false;
                }
                if (our_gate_closer) {
                    drv.drive(-yaw, (int)deltaAngle, 50);
                    // Debug.Log($"Paradox up");
                } else {
                    drv.drive(-yaw + 180, (int)deltaAngle, 50);
                    // Debug.Log($"Paradox down");
                }
            } else {
                if (globalBall < 0) {
                    drv.drive(-yaw - 90, (int)deltaAngle, 50);
                    // Debug.Log($"Paradox left");
                } else {
                    drv.drive(-yaw + 90, (int)deltaAngle, 50);
                    // Debug.Log($"Paradox right");
                }
            }
        } else {
            return false;
        }
    }
    return true;
}

// #define OTLADKA_petrovich
#ifdef OTLADKA_petrovich
    const char* petrovich = "p";
#endif

void petrovich_iter(int color, int offset = 0, bool useLine = true)
{
    sensor.update();

    ballAngle = sensor.Locator.getBallAngleLocal();
    
    // int gateAngle = (int)sensor.Cam.gate(color).center_angle;
    int gateAngle = -sensor.IMU.getYaw();
    lineAngle = (useLine ? sensor.LineSensor.getAngleDelayed() : 360);

    int offset_angle = (int)goodAngle(gateAngle - offset);

    menu.writeLineClean(1, std::to_string(gateAngle) + " " + std::to_string(ballAngle) + " ");
    // menu.writeLineClean(2, "");
    // menu.writeLineClean(3, "");

    if (!paradox(color)) {
        deltaAngle = goodAngle(offset_angle) * 0.25f;
        // menu.writeLineClean(2, std::to_string(deltaAngle) + " ");
        if (lineAngle != 360) {
            // menu.writeLineClean(2, "LINE!!! " + std::to_string(lineAngle));
            drv.drive(goodAngle(lineAngle + 180), (int)deltaAngle, 50);
            #ifdef OTLADKA_petrovich
                    ESP_LOGI(petrovich, "LINE!!!");
            #endif
        }
        else {
            int angle_err = goodAngle(ballAngle - gateAngle);
            static bool gate_in_front;
            gate_in_front = gate_in_front || (abs(angle_err) <= 15); // включение атаки
            gate_in_front = gate_in_front && (abs(angle_err) <= 15); // условие продолжения атаки

            ESP_LOGI("gates", "%f", moveAngle);
            if ((gate_in_front || isBall()/* && abs(goodAngle(ballAngle - offset)) <= 10*/)
                )
            {
                #ifdef OTLADKA_petrovich
                    ESP_LOGI(petrovich, "drive to gate");
                #endif
                // menu.writeLineClean(2, "V");
                moveAngle = gateAngle;
                drv.drive(moveAngle, (int)deltaAngle, 80);

                if (sensor.Cam.gate(color).distance < 50 && isBall()) {
                    kicker.kick();
                }
            }
            else {
                #ifdef OTLADKA_petrovich
                    ESP_LOGI(petrovich, "drive to ball");
                #endif
                moveAngle = ballAngle;
                if (angle_err > 0) {
                    moveAngle = goodAngle(ballAngle + constrain(sensor.Locator.getStrength() * goRoundBallCoefFw, 0, 90));
                }
                else {
                    moveAngle = goodAngle(ballAngle - constrain(sensor.Locator.getStrength() * goRoundBallCoefFw, 0, 90));
                }

                int speed = 80;
                if (abs(ballAngle) < 40){
                    speed = 40;
                }
                drv.drive(moveAngle, (int)deltaAngle, speed);
            }
        }
    }
    else{
        #ifdef OTLADKA_petrovich
            ESP_LOGI(petrovich, "PARADOX");
        #endif
    }
}

// функция, чтобы ехать в опр. направлении, объезжая препятствия
void goOverObstacleOmni(float generalSpeed, float generalAngle, float rot_angle, float critDist, bool rotate = false)
{
    // стартовые параметры для drive
    moveAngle = generalAngle;
    deltaAngle = rot_angle;

    //if (Mathf.Abs(deltaAngle) > maxTorqueWithBall)
    //{
    //    deltaAngle = (deltaAngle > 0) ? maxTorqueWithBall : -maxTorqueWithBall;
    //}

    // если рядом препятствие
    if (sensor.Cam.obst_dist < critDist)
    {
        menu.writeLineClean(0, "obst: " + std::to_string((int)-sensor.Cam.obst_angle));
        float co_angle = goodAngle(-sensor.Cam.obst_angle - generalAngle); // разность углов ворот и препятствия
        if (rotate)                                                        // если нужно отворачиваться от препятствия, то отворачиваемся
        {
            if (co_angle > 0 && co_angle < criticalObstacleAngle)
            {
                deltaAngle = -0.15 * goodAngle(180 - -sensor.Cam.obst_angle);
            }
            if (co_angle < 0 && co_angle > -criticalObstacleAngle)
            {
                deltaAngle = -0.15 * goodAngle(180 + -sensor.Cam.obst_angle);
            }
        }
        menu.writeLineClean(1, "co_angle: " + std::to_string((int)co_angle));

        if (abs(co_angle) < criticalObstacleAngle) // если препятствие не сзади
        {
            // проецируем скорость поперёк препятствия

            if (co_angle > 10)
            {
                moveAngle = goodAngle(-sensor.Cam.obst_angle - 90);
            }
            else if (co_angle < -10)
            {
                moveAngle = goodAngle(-sensor.Cam.obst_angle + 90);
            }
            else
            {
                moveAngle = goodAngle(-sensor.Cam.obst_angle + 120);
            }
        }
    }
    else
    {
        menu.writeLineClean(0, "");
        menu.writeLineClean(1, "");
    }

    menu.writeLineClean(2, "moveAngle: " + std::to_string((int)moveAngle));
    menu.writeLineClean(3, "deltaAngle: " + std::to_string((int)deltaAngle));

    //moveAngle = limitSpeed(moveAngle, generalSpeed, global_x, global_y);
    drv.drive((int)moveAngle, (int)deltaAngle, (int)generalSpeed);
}

void killerFeature(int color)
{
    while (true)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);

        if (stateGame != 1)
        {
            break;
            // playGoalkeeperCamera(1 ^ color);
        }
        if (millis() - kfTimer >= maxKfTime)
        {
            stateGame = 0;
            ballMoveTime = millis();
            break;
        }

        sensor.update();

        ballAngle = sensor.Locator.getBallAngleLocal();
        int robotAngle = sensor.IMU.getYaw();
        int gateAngle = sensor.Cam.gate(color).center_angle;

        if (gateAngle != 360)
        {
            gateAngle = (int)goodAngle(gateAngle /*+ 180*/);
        }
        else
        {
            drv.driveXY(0, 0, 20);
            continue;
        }

        ESP_LOGI("killerFeature", "gateAngle: %d", gateAngle);

        petrovich_iter(color, 0);
    }
}

void playGoalkeeperCamera(int color)
{
    constexpr int64_t goalHeadingTimeoutUs = 350'000;
    constexpr int64_t logIntervalUs = 50'000;
    HeadingRegulator heading;
    int64_t lastFrameUs = sensor.Cam.lastFrameTimeUs();
    int64_t lastGoalUs = 0;
    int64_t lastLogUs = 0;
    int goalHeading = 0;

    menu.clearDisplay();
    while (true)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        sensor.update();

        const int yaw = sensor.IMU.getYaw();
        const auto &goal = sensor.Cam.gate(color);
        const int64_t frameUs = sensor.Cam.lastFrameTimeUs();
        const int64_t nowUs = esp_timer_get_time();
        const bool goalVisible = goal.center_angle != 360;
        if (frameUs != lastFrameUs)
        {
            lastFrameUs = frameUs;
            if (goalVisible)
            {
                goalHeading = goodAngle(yaw + goal.center_angle);
                heading.setTarget(goalHeading);
                lastGoalUs = frameUs;
            }
        }

        const int ballStrength = sensor.Locator.getStrength();
        const bool haveBall = ballStrength >= 2;
        const bool targetValid = lastGoalUs != 0 && nowUs - lastGoalUs <= goalHeadingTimeoutUs;
        const auto logDrive = [&](const char *mode, int angle, int speed) {
            if (nowUs - lastLogUs < logIntervalUs)
                return;
            lastLogUs = nowUs;

            const float angleRad = angle * DEG_TO_RAD;
            const int x = std::lround(speed * std::sin(angleRad));
            const int y = std::lround(speed * std::cos(angleRad));
            ESP_LOGI("Keeper",
                     "mode=%s, px=%d rate=%.1f move_angl=%d speed=%d x=%d y=%d ball_str=%d",
                     mode, goal.distance, targetValid ? heading.getAngularRate() : 0.0f,
                     angle, speed, x, y, ballStrength);
        };

        if (haveBall && stateGame != 0)
        {
            ESP_LOGI("Keeper", "event=attack_start yaw=%d ball_str=%d", yaw, ballStrength);
            kfTimer = millis();
            killerFeature(1 ^ color);
            heading.reset();
            lastGoalUs = 0;
            lastFrameUs = sensor.Cam.lastFrameTimeUs();
            continue;
        }

        if (!targetValid)
        {
            if (lastGoalUs != 0)
            {
                heading.reset();
                lastGoalUs = 0;
            }
            const int searchRotation = haveBall ? 20 : -10;
            drv.drive(0, searchRotation, 0);
            logDrive("search", 0, 0);
            continue;
        }

        const int rotation = heading.update(yaw, nowUs);

        // drv.drive(0, rotation, 0);
        // continue;

        if (!haveBall || !goalVisible)
        {
            drv.drive(0, rotation, 0);
            logDrive(!haveBall ? "no_ball" : "hold_goal", 0, 0);
            continue;
        }

        // Use the same filtered world heading for translation and nose alignment.
        const float globalGoalAngle = std::remainder(heading.getTargetAngle(), 360.0f);
        const float goalRearHeading = std::remainder(globalGoalAngle + 180.0f, 360.0f);
        const int localBallAngle = sensor.Locator.getBallAngleLocal();
        const int globalBallAngle = goodAngle(localBallAngle + yaw);
        ballAngle = goodAngle(localBallAngle + 180);
        lineAngle = sensor.LineSensor.getAngleDelayed();

            // lineAngle = 360;

        if (lineAngle != 360)
            lineAngle = goodAngle(lineAngle + 180);

        ballSeenTime = millis();
        if (abs(goodAngle(globalBallAngle - lastBallAngle)) > ballNoMotionDiap ||
            abs(ballStrength - lastMoveBallStrength) > ballNoMotionStrength)
        {
            lastBallAngle = globalBallAngle;
            lastMoveBallStrength = ballStrength;
            ballMoveTime = millis();
        }

        if (millis() - ballMoveTime >= 5000)
        {
            drv.driveXY(0, -70, 0);
            logDrive("attack_prepare", 180, 70);
            make_pause(100);
            stateGame = 1;
            continue;
        }

        if (lineAngle != 360 && std::abs(globalGoalAngle) <= 135 &&
            ((goalRearHeading > 0 && lineAngle > 0) || (goalRearHeading < 0 && lineAngle < 0)))
        {
            const int driveAngle = lineAngle;
            drv.drive(driveAngle, rotation, 100);
            // kicker.kick(500);
            logDrive("line", driveAngle, 100);
            continue;
        }

        speedX = 0;
        speedY = 0;
        const int camDist = goal.distance;
        const float absGoalAngle = std::abs(globalGoalAngle);
        const bool gateCorner = absGoalAngle < 125;
        if (lineAngle != 360 && absGoalAngle > 160){
            kicker.kick(1000);
            speedY = 40;
        }
        else if (!gateCorner || true)
        {
            if (camDist < 19)
                speedY = 40;
            // else if (camDist < 22)
            //     speedY = 40;
            else if (camDist < 21)
                speedY = 20;
            else if (camDist < 23)
                speedY = 0;
            else
                speedY = -(camDist - 23) * 5;
        }
        else
        {
            if (camDist < 10)
                speedY = 40;
            else if (camDist < 12)
                speedY = 20;
            else if (camDist < 15)
                speedY = 0;
            else
                speedY = -(camDist - 14) * 7;
        }
        speedY = constrain(speedY, -limitGateSpeed, 100);

                            if (abs(ballAngle) > 80 && speedY <= 0)
                            {
                                const int offset = constrain(ballStrength * goRoundBallCoefGk, 0, 90);
                                moveAngle = goodAngle(ballAngle + (ballAngle > 0 ? offset : -offset));
                                const int driveAngle = goodAngle(moveAngle + 180);
                                drv.drive(driveAngle, rotation, 50);
                                logDrive("orbit", driveAngle, 50);
                                continue;
                            }

        const float goalRelativeBallAngle = std::remainder(globalBallAngle - goalRearHeading, 360.0f);
        const float maxBallError = maxGoalkeeperAngle;
        const float ballError = std::clamp(goalRelativeBallAngle, -maxBallError, maxBallError);
        const float ballSpeed = gb_kp * ballError + gkBallIntegral * gb_ki +
                                gb_kd * (ballError - gkBallPrev);

        static int prevIntegralTime = 0;
        const int nowMs = millis();
        gkBallIntegral += ballError * (nowMs - prevIntegralTime);
        gkBallPrev = ballError;
        gkBallIntegral2 += ballError * (nowMs - prevIntegralTime);
        gkBallPrev2 = ballError;
        prevIntegralTime = nowMs;

                        speedX += ballSpeed;

        // speedX = 0;

        const int translationSpeed = constrain(sqrt(speedX * speedX + speedY * speedY), 0, 100);
        const float fieldMoveAngle = atan2(speedX, speedY) * RAD_TO_DEG;
        const int bodyMoveAngle = goodAngle(fieldMoveAngle - yaw);
        drv.drive(bodyMoveAngle, rotation, translationSpeed);
        logDrive("track", bodyMoveAngle, translationSpeed);

        // Do not kick into our own goal when facing it.
        if (isBall() && abs(goodAngle(goalHeading - yaw)) > 90)
            kicker.kick();
    }
}

void playForwardGoyda(int color)
{
    color ^= 1;
    menu.clearDisplay();
    while (true)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);

        sensor.update();

        if (sensor.Locator.getStrength() < 2)
        {
            drv.drive(0, 30, 0);
            // dribbler.smart_dribble(0);
            //return;
            continue;
        }

        int cam_angle = sensor.Cam.gate(color).center_angle;
        int cam_dist = sensor.Cam.gate(color).distance;

        int st = sensor.Locator.getStrength();
        int ballAngle = sensor.Locator.getBallAngleLocal();
        int lineAngle = sensor.LineSensor.getAngleDelayed();

        int robotAngle = sensor.IMU.getYaw();

        int deltaAngle = sensor.Cam.gate(color).center_angle * 0.3;
        
        petrovich_iter(color, 0);
    }
}

void goalRotate(int color)
{
    int sign = (sensor.IMU.getYaw() < 0) ? -1 : 1;
    //menu.clearDisplay();
    while (isBall())
    {
        //menu.writeLineClean(0, "GoalRotate");
        sensor.update();
        int rotateSpeed = abs(sensor.Cam.gate(color).center_angle) > 150 ? 30 : 80;        // 42 : 100 // > 150 ? 30 : 90
        dribbler.smart_dribble(abs(sensor.Cam.gate(color).center_angle) > 130 ? 80 : 40); // 110 // > 120 ? 110 : 60
        drv.drive(0, rotateSpeed * sign, 0);

        if (abs(sensor.Cam.gate(color).center_angle) < 20){
            dribbler.smart_dribble(0);
            return;
        }

        // if (omnicam().gates[color].center_angle > 0){
        //     drive(0, -40, 0);
        // }
        // else {
        //     drive(0, 40, 0);
        // }
        // Debug.SendInfo();
    }
    //menu.writeLineClean(0, "END GoalRotate");
}

void goalDriveBack(int color)
{
    dribbler.smart_dribble(0);
    while (isBall())
    {
        sensor.update();
        drv.driveXY(0, -100, 0);
    }

    ballAngle = sensor.Locator.getBallAngleLocal();
    int sign = (sensor.IMU.getYaw() < 0) ? -1 : 1;
    while (abs(ballAngle) < 40 && !isBall())
    {
        sensor.update();
        ballAngle = sensor.Locator.getBallAngleLocal();
        drv.driveXY(60 * sign, 0, 0);
    }
}

void goalPush(int color)
{
    while (true)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        sensor.update();
        // dribbler.smart_dribble(0);
        if (sensor.Cam.gate(color).width < 50){
            menu.writeLineClean(0, "exit goalPush");
            break;
        }

        dribbler.smart_dribble(-10);
        menu.writeLineClean(0, "goalPush");

        menu.writeLineClean(1, "w: " + std::to_string(sensor.Cam.gate(color).width));
        ballAngle = sensor.Locator.getBallAngleLocal();
        
        int gateAngle = (int)sensor.Cam.gate(color).center_angle;
        if (gateAngle == 360)
            break;
        lineAngle = sensor.LineSensor.getAngleDelayed();
        
        int offset = 180; //(gateAngle > 0 ? 90 : -90);
        petrovich_iter(color, offset);
        continue;
    }
}

bool isBallInFront(int angle_err)
{
    return abs(angle_err) <= 15;
}

float FwBallAnglIntegral = 0;
float FwBallAnglPrev = 0;
float Fw_kd_if_front = 3.5; //3.0
// float Fw_ki = 0;
float Fw_kp_if_front = 0.35;

float Fw_kp_not_in_front_of = 0.7; //0.35

//#define OTLADKA_Dribble2
#ifdef OTLADKA_Dribble2
    const char* drible2 = "f";
#endif
void playForwardDribble2(int color)
{
    srand(millis());
    menu.clearDisplay();
    color = 1 ^ color;
    
    while (true)
    {
    fwDribbleBegin:

        #ifdef OTLADKA_Dribble2
            ESP_LOGI(drible2, "-NEW ITERATION-");
        #endif
        vTaskDelay(10 / portTICK_PERIOD_MS);

        sensor.update();

        int cam_angle = sensor.Cam.gate(color).center_angle;
        int cam_dist = sensor.Cam.gate(color).distance;
        

        int st = sensor.Locator.getStrength();
        int ballAngle = sensor.Locator.getBallAngleLocal();
        static bool ball_is_too_futher = false;
        if (sensor.Locator.getStrength() < 2)
        {
            //ESP_LOGW(drible2, "Ne vizhu miach");
            drv.drive(0, 20, 0);
            ball_is_too_futher = true;
            continue;
        }
        int lineAngle = sensor.LineSensor.getAngleDelayed();

        // menu.writeLineClean(1, "w: " + std::to_string(sensor.Cam.gate(color).width));

        // if (sensor.Cam.gate(color).width >= 30)
        //    goalPush(color);

        if (!isBall())
        {
            //menu.writeLineClean(0, "ball");

            dribbler.smart_dribble((abs(ballAngle) < 60) ? 75 : 0);

            if (!paradox(color)) {
                int robotAnglSpeed;
                int speed;
                moveAngle = ballAngle;

                // if(ball_is_too_futher){
                //     if(sensor.Locator.getStrength() < 35)
                //         ballAngle = goodAngle(ballAngle + 90);
                //     else
                //         ball_is_too_futher = false;
                // }

                if (lineAngle == 360)
                {
                    #ifdef OTLADKA_Dribble2
                        ESP_LOGW(drible2, "Mach poteryan, edu k machu");
                    #endif
                    
                    float ballAnfl_err = ballAngle;
                    if(isBallInFront(ballAnfl_err))
                    {
                        robotAnglSpeed = Fw_kp_if_front * ballAnfl_err;
                    }
                    else
                    {
                        robotAnglSpeed = Fw_kp_not_in_front_of * ballAnfl_err + Fw_kd_if_front * (ballAnfl_err - FwBallAnglPrev);
                    }
                    // 
                    // 
                    // FwBallAnglIntegral += (ballAnfl_err)*Fw_ki;
                    FwBallAnglPrev = ballAnfl_err;

                    speed = sensor.Locator.getStrength() < 65 ? 100 : 30;  //70 
                }
                else
                {
                    #ifdef OTLADKA_Dribble2
                        ESP_LOGW(drible2, "Mach poteryan, edu k machu no vizhu liniju");
                    #endif
                    moveAngle = goodAngle(lineAngle + 180);
                    robotAnglSpeed = ballAngle * 0.3;
                    speed = 80;
                }
                drv.drive(moveAngle, (int)(robotAnglSpeed), speed);
            }
        }
        else
        {
            #ifdef OTLADKA_Dribble2  
                ESP_LOGW(drible2, "Mach zachvachen, podjzzaju");
            #endif

            if (abs(cam_angle) < 30 && cam_dist <= 50){
                dribbler.smart_dribble(0);
                make_pause(100);
                kicker.kick();
                goto fwDribbleBegin;
            }


            //menu.writeLineClean(0, "gate...");
            dribbler.smart_dribble(180);
            drv.driveXY(0, 20, 0); // чтобы лучше задриблить 
            int start = millis();
            while (millis() - start < 500)
            {
                sensor.update();
                lineAngle = sensor.LineSensor.getAngleDelayed();
                if (lineAngle != 360){
                    drv.drive(goodAngle(lineAngle + 180), 0, 80);
                    break;
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            // drv.drive(0, 0, 0, 0);
            make_pause(500);
            sensor.BallSensor.update();
            while (isBall())
            {
                sensor.update();

                cam_angle = sensor.Cam.gate(color).center_angle;
                cam_dist = sensor.Cam.gate(color).distance;
                lineAngle = sensor.LineSensor.getAngleDelayed();
                // dribbler.smart_dribble((abs(cam_angle) > 10 || cam_dist > 50) ? 50 : 0);

                if (lineAngle == 360)
                {
                    #ifdef OTLADKA_Dribble2  
                        ESP_LOGW(drible2, "Mach zachvachen, povorachivajus 1: %d", abs(cam_angle) < 130 && isBall());
                    #endif
                    while (abs(cam_angle) < 130 && isBall())
                    {
                        sensor.update();
                        lineAngle = sensor.LineSensor.getAngleDelayed();
                        moveAngle = goodAngle(lineAngle + 180);
                        speed = (lineAngle == 360) ? 0 : 50;
                        cam_angle = sensor.Cam.gate(color).center_angle;
                        deltaAngle = goodAngle(cam_angle + 180);
                        deltaAngle = abs(cam_angle) < 90 ? deltaAngle * 0.3 : deltaAngle * 0.5; //((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                        deltaAngle = constrain(deltaAngle, -10, 10);
                        drv.drive(moveAngle, (int)(deltaAngle), speed);
                    }
                    #ifdef OTLADKA_Dribble2  
                        ESP_LOGW(drible2, "POVERNUL 1");
                    #endif

                    if (!isBall())
                    {
                        #ifdef OTLADKA_Dribble2  
                            ESP_LOGW(drible2, "POTERIALY MATCH");
                        #endif
                        goto fwDribbleBegin;
                    }
                    // make_pause(300);

                    sensor.update();

                    int delta_angle = goodAngle(cam_angle + 180) * 0.3; //((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                    delta_angle = constrain(delta_angle, -10, 10);

                    lineAngle = sensor.LineSensor.getAngleDelayed();   
                    #ifdef OTLADKA_Dribble2  
                        ESP_LOGI(drible2, "gate: %d, dist: %d, line: %d", cam_angle, cam_dist, lineAngle);
                    #endif
                    if (lineAngle != 360)
                    {
                        #ifdef OTLADKA_Dribble2  
                            ESP_LOGW(drible2, "edu k vorotam NO LINIJA");
                        #endif
                        drv.drive(goodAngle(lineAngle + 180), delta_angle, 100);
                        continue;
                    }
                    else{
                        #ifdef OTLADKA_Dribble2  
                            ESP_LOGW(drible2, "edu k vorotam");
                        #endif
                        
                        drv.drive(cam_angle, delta_angle, 45);
                    }
                    cam_dist = sensor.Cam.gate(color).distance;
                    // Serial.println(cam_dist);
                    if (abs(cam_angle) > 100 && cam_angle != 360 && cam_dist <= 30)
                    {
                        #ifdef OTLADKA_Dribble2                      
                            ESP_LOGW(drible2, "---ATAKUJU---");
                        #endif
                        // return;
                        drv.drive(0, 0, 0, 0);
                        sensor.update();
                        dribbler.smart_dribble(180);

                        cam_angle = sensor.Cam.gate(color).center_angle;

                        while (abs(cam_angle) < 160 && isBall())
                        {
                            sensor.update();
                            cam_angle = sensor.Cam.gate(color).center_angle;
                            deltaAngle = goodAngle(cam_angle + 180); //((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                            deltaAngle = constrain(deltaAngle, -40, 40);
                            drv.drive(0, (int)(deltaAngle * 0.3), 0);
                        }

                        cam_angle = sensor.Cam.gate(color).center_angle;
                        cam_dist = sensor.Cam.gate(color).distance;

                        if (!isBall())
                        {
                            goto fwDribbleBegin;
                        }

                        // может это в goalDriveBack?
                        dribbler.smart_dribble(180);
                        //make_pause(500);

                        // if (abs(sensor.IMU.getYaw()) < 135 || cam_dist < 30)

                        MPU_zakrut(color);
                        // else
                        //goalDriveBack(color);

                        goto fwDribbleBegin;
                    }
                }
                else
                {
                    drv.drive(goodAngle(lineAngle + 180), 0, 80);
                }
            }
        }
    }
}

namespace
{
bool isGateVisible(const OmniCamBlobInfo_t &gate)
{
    return gate.center_angle != 360 && gate.left_angle != 360 &&
           gate.right_angle != 360 && gate.width > 0;
}
} // namespace

bool isGateCornerForKicking(const OmniCamBlobInfo_t &gate)
{
    return std::abs(goodAngle(gate.center_angle + sensor.IMU.getYaw())) > 30.0f;
}

bool gateIsCloseAndFree(int color)
{
    const auto& gate = sensor.Cam.gate(color);
    if (!isGateVisible(gate) || gate.distance < 0 || gate.distance > 70)
        return false;

    return gate.width > (isGateCornerForKicking(gate) ? 20 : 40);
}

int AngleErrorToRange(int cur, int from, int to, int perfect)
{
    // cur - текущий угол удара
    // [from, to] - диапазон, в который допустимо бить
    // perfect - точка, в которую идеально бить
    // (все это локальные углы, + по часовой)

    int left_error = goodAngle(from - cur);
    int right_error = goodAngle(to - cur);
    if (left_error <= 0.0f && right_error >= 0.0f)
        return goodAngle(perfect - cur);

    return std::abs(left_error) < std::abs(right_error) ? left_error : right_error;
}

int AccurateRotationToKick(const OmniCamBlobInfo_t &gate)
{
    if (gate.left_angle > 0 || gate.right_angle < 0)
        return gate.center_angle;

    // в углу поля другое поведение, чтобы не попасть в штангу
    bool gate_corner = isGateCornerForKicking(gate);

    // минимальный угол до края области, чтобы попасть в неё с достаточной точностью
    // увеличить, если промахивается (но не получится бить в узкие окна)
    const int min_offset = gate_corner ? 10 : 10; //5
    
    // перекос в сторону дальнего угла
    // увеличить, если при ударе сбоку часто попадает в ближнюю штангу
    int asymmetric_offset = gate_corner
        ? std::min(15, gate.width / 2)
        : 0;

    int global_gate_angle = goodAngle(gate.center_angle + sensor.IMU.getYaw());

    int left_offset = min_offset;
    int right_offset = min_offset;
    if (global_gate_angle > 0)
        left_offset += asymmetric_offset;
    else
        right_offset += asymmetric_offset;

    if (gate.width >= left_offset + right_offset)
    {
        return AngleErrorToRange(
            0,
            gate.left_angle + left_offset,
            gate.right_angle - right_offset,
            gate.center_angle);
    }

    int left_limit, right_limit, perfect_angle;
    if (global_gate_angle > 0)
    {
        right_limit = gate.right_angle - min_offset;
        if (gate.width >= left_offset + min_offset) {
            left_limit = gate.left_angle + left_offset;
        } else if (gate.width >= min_offset + min_offset) {
            left_limit = gate.left_angle + min_offset;
        } else {
            left_limit = right_limit;
        }
        
        perfect_angle = right_limit;
    } else {
        left_limit = gate.left_angle + min_offset;
        if (gate.width >= min_offset + right_offset) {
            right_limit = gate.right_angle - right_offset;
        } else if (gate.width >= min_offset + min_offset) {
            right_limit = gate.right_angle - min_offset;
        } else {
            right_limit = left_limit;
        }
        
        perfect_angle = left_limit;
    }

    return AngleErrorToRange(0, left_limit, right_limit, perfect_angle);
}

int AngleErrorToRotationSpeed(int err)
{
    // скорость поворота (сюда можно и поумнее регулятор)
    int rotation_speed = err * 0.2f;
    if (std::abs(rotation_speed) >= 15.0f)
        return static_cast<int>(rotation_speed);
    if (rotation_speed == 0.0f)
        return 0;
    return rotation_speed < 0 ? -10 : 10;
}

void goalKicker(int color)
{
    const int time_guard = millis();
    const int kTimeLimit = 2000;
    const int kAcceptableAngleErr = 5; // можно и поменьше мб
    while (true)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        sensor.update();

        const auto& gate = sensor.Cam.gate(color);
        if (!isBall()) { break; }
        if (sensor.LineSensor.getAngleDelayed() != 360) { break; }
        if (millis() - time_guard > kTimeLimit) { break; }

        if (!isGateVisible(gate)) { continue; }

        int angle_err = AccurateRotationToKick(gate);
        if (std::abs(angle_err) <= kAcceptableAngleErr)
        {
            drv.drive(0, 0, 0, 0);
            kicker.kick();
            return;
        }

        deltaAngle = constrain(AngleErrorToRotationSpeed(angle_err), -20, 20);
        drv.drive(0, static_cast<int>(deltaAngle), 0);
    }
    drv.drive(0, 0, 0, 0);
}

#define OTLADKA_Kicker
#ifdef OTLADKA_Kicker
    const char* kick_log = "k";
#endif
void playForwardKicker(int our_color)
{
    srand(millis());
    menu.clearDisplay();
    int color = 1 ^ our_color;
    
    while (true)
    {
    fwKickerBegin:

        #ifdef OTLADKA_Kicker
            ESP_LOGI(kick_log, "-NEW ITERATION-");
        #endif
        vTaskDelay(10 / portTICK_PERIOD_MS);

        sensor.update();

        int cam_angle = sensor.Cam.gate(color).center_angle;
        int cam_dist = sensor.Cam.gate(color).distance;

        int st = sensor.Locator.getStrength();
        int ballAngle = sensor.Locator.getBallAngleLocal();
        static bool ball_is_too_far = false;
        if (sensor.Locator.getStrength() < 5)
        {
            ESP_LOGW(kick_log, "Ne vizhu miach");
            drv.drive(0, 20, 0);
            ball_is_too_far = true;
            continue;
        }
        int lineAngle = sensor.LineSensor.getAngleDelayed();

        if (!isBall())
        {
            //menu.writeLineClean(0, "ball");

            dribbler.smart_dribble((abs(ballAngle) < 60) ? 30 : 0);

            if (!paradox(color)) {
                int robotAnglSpeed;
                int speed;
                moveAngle = ballAngle;

                if(ball_is_too_far){
                    if(sensor.Locator.getStrength() < 35)
                        ballAngle = goodAngle(ballAngle + 90);
                    else
                        ball_is_too_far = false;
                }

                if (lineAngle == 360)
                {
                    #ifdef OTLADKA_Kicker
                        ESP_LOGW(kick_log, "Mach poteryan, edu k machu");
                    #endif
                    
                    int ballAnfl_err = ballAngle;
                    // 
                    if(isBallInFront(ballAnfl_err))
                    {
                        robotAnglSpeed = Fw_kp_if_front * ballAnfl_err;
                    }
                    else
                    {
                        robotAnglSpeed = Fw_kp_not_in_front_of * ballAnfl_err + Fw_kd_if_front * (ballAnfl_err - FwBallAnglPrev);
                    }
                    // 
                    FwBallAnglPrev = ballAnfl_err;

                    speed = sensor.Locator.getStrength() < 65 ? 100 : 30;  //70 
                }
                else
                {
                    #ifdef OTLADKA_Kicker
                        ESP_LOGW(kick_log, "Mach poteryan, edu k machu no vizhu liniju");
                    #endif
                    moveAngle = goodAngle(lineAngle + 180);
                    robotAnglSpeed = ballAngle * 0.3;
                    speed = 80;
                }
                drv.drive(moveAngle, (int)(robotAnglSpeed), speed);
            }
        }
        else
        {
            #ifdef OTLADKA_Kicker  
                ESP_LOGW(kick_log, "Mach zachvachen, podjzzaju");
            #endif
            //menu.writeLineClean(0, "gate...");
            dribbler.smart_dribble(30);
            drv.driveXY(0, 20, 0); // чтобы лучше задриблить 
            int start = millis();
            while (millis() - start < 300)
            {
                sensor.update();
                lineAngle = sensor.LineSensor.getAngleDelayed();
                if (lineAngle != 360){
                    drv.drive(goodAngle(lineAngle + 180), 0, 80);
                    break;
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            // drv.drive(0, 0, 0, 0);
            make_pause(200);
            sensor.BallSensor.update();
            while (isBall())
            {
                //menu.writeLineClean(0, "gate");
                vTaskDelay(10 / portTICK_PERIOD_MS);
                sensor.update();

                // дополнительный переход чтобы не поворачиваться сначала спиной, а потом снова лицом
                if (gateIsCloseAndFree(color)) {
                    #ifdef OTLADKA_Kicker  
                        ESP_LOGW(kick_log, "Kicker attack");
                    #endif

                    goalKicker(color);
                    goto fwKickerBegin;
                }

                cam_angle = sensor.Cam.gate(color).center_angle;
                cam_dist = sensor.Cam.gate(color).distance;
                lineAngle = sensor.LineSensor.getAngleDelayed();
                // dribbler.smart_dribble((abs(cam_angle) > 10 || cam_dist > 50) ? 50 : 0);

                if (lineAngle == 360)
                {
                    #ifdef OTLADKA_Kicker  
                        ESP_LOGW(kick_log, "Mach zachvachen, povorachivajus 1: %d", abs(cam_angle) < 130 && isBall());
                    #endif
                    while (abs(cam_angle) < 130 && isBall())
                    {
                        sensor.update();
                        lineAngle = sensor.LineSensor.getAngleDelayed();
                        moveAngle = goodAngle(lineAngle + 180);
                        speed = (lineAngle == 360) ? 0 : 60;
                        cam_angle = sensor.Cam.gate(color).center_angle;
                        deltaAngle = goodAngle(cam_angle + 180);
                        deltaAngle = abs(cam_angle) < 90 ? deltaAngle * 0.3 : deltaAngle * 0.5; //((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                        deltaAngle = constrain(deltaAngle, -10, 10);
                        drv.drive(moveAngle, (int)(deltaAngle), speed);
                    }
                    #ifdef OTLADKA_Kicker  
                        ESP_LOGW(kick_log, "POVERNUL 1");
                    #endif

                    if (!isBall())
                    {
                        #ifdef OTLADKA_Kicker  
                            ESP_LOGW(kick_log, "POTERIALY MATCH");
                        #endif
                        goto fwKickerBegin;
                    }

                    sensor.update();

                    int delta_angle = goodAngle(cam_angle + 180) * 0.3; //((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                    delta_angle = constrain(delta_angle, -10, 10);

                    lineAngle = sensor.LineSensor.getAngleDelayed();
                    if (lineAngle != 360)
                    {
                        #ifdef OTLADKA_Kicker  
                            ESP_LOGW(kick_log, "edu k vorotam NO LINIJA");
                        #endif
                        drv.drive(goodAngle(lineAngle + 180), delta_angle, 100);
                        continue;
                    }
                    else{
                        #ifdef OTLADKA_Kicker  
                            ESP_LOGW(kick_log, "edu k vorotam");
                        #endif
                        
                        drv.drive(cam_angle, delta_angle, 75);
                    }
                    cam_dist = sensor.Cam.gate(color).distance;
                    // Serial.println(cam_dist);
                    if (abs(cam_angle) > 100 && cam_angle != 360 && cam_dist <= 30)
                    {
                        #ifdef OTLADKA_Kicker                      
                            ESP_LOGW(kick_log, "---ATAKUJU---");
                        #endif

                        if (gateIsCloseAndFree(color)) {
                            #ifdef OTLADKA_Kicker  
                                ESP_LOGW(kick_log, "Kicker attack");
                            #endif
                            goalKicker(color);
                            goto fwKickerBegin;
                        }

                        // return;
                        drv.drive(0, 0, 0, 0);
                        sensor.update();
                        dribbler.smart_dribble(30);

                        make_pause(100);

                        cam_angle = sensor.Cam.gate(color).center_angle;

                        while (abs(cam_angle) < 160 && isBall())
                        {
                            sensor.update();
                            cam_angle = sensor.Cam.gate(color).center_angle;
                            deltaAngle = goodAngle(cam_angle + 180);
                            deltaAngle = constrain(deltaAngle, -20, 20);
                            drv.drive(0, (int)(deltaAngle * 0.5), 0);
                        }

                        cam_angle = sensor.Cam.gate(color).center_angle;
                        cam_dist = sensor.Cam.gate(color).distance;

                        if (!isBall())
                        {
                            goto fwKickerBegin;
                        }

                        dribbler.smart_dribble(100);
                        MPU_zakrut(color);

                        goto fwKickerBegin;
                    }
                }
                else
                {
                    drv.drive(goodAngle(lineAngle + 180), 0, 80);
                }
            }
        }
    }
}

int getZakrutSign(int gateAngle)
{
    // return 0; //// заглушка, пока не будет детекции препятствий

    // print($"global gate: {goodAngle(gateAngle + getYaw())}");
    if (abs(goodAngle(gateAngle + sensor.IMU.getYaw())) < 30)
    {
        return (rand() % 2 ? -1 : 1);
        // // print($"ok 1");
        // if (sensor.Cam.obstacles[0].distance < 10)
        // {
        //     return (sensor.Cam.obstacles[0].clos_angle > 0 ? -1 : 1);
        // }
    }
    return 0;
}

void MPU_zakrut(int color)
{
    sensor.IMU.update();
    sensor.BallSensor.update();
    sensor.Cam.update();
    int sign = getZakrutSign(sensor.Cam.gate(color).center_angle); //goodAngle(sensor.IMU.Yaw + 180));
    int start_cam_angle = sensor.Cam.gate(color).center_angle;
    if (sign == 0) {
        sign = ((sensor.IMU.Yaw) < 0) ? -1 : 1;
    }
    int start_angle = sensor.IMU.Yaw;

    int rotateSpeed = 0;

    while (isBall())
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        sensor.IMU.update();
        sensor.BallSensor.update();
        sensor.LineSensor.update();

        if (sign > 0){
            rotateSpeed = abs(goodAngle(sensor.IMU.Yaw - start_angle)) < 50 ? 18 : 80; //80
            if (abs(goodAngle(sensor.IMU.Yaw - start_angle)) < 60)
                dribbler.smart_dribble(180);
            else
                kicker.kick();
            //dribbler.smart_dribble(abs(goodAngle(sensor.IMU.Yaw - start_angle)) < 80 ? 100 : 0);
        }
        else if (sign < 0){
            rotateSpeed = abs(goodAngle(sensor.IMU.Yaw - start_angle)) < 50 ? 18 : 80; //70
            if (abs(goodAngle(sensor.IMU.Yaw - start_angle)) < 60)
                dribbler.smart_dribble(180);
            else
                kicker.kick();
            //dribbler.smart_dribble(abs(goodAngle(sensor.IMU.Yaw - start_angle)) < 70 ? 100 : 0);
        }

        lineAngle = sensor.LineSensor.getAngleDelayed();
        if (lineAngle == 360){
            //if (goodAngle(start_cam_angle + start_angle) < 20)
                drv.drive(0, rotateSpeed * sign, 0);
            //else
                //drv.drive(-sign * 90 - sensor.IMU.Yaw, rotateSpeed * sign, 70); //50
        }
        else
            drv.drive(goodAngle(lineAngle + 180), 0, 80);
        // if (omnicam().gates[color].center_angle > 0){
        //     drive(0, -40, 0);
        // }
        // else {
        //     drive(0, 40, 0);
        // }
        // Debug.SendInfo();
    }

    // while (isBall())
    // {
    //     menu.writeLineClean(0, "2");
    //     // ESP_LOGI("debug", "2");
    //     vTaskDelay(10 / portTICK_PERIOD_MS);
    //     sensor.IMU.update();
    //     sensor.BallSensor.update();
    //     sensor.LineSensor.update();
        
    //     if (sign > 0){
    //         rotateSpeed = abs(goodAngle(sensor.IMU.Yaw - start_angle)) < 50 ? 25 : 80;
    //         dribbler.smart_dribble(abs(goodAngle(sensor.IMU.Yaw - start_angle)) < 50 ? 100 : 0);
    //     }
    //     else if (sign < 0){
    //         rotateSpeed = abs(goodAngle(sensor.IMU.Yaw - start_angle)) < 60 ? 25 : 80;
    //         dribbler.smart_dribble(abs(goodAngle(sensor.IMU.Yaw - start_angle)) < 60 ? 100 : 0);
    //     }

    //     lineAngle = sensor.LineSensor.getAngleDelayed();
    //     if (lineAngle == 360){
    //         drv.drive(0, rotateSpeed * sign, 0);
    //     }
    //     else{
    //         drv.drive(lineAngle, 0, -80);
    //     }

    //     // if (abs(goodAngle(sensor.IMU.Yaw - start_angle)) > 160){
    //     //     dribbler.smart_dribble(0);
    //     //     // return;
    //     // }
    // }

    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void vyravnivanije(int color)
{
    sensor.update();
    //while (abs(sensor.Cam.gate(color).center_angle) < 170 && isBall())
    while (abs(sensor.IMU.getYaw()) > 15 && isBall())
    {
        menu.writeLineClean(0, "1");
        ESP_LOGI("debug", "1");
        sensor.update();
        int cam_angle = sensor.Cam.gate(color).center_angle;
        deltaAngle = -sensor.IMU.getYaw(); //goodAngle(cam_angle + 180); //((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
        if (deltaAngle > -30 && deltaAngle <= 0) deltaAngle = -30;
        if (deltaAngle < 30 && deltaAngle > 0) deltaAngle = 30;
        // menu.writeLineClean(0, "1 " + std::to_string((int)deltaAngle));
        deltaAngle = constrain(deltaAngle, -40, 40);
        drv.drive(0, (int)(deltaAngle * 0.5), 0);
    }
    ESP_LOGI("debug", "exit 1");
    // menu.writeLineClean(0, "exit 1");
}

void penaltyDribbler(int color)
{
    make_pause(5000);

    while (true){
        vTaskDelay(10 / portTICK_PERIOD_MS);

        sensor.update();

        int robotAngle = goodAngle(sensor.IMU.getYaw());

        if (abs(robotAngle) > 60){
            drv.drive(0, -robotAngle * 0.5, 0);
        }
        else{
            petrovich_iter(color, 0, false);
            if (isBall()){
                dribbler.smart_dribble(80);
                make_pause(500);
                dribbler.smart_dribble(0);
                return;
            }
        }
    }
}

void penaltyKicker(int color)
{
    make_pause(5000);

    menu.clearDisplay();

    while (true){
        vTaskDelay(10 / portTICK_PERIOD_MS);

        sensor.update();

        int robotAngle = goodAngle(sensor.IMU.getYaw());
        int gateAngle = goodAngle(sensor.Cam.gate(color).center_angle);
        int gate_dist = sensor.Cam.gate(color).distance;
        gateAngle += (gateAngle > 0 ? 1 : -1);
        ballAngle = sensor.Locator.getBallAngleLocal();

        menu.writeLineClean(1, "Gate " + std::to_string(gateAngle));
        // menu.writeLineClean(2, "Dist " + std::to_string(gate_dist));
        // menu.writeLineClean(3, "Ball " + std::to_string(ballAngle));

        if (gateAngle == 360){
            // menu.writeLineClean(1, "NO BALL");
            continue;
        }

        bool save_front = false;

        if (abs(robotAngle) > 60){
            drv.drive(0, -robotAngle * 0.5, 0);
        }
        // else if (sensor.Cam.gate(color).distance < sensor.Cam.gate(color ^ 1).distance){
        //     drv.drive(goodAngle(-robotAngle + 180), 0, 60);
        // }
        else{
            int angle_err = goodAngle(ballAngle - gateAngle);
            bool gate_in_front = abs(angle_err) <= sensor.Cam.gate(color).width * 0.5 || abs(angle_err) <= 2;

            deltaAngle = gateAngle * 0.4;

            if ((gate_in_front) || (isBall()))
            {
                // menu.writeLineClean(2, "Move2gate");
                moveAngle = gateAngle;
                drv.drive(moveAngle, (int)deltaAngle, 30);

                if (isBall() && gate_in_front){
                    drv.drive(moveAngle, 0, 80);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    kicker.kick();
                    drv.drive(0, 0, 0, 0);
                    return;
                }
            }
            else
            {
                // menu.writeLineClean(3, "Drive2ball");
                moveAngle = ballAngle;
                if (angle_err > 0)
                    moveAngle = goodAngle(ballAngle + constrain(sensor.Locator.getStrength() * goRoundBallCoefFw, 0, 90));
                else
                    moveAngle = goodAngle(ballAngle - constrain(sensor.Locator.getStrength() * goRoundBallCoefFw, 0, 90));
                drv.drive(moveAngle, (int)deltaAngle, 50);
            }
        }
    }
}
