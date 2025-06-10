#include "logics.h"
#include "esp_timer.h"
#include "vector2.h"
#include "RealDist.h"

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
float goRoundBallCoefFw = 1.f;
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

float g_kp = 50;
float g_ki = 0;
float g_kd = 30;

float leftIntegral = 0;
float leftPrev = 0;
float rightIntegral = 0;
float rightPrev = 0;

float gb_kp = 1.;
float gb_ki = 0;
float gb_kd = 0;
float gkBallIntegral = 0;
float gkBallPrev = 0;

float gkReactOnBallDiap = 0;

// Вратарь по камере
int lastGateAngle = 180;
int lastGateTime = 0;
int lastGateReset = 5000;

float gate_kp = -10;
float gate_kd = 10;
float gate_ki = 0;

float gateIntegral = 0;
float gatePrev = 0;

float limitGateIntegral = 150;
float limitGateSpeed = 70;

float ballMoveTime = 0;
float lastBallAngle = 0;
float ballNoMotionDiap = 10;
float lastMoveBallStrength = 0;
float ballNoMotionStrength = 15;

float kfTimer = 0;
float maxKfTime = 10000;

float prevBallStrength = 0;
// float gk_st_kd = 100;

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
        return sensor.BallSensor.ballCatched();
    }
    else
        return (sensor.Locator.getStrength() >= 100 && abs(sensor.Locator.getBallAngleLocal()) <= 10);
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
            //lastGateAngle = (int)goodAngle(gateAngle + robotAngle);
            //lastGateTime = millis();
        }
        else
        {
            drv.driveXY(0, 0, 20);
            continue;
        }

        int cam_height = sensor.Cam.gate(color).height;

        lineAngle = sensor.LineSensor.getAngleDelayed();

        deltaAngle = goodAngle(gateAngle) * 0.25f;

        ESP_LOGI("killerFeature", "gateAngle: %d", gateAngle);

        // if (abs(goodAngle(gateAngle - ballAngle)) >= 140){
        //     stateGame = 0;
        //     continue;
        // }

        // if (abs(deltaAngle) <= 10 && cam_height == 0)
        // {
        //     deltaAngle = 50;
        // }

        if (lineAngle != 360)
        {
            drv.drive(goodAngle(lineAngle + 180), (int)deltaAngle, 50);
        }
        else
        {
            int angle_err = goodAngle(ballAngle - gateAngle);
            if (sensor.Locator.getStrength() >= 70 && abs(angle_err) < 15)
            {
                moveAngle = gateAngle;
                drv.drive(moveAngle, (int)deltaAngle, 80);
            }
            else
            {
                moveAngle = ballAngle;
                /*if (ballAngle >= 0 && ballAngle < 150)
                    moveAngle -= (180 - ballAngle) * 0.6f;
                if (ballAngle < 0 && ballAngle > -150)
                    moveAngle -= (-180 - ballAngle) * 0.6f;*/
                if (angle_err > 0)
                    moveAngle = goodAngle(ballAngle + constrain(sensor.Locator.getStrength() * goRoundBallCoefGk, 0, 90));
                else
                    moveAngle = goodAngle(ballAngle - constrain(sensor.Locator.getStrength() * goRoundBallCoefGk, 0, 90));

                drv.drive(moveAngle, (int)deltaAngle, 50);
            }
        }

        // if (cam_height >= 700)
        // {
        //     stateGame = 0;
        //     continue;
        // }
    }
}

void playGoalkeeperCamera(int color)
{
    menu.clearDisplay();
    while (true)
    {
        // sensor.update();
        // moveAngle = sensor.Cam.Blue.center_angle;
        // deltaAngle = -sensor.IMU.getYaw() * 0.25;
        // goOverObstacleOmni(60, moveAngle, deltaAngle, 13, false);

        // // drv.drive(moveAngle, deltaAngle, 30);
        // continue;

        vTaskDelay(10 / portTICK_PERIOD_MS);

        //killerFeature(1 ^ color);
        //continue;

        sensor.update();

        if (sensor.Locator.getStrength() < 5)
        {
            menu.writeLineClean(1, "No ball");
            drv.drive(0, 0, 0, 0);
            continue;
        }

        if (stateGame != 0)
        {
            kfTimer = millis();
            killerFeature(1 ^ color);
            // Serial.println("KILLER!!!");
            continue;
        }

        float global_x, global_y;
        // int get_pos_callback = getGlobalPosition_2gates(global_x, global_y, color);
        // BTDebug.send();
        // if (get_pos_callback == 0)
        // {
        //     menu.writeLineClean(3, "GP X " + std::to_string(global_x));
        //     menu.writeLineClean(4, "GP Y " + std::to_string(global_y));
        // }
        // else if (get_pos_callback == 1)
        // {
        //     menu.writeLineClean(3, "FAILED: no gate");
        //     menu.writeLineClean(4, "");
        // }
        // else if (get_pos_callback == 2)
        // {
        //     menu.writeLineClean(3, "FAILED: parallel");
        //     menu.writeLineClean(4, "");
        // }
        Vector2 rightBallDir(1.0, 0.0);
        Vector2 leftBallDir(-1.0, 0.0);

        // if (global_x > 35){
        //     rightBallDir = Vector2(0, -0.5f);
        //     if (global_y < -65) {
        //         leftBallDir = Vector2(0, 1);
        //     }
        //     else {
        //         leftBallDir = Vector2(-1.2f, 0);
        //     }
        // }

        // if (global_x < -35){
        //     leftBallDir = Vector2(0, -0.5f);
        //     if (global_y < -65) {
        //         rightBallDir = Vector2(0, 1);
        //     }
        //     else {
        //         rightBallDir = Vector2(1.2f, 0);
        //     }
        // }

        // std::string s;
        // if (leftBallDir.y < 0){
        //     s += "v   ";
        // }
        // else if (leftBallDir.y > 0){
        //     s += "^   ";
        // }
        // else
        //     s += "    ";
        // if (rightBallDir.y < 0){
        //     s += "v   ";
        // }
        // else if (rightBallDir.y > 0){
        //     s += "^   ";
        // }
        // else
        //     s += "    ";
        // menu.writeLineClean(5, s);

        ballAngle = sensor.Locator.getBallAngleLocal();
        int robotAngle = sensor.IMU.getYaw();

        float lineX, lineY;
        //getLineDirection_Delayed(lineX, lineY, true);
        sensor.LineSensor.getDirectionDelayed(lineX, lineY);

        int gateAngle = goodAngle(sensor.Cam.gate(color).center_angle /* + 180*/);
        int globalGateAngle = goodAngle(gateAngle + robotAngle);
        int cam_height = sensor.Cam.gate(color).height;
        int cam_dist = sensor.Cam.gate(color).distance;

        // if (globalGateAngle < -135 || globalGateAngle > 135)
        // {
        //     leftBallDir = Vector2(-1.5f, 0);
        //     rightBallDir = Vector2(1.5f, 0);
        // }

        // menu.writeLineClean(1, "GK " + std::to_string(globalGateAngle) + "  " + std::to_string(cam_dist));

        if (cam_height > 0)
        {
            lastGateAngle = globalGateAngle;
            lastGateTime = millis();
        }
        else
        {
            drv.driveXY(0, 0, 20);
            menu.writeLineClean(1, "No gates");
            continue;
            // gateAngle = lastGateAngle - robotAngle;
            // cam_height = 110;
        }

        lineAngle = sensor.LineSensor.getAngleDelayed();

        int ball_strength = sensor.Locator.getStrength();

        if (abs(goodAngle(ballAngle + robotAngle - lastBallAngle)) > ballNoMotionDiap)
        {
            lastBallAngle = ballAngle + robotAngle;
            lastMoveBallStrength = ball_strength;
            ballMoveTime = millis();
        }

        if (abs(ball_strength - lastMoveBallStrength) > ballNoMotionStrength)
        {
            lastBallAngle = ballAngle + robotAngle;
            lastMoveBallStrength = ball_strength;
            ballMoveTime = millis();
        }

        if (millis() - ballMoveTime >= 5000)
        {
            drv.driveXY(0, 70, 0);
            make_pause(100);
            stateGame = 1;
            continue;
        }

        if (lineAngle != 360 && (abs(globalGateAngle)) <= 135)
        {
            menu.writeLineClean(2, "Line");
            // speedX = (int)(-lineX * 80);
            // speedY = (int)(-lineY * 80);
            //deltaAngle = -robotAngle * 0.25;
            deltaAngle = -(int)goodAngle(180 - gateAngle) * 0.25;
            drv.drive(goodAngle(lineAngle + 180), 0, 100);
            continue;
        }
        else
        {
            menu.writeLineClean(2, "");
            speedX = 0;

            // int err = pixel_dist_to_real(cam_dist) - 48;

            // if (err < 0){
            //     speedY = constrain(-err * 20, 0, 80);
            // }
            // else if (err > 10){
            //     speedY = constrain(-err * 5, -80, 0);
            // }
            // else{
            //     speedY = 0;
            // }

            //int err = -constrain((-1. / 84000 * cam_dist * cam_dist * cam_dist * cam_dist + 1. / 672 * cam_dist * cam_dist * cam_dist - 31. / 1680 * cam_dist * cam_dist - 11. / 84 * cam_dist), 0, 100);
            int err = -constrain((0. + 0.628449 * cam_dist - 0.0637014 * cam_dist * cam_dist + 0.00194066 * cam_dist * cam_dist * cam_dist - 0.0000172587 * cam_dist * cam_dist * cam_dist * cam_dist +
                                  4.62172e-8 * cam_dist * cam_dist * cam_dist * cam_dist * cam_dist - 2.788e-12 * cam_dist * cam_dist * cam_dist * cam_dist * cam_dist * cam_dist),
                                 0, 100);
            if (cam_dist < 15)
                err = 50;
            else if (cam_dist < 20)
                err = 25;
            else if (cam_dist < 25)
                err = 0;
            speedY = err;
            // int err = speedY = 0;

            // speedY = (int)(err * gate_kp + (err - gatePrev) * gate_kd + gateIntegral);

            // menu.writeLineClean(3, std::to_string(cam_dist) + " " + std::to_string(err) + " " + std::to_string(speedY));
            // if (err < -2)
            //     speedY = 0.07 * err * err + 5 * err - 3.5; zzzzzzzzzzzzzzzzz
            // else if (err < 2)
            //     speedY = 0;
            // else
            //     speedY = 40 + err * 7;
            speedY = (int)constrain(speedY, -limitGateSpeed, 100);
            gatePrev = err;
            gateIntegral += (err * gate_ki);
            gateIntegral = constrain(gateIntegral, -limitGateIntegral, limitGateIntegral);

            // если мяч сзади он должен сюда заходить
            if (abs(ballAngle) > 80 && speedY <= 0)
            {
                //deltaAngle = -robotAngle * 0.25;
                deltaAngle = -(int)goodAngle(180 - gateAngle) * 0.25;
                ball_strength = sensor.Locator.getStrength();
                if (ballAngle > 0)
                {
                    moveAngle = ballAngle + constrain(ball_strength * goRoundBallCoefGk, 0, 90);
                }
                else
                {
                    moveAngle = ballAngle - constrain(ball_strength * goRoundBallCoefGk, 0, 90);
                }
                moveAngle = goodAngle(moveAngle);
                drv.drive(moveAngle, (int)deltaAngle, (int)(50));
                //Debug.Log("go round");
                continue;
            }
        }

        // ESP_LOGI("playGoalkeeperCamera", "speedY = %d", speedY);

        // if (abs(ballAngle) < 15)
        //     ballAngle *= 2;

        ballAngle = constrain(ballAngle, -maxGoalkeeperAngle - robotAngle, maxGoalkeeperAngle - robotAngle);
        int ballAngleGlobal = goodAngle(ballAngle - gateAngle - 180);

        float ball_err = ballAngleGlobal;

        float ballSpeed = gb_kp * ball_err + gkBallIntegral + gb_kd * (ball_err - gkBallPrev);
        // if (ball_strength > prevBallStrength){
        //     ballSpeed += constrain((ball_strength - prevBallStrength) * gk_st_kd, -50, 50);
        // }
        gkBallIntegral += (ball_err)*gb_ki;
        gkBallPrev = ball_err;

        if (ballSpeed > 0)
        {
            speedX += (int)(abs(ballSpeed) * rightBallDir.x);
            speedY += (int)(abs(ballSpeed) * rightBallDir.y);
        }
        else
        {
            speedX += (int)(abs(ballSpeed) * leftBallDir.x);
            speedY += (int)(abs(ballSpeed) * leftBallDir.y);
        }

        // menu.writeLineClean(2, "sp " + std::to_string(speedX) + ";" + std::to_string(speedY));

        deltaAngle = -(int)goodAngle(180 - gateAngle) * 0.25;

        // prevBallStrength = ball_strength;

        // speedX = constrain(speedX, -50, 50);
        // speedY = constrain(speedY, -50, 50);
        int sp = sqrt(speedX * speedX + speedY * speedY);
        float angle = atan2(speedX, speedY) * RAD_TO_DEG;
        angle += goodAngle(gateAngle - 180);
        drv.drive(angle, (int)deltaAngle, constrain(sp, 0, 100));

        // if (sp > 100)
        // {
        //     speedX = speedX * 100 / sp;
        //     speedY = speedY * 100 / sp;
        // }

        // drv.driveXY(speedX, speedY, (int)deltaAngle);
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

        if (sensor.Locator.getStrength() < 5)
        {
            drv.drive(0, 0, 0, 0);
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
        
        if (lineAngle != 360)
        {
            drv.drive(goodAngle(lineAngle + 180), (int)deltaAngle, 50);
        }
        else
        {
            int angle_err = goodAngle(ballAngle - cam_angle);
            if (sensor.Locator.getStrength() >= 70 && abs(angle_err) < 15)
            {
                moveAngle = cam_angle;
                drv.drive(moveAngle, (int)deltaAngle, 80);
            }
            else
            {
                moveAngle = ballAngle;
                if (angle_err > 0)
                    moveAngle = goodAngle(ballAngle + constrain(sensor.Locator.getStrength() * goRoundBallCoefGk, 0, 90));
                else
                    moveAngle = goodAngle(ballAngle - constrain(sensor.Locator.getStrength() * goRoundBallCoefGk, 0, 90));

                drv.drive(moveAngle, (int)deltaAngle, 50);

                if (sensor.Cam.gate(color).distance < 50){
                    kicker.kick();
                }
            }
        }

        // if (abs(robotAngle) > 80)
        // {
        //     menu.writeLineClean(0, "rotate");
        //     menu.writeLineClean(1, "");
        //     menu.writeLineClean(2, "");
        //     drv.drive(0, -robotAngle * 0.3, 0);
        // }

        // else if (!isBall())
        // {
        //     //smart_dribble((abs(ballAngle) < 40) ? 30 : 0);
        //     int deltaAngle = -robotAngle; //ballAngle * 0.3;
        //     int angle_err = goodAngle(ballAngle - cam_angle);
        //     if (lineAngle == 360)
        //     {
        //         // moveAngle = ballAngle + (ballAngle > 0) ? 15 : -15;
        //         // int angle_err = goodAngle(ballAngle - cam_angle);
        //         // if (abs(angle_err) <= 25)
        //         // {
        //         //     moveAngle = ballAngle;
        //         // }
        //         // else if (angle_err > 25)
        //         //     moveAngle = ballAngle + constrain(st * goRoundBallCoefGk, 0, 90);
        //         // else if (angle_err < 25)
        //         //     moveAngle = ballAngle - constrain(st * goRoundBallCoefGk, 0, 90);
        //         moveAngle = ballAngle;
        //         if (angle_err > 0)
        //             moveAngle = goodAngle(ballAngle + constrain(sensor.Locator.getStrength() * goRoundBallCoefGk, 0, 90));
        //         else
        //             moveAngle = goodAngle(ballAngle - constrain(sensor.Locator.getStrength() * goRoundBallCoefGk, 0, 90));
                
        //         if (sensor.Cam.gate(color).distance < 40){
        //             kicker.kick();
        //         }

        //         // double k = 0.5;

        //         // int delta = 0;
        //         // // if (abs(angle) <= 45){
        //         // //   k = 3;
        //         // // }
        //         // if (ballAngle >= 30) {
        //         //     //delta = sqrt(st) * k;
        //         //     delta = st * k;
        //         //     //speed = 150;
        //         // }
        //         // else if (ballAngle <= -30) {
        //         //     //delta = -sqrt(st) * k;
        //         //     delta = -st * k;
        //         //     // speed = 150;
        //         // }
        //         // else {
        //         //     delta = 0;
        //         //     // speed = 230;
        //         // }

        //         // moveAngle = ballAngle + delta;

        //         // menu.writeLineClean(0, "drive2ball");
        //         // menu.writeLineClean(1, std::to_string(st));
        //         // menu.writeLineClean(2, std::to_string(ballAngle));

        //         drv.drive(moveAngle, (int)deltaAngle, 60);
        //     }
        //     else
        //     {
        //         drv.drive(goodAngle(lineAngle + 180), deltaAngle, 90);
        //         // menu.writeLineClean(0, "LINE 1");
        //         // menu.writeLineClean(1, std::to_string(lineAngle));
        //         // menu.writeLineClean(2, "");
        //     }
        // }
        // else
        // {
        //     make_pause(0);

        //     while (isBall())
        //     {
        //         sensor.update();

        //         cam_angle = sensor.Cam.gate(color).center_angle;
        //         cam_dist = sensor.Cam.gate(color).distance;

        //         st = sensor.Locator.getStrength();
        //         ballAngle = sensor.Locator.getBallAngleLocal();
        //         lineAngle = sensor.LineSensor.getAngleDelayed();

        //         robotAngle = sensor.IMU.getYaw();

        //         if (lineAngle == 360)
        //         {
        //             int delta_angle = constrain(cam_angle * 0.5, -20, 20);
        //             drv.drive(cam_angle, delta_angle, 70);
        //             // menu.writeLineClean(0, "move2gate");
        //             // menu.writeLineClean(1, std::to_string(cam_angle));
        //             // menu.writeLineClean(2, std::to_string(cam_dist));
        //             if (abs(cam_angle) < 10 && cam_dist < 35)
        //             {
        //                 drv.drive(0, 0, 0, 0);
        //                 // menu.writeLineClean(0, "WAIT");
        //                 // menu.writeLineClean(1, "");
        //                 // menu.writeLineClean(2, "");
        //                 // make_pause(1000);
        //             }
        //         }
        //         else
        //         {
        //             drv.drive(goodAngle(lineAngle + 180), 0, 90);
        //             // menu.writeLineClean(0, "LINE 2");
        //             // menu.writeLineClean(1, std::to_string(lineAngle));
        //             // menu.writeLineClean(2, "");
        //         }
        //     }
        // }
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
        int rotateSpeed = abs(sensor.Cam.gate(color).center_angle) > 150 ? 30 : 90;        // 42 : 100 // > 150 ? 30 : 90
        dribbler.smart_dribble(abs(sensor.Cam.gate(color).center_angle) > 130 ? 110 : 60); // 110 // > 120 ? 110 : 60
        drv.drive(0, rotateSpeed * sign, 0);

        if (abs(sensor.Cam.gate(color).center_angle) < 20){
            dribbler.smart_dribble(-10);
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
        if (sensor.Cam.gate(color).width < 0){
            menu.writeLineClean(0, "exit goalPush");
            break;            
        }
        menu.writeLineClean(0, "goalPush");
        ballAngle = sensor.Locator.getBallAngleLocal();
        
        int gateAngle = (int)sensor.Cam.gate(color).center_angle;
        if (gateAngle == 360)
            break;
        lineAngle = sensor.LineSensor.getAngleDelayed();

        int offset_angle = (int)goodAngle(gateAngle - 45);
        if (abs((int)goodAngle(gateAngle + 45)) < abs(offset_angle))
            offset_angle = (int)goodAngle(gateAngle + 45);

        deltaAngle = goodAngle(offset_angle) * 0.25f;
        if (lineAngle != 360)
        {
            drv.drive(goodAngle(lineAngle + 180), (int)deltaAngle, 50);
        }
        else
        {
            int angle_err = goodAngle(ballAngle - gateAngle);
            if (sensor.Locator.getStrength() >= 70 && abs(angle_err) <= 25)
            {
                moveAngle = gateAngle;
                drv.drive(moveAngle, (int)deltaAngle, 80);
            }
            else
            {
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

float FwBallAnglIntegral = 0;
float FwBallAnglPrev = 0;
float Fw_kd = 5;
float Fw_ki = 0;
float Fw_kp = 0.4;

#define OTLADKA_Dribble2
#ifdef OTLADKA_Dribble2
    const char* drible2 = "f";
#endif
void playForwardDribble2(int color)
{
    menu.clearDisplay();
    color = 1 ^ color;
    
    while (true)
    {
    fwDribbleBegin:

        ESP_LOGI(drible2, "-NEW ITERATION-");
        vTaskDelay(10 / portTICK_PERIOD_MS);

        sensor.update();

        int cam_angle = sensor.Cam.gate(color).center_angle;
        int cam_dist = sensor.Cam.gate(color).distance;

        int st = sensor.Locator.getStrength();
        int ballAngle = sensor.Locator.getBallAngleLocal();
        if (sensor.Locator.getStrength() < 5)
        {
            drv.drive(0, 0, 0, 0);
            continue;
        }
        int lineAngle = sensor.LineSensor.getAngleDelayed();

        // if (sensor.Cam.gate(color).width >= 0)
        //    goalPush(color);

        if (!isBall())
        {
            //menu.writeLineClean(0, "ball");
            dribbler.smart_dribble((abs(ballAngle) < 50) ? 60 : 0);
            // dribbler.neutral();
            int deltaAngle = ballAngle * 0.3; //(ballAngle < 45 ? 0.3 : 0.5);
            if (lineAngle == 360)
            {
                ESP_LOGW(drible2, "Mach poteryan, edu k machu");
                moveAngle = ballAngle;
                float ballAnfl_err = ballAngle;
                float robotAnglSpeed = Fw_kp * ballAnfl_err + FwBallAnglIntegral + Fw_kd * (ballAnfl_err - FwBallAnglPrev);
                FwBallAnglIntegral += (ballAnfl_err)*Fw_ki;
                FwBallAnglPrev = ballAnfl_err;

                drv.drive(moveAngle, (int)(robotAnglSpeed), 60);  
            }
            else
            {
                ESP_LOGW(drible2, "Mach poteryan, edu k machu no vizhu liniju");
                drv.drive(goodAngle(lineAngle + 180), (int)(deltaAngle), 80);
            }
        }
        else
        {
            #ifdef OTLADKA_Dribble2  
            ESP_LOGW(drible2, "Mach zachvachen, podjzzaju");
            #endif
            //menu.writeLineClean(0, "gate...");
            dribbler.smart_dribble(70);
            drv.driveXY(0, 30, 0); // чтобы лучше задриблить 
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
            // make_pause(200);
            sensor.BallSensor.update();
            while (isBall())
            {
                
                //menu.writeLineClean(0, "gate");
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
                        speed = (lineAngle == 360) ? 0 : 60;
                        cam_angle = sensor.Cam.gate(color).center_angle;
                        deltaAngle = goodAngle(cam_angle + 180);
                        deltaAngle = abs(cam_angle) < 90 ? deltaAngle * 0.3 : deltaAngle * 0.5; //((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                        deltaAngle = constrain(deltaAngle, -40, 40);
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

                    sensor.update();

                    int delta_angle = goodAngle(cam_angle + 180) * 0.3; //((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                    delta_angle = constrain(delta_angle, -15, 15);

                    lineAngle = sensor.LineSensor.getAngleDelayed();
                    if (lineAngle != 360)
                    {
                        #ifdef OTLADKA_Dribble2  
                        ESP_LOGW(drible2, "edu k vorotam NO LINIJA");
                        #endif
                        drv.drive(goodAngle(lineAngle + 180), delta_angle, 80);
                        continue;
                    }
                    else{
                        #ifdef OTLADKA_Dribble2  
                        ESP_LOGW(drible2, "edu k vorotam");
                        #endif
                        
                        drv.drive(cam_angle, delta_angle, 80);
                    }
                    cam_dist = sensor.Cam.gate(color).distance;
                    // Serial.println(cam_dist);
                    if (abs(cam_angle) > 100 && cam_angle != 360 && cam_dist < 20)
                    {
                        #ifdef OTLADKA_Dribble2                      
                            ESP_LOGW(drible2, "---ATAKUJU---");
                        #endif
                        // return;
                        drv.drive(0, 0, 0, 0);
                        sensor.update();
                        dribbler.smart_dribble(80);

                        make_pause(100);

                        cam_angle = sensor.Cam.gate(color).center_angle;

                        while (abs(cam_angle) < 160 && isBall())
                        {
                            sensor.update();
                            cam_angle = sensor.Cam.gate(color).center_angle;
                            deltaAngle = goodAngle(cam_angle + 180); //((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                            deltaAngle = constrain(deltaAngle, -40, 40);
                            drv.drive(0, (int)(deltaAngle * 0.5), 0);
                        }

                        cam_angle = sensor.Cam.gate(color).center_angle;
                        cam_dist = sensor.Cam.gate(color).distance;

                        if (!isBall())
                        {
                            goto fwDribbleBegin;
                        }

                        // может это в goalDriveBack?
                        dribbler.smart_dribble(110);
                        make_pause(500);

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

void MPU_zakrut(int color)
{
    sensor.IMU.update();
    sensor.BallSensor.update();
    int sign = ((sensor.IMU.Yaw) < 0) ? -1 : 1;
    int start_angle = sensor.IMU.Yaw;

    while (isBall())
    {
        menu.writeLineClean(0, "2");
        // ESP_LOGI("debug", "2");
        vTaskDelay(10 / portTICK_PERIOD_MS);
        sensor.IMU.update();
        sensor.BallSensor.update();
        
        int rotateSpeed = abs(goodAngle(sensor.IMU.Yaw - start_angle)) < 40 ? 30 : 90;
        dribbler.smart_dribble(abs(goodAngle(sensor.IMU.Yaw - start_angle)) < 50 ? 110 : -10);
        drv.drive(0, rotateSpeed * sign, 0);

        if (abs(goodAngle(sensor.IMU.Yaw - start_angle)) > 160){
            dribbler.smart_dribble(-10);
            return;
        }
    }
    
    // ESP_LOGI("debug", "exit 2");
    menu.writeLineClean(0, "exit 2");
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
