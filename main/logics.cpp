#include "logics.h"
#include "esp_timer.h"
#include "vector2.h"

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
float goRoundBallCoefGk = 0.62f;

float g_kp = 50;
float g_ki = 0;
float g_kd = 30;

float leftIntegral = 0;
float leftPrev = 0;
float rightIntegral = 0;
float rightPrev = 0;

float gb_kp = 2.;
float gb_ki = 0;
float gb_kd = 6;
float gkBallIntegral = 0;
float gkBallPrev = 0;

float gkReactOnBallDiap = 0;

// Вратарь по камере
int lastGateAngle = 180;
int lastGateTime = 0;
int lastGateReset = 5000;

float gate_kp = 1.6;
float gate_kp_neg = 0.5;
float gate_kd = 0;
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
    vTaskDelay(ms / portTICK_PERIOD_MS);
    // int start = millis();
    // while (millis() - start > 0){
    //     sensor.update();
    //     vTaskDelay(10 / portTICK_PERIOD_MS);
    // }
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

int getGlobalPosition_2gates(float &x, float &y, int color)
{
    int our_gate = sensor.Cam.GlobalYellow.center_angle + sensor.IMU.getYaw();
    int other_gate = sensor.Cam.GlobalBlue.center_angle + sensor.IMU.getYaw();
    if (color == 1)
        std::swap(our_gate, other_gate);

    if (our_gate == 360 || other_gate == 360)
        return 1;

    float our_x = 0, our_y = -110;
    float other_x = 0, other_y = 110;

    float a1 = cos(our_gate * DEG_TO_RAD);
    float b1 = -sin(our_gate * DEG_TO_RAD);
    float c1 = -a1 * our_x - b1 * our_y;

    float a2 = cos(other_gate * DEG_TO_RAD);
    float b2 = -sin(other_gate * DEG_TO_RAD);
    float c2 = -a2 * other_x - b2 * other_y;

    float vp = a1 * b2 - a2 * b1;

    float our_dist = (color == 0 ? sensor.Cam.gate(0).distance : sensor.Cam.gate(1).distance);
    float other_dist = (color == 0 ? sensor.Cam.gate(1).distance : sensor.Cam.gate(0).distance);

    // if (a1 * b2 == a2 * b1 || abs(vp) < 0.35f)
    // {
    //     x = 0;
    //     y = -110 + 220 * our_dist / (our_dist + other_dist);

    //     return 2;
    // }

    // x = -(c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1);
    // y = -110 + 220 * our_dist / (our_dist + other_dist); // -(a1 * c2 - a2 * c1) / (a1 * b2 - a2 * b1);

    // return 0;

    if (a1 * b2 == a2 * b1)
        return 2;

    x = -(c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1);
    y = -(a1 * c2 - a2 * c1) / (a1 * b2 - a2 * b1);
    return 0;
}

bool isBall()
{
    return (sensor.Locator.getStrength() >= 100 && abs(sensor.Locator.getBallAngleLocal()) <= 10);
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
            gateAngle = (int)goodAngle(-gateAngle /*+ 180*/);
            //lastGateAngle = (int)goodAngle(gateAngle + robotAngle);
            //lastGateTime = millis();
        }
        else
        {
            drv.driveXY(0, 0, 20);
            continue;
            gateAngle = (int)goodAngle(180 - robotAngle);
        }

        int cam_height = sensor.Cam.gate(color).height;

        lineAngle = sensor.LineSensor.getAngleDelayed();

        deltaAngle = goodAngle(gateAngle + 180) * 0.25f;

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
            if (sensor.Locator.getStrength() >= 70 && abs(ballAngle) > 165)
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
                if (ballAngle > 0)
                    moveAngle = goodAngle(ballAngle - constrain(sensor.Locator.getStrength() * goRoundBallCoefGk, 0, 90));
                else
                    moveAngle = goodAngle(ballAngle + constrain(sensor.Locator.getStrength() * goRoundBallCoefGk, 0, 90));

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
        if (rotate) // если нужно отворачиваться от препятствия, то отворачиваемся
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
    else{
        menu.writeLineClean(0, "");
        menu.writeLineClean(1, "");
    }

    menu.writeLineClean(2, "moveAngle: " + std::to_string((int)moveAngle));
    menu.writeLineClean(3, "deltaAngle: " + std::to_string((int)deltaAngle));

    //moveAngle = limitSpeed(moveAngle, generalSpeed, global_x, global_y);
    drv.drive((int)moveAngle, (int)deltaAngle, (int)generalSpeed);
}

void playGoalkeeperCamera(int color)
{
    menu.clearDisplay();
    while (true)
    {
        sensor.update();
        moveAngle = -sensor.Cam.Blue.center_angle;
        deltaAngle = -sensor.IMU.getYaw() * 0.25;
        goOverObstacleOmni(60, moveAngle, deltaAngle, 13, false);

        // drv.drive(moveAngle, deltaAngle, 30);
        continue;

        //while (true)
        //{
            vTaskDelay(10 / portTICK_PERIOD_MS);

            //killerFeature(1 ^ color);
            //continue;

            sensor.update();

            // if (sensor.Locator.getStrength() < 5)
            // {
            //     menu.writeLineClean(1, "No ball");
            //     drv.drive(0, 0, 0, 0);
            //     continue;
            // }

            // if (stateGame != 0)
            // {
            //     kfTimer = millis();
            //     // killerFeature(1 ^ color);
            //     // Serial.println("KILLER!!!");
            //     continue;
            // }

            float global_x, global_y;
            int get_pos_callback = getGlobalPosition_2gates(global_x, global_y, color);
            // if (get_pos_callback == 0)
            // {
            //     menu.writeLineClean(3, "GP X" + std::to_string(global_x));
            //     menu.writeLineClean(4, "GP Y" + std::to_string(global_y));
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
        //}

        Vector2 rightBallDir(1.0, 0.0);
        Vector2 leftBallDir(-1.0, 0.0);

        // if (global_x > 35){
        //     rightBallDir = Vector2(0, -0.5f);
        //     if (global_y < -80) {
        //         leftBallDir = Vector2(0, 1);
        //     }
        //     else {
        //         leftBallDir = Vector2(-1.5f, 0);
        //     }
        // }

        // if (global_x < -35){
        //     leftBallDir = Vector2(0, -0.5f);
        //     if (global_y < -80) {
        //         rightBallDir = Vector2(0, 1);
        //     }
        //     else {
        //         rightBallDir = Vector2(1.5f, 0);
        //     }
        // }

        ballAngle = sensor.Locator.getBallAngleLocal();
        int robotAngle = sensor.IMU.getYaw();

        float lineX, lineY;
        //getLineDirection_Delayed(lineX, lineY, true);
        sensor.LineSensor.getDirectionDelayed(lineX, lineY);

        int gateAngle = goodAngle(-sensor.Cam.gate(color).center_angle /* + 180*/);
        int globalGateAngle = goodAngle(gateAngle + robotAngle);
        int cam_height = sensor.Cam.gate(color).height;
        int cam_dist = sensor.Cam.gate(color).distance;

        menu.writeLineClean(1, "GK " + std::to_string(gateAngle) + "  " + std::to_string(cam_dist));

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
            deltaAngle = (gateAngle == 360) ? -robotAngle : -(int)goodAngle(180 - gateAngle);
            drv.drive(goodAngle(lineAngle + 180), deltaAngle, 80);
            continue;
        }
        else
        {
            menu.writeLineClean(2, "");
            speedX = 0;
            int err = 13 - cam_dist; //cam_height - 10;
            // speedY = (int)(err * gate_kp + (err - gatePrev) * gate_kd + gateIntegral);
            if (err < -2)
                speedY = 0.07 * err * err + 5 * err - 3.5;
            else if (err < 2)
                speedY = 0;
            else
                speedY = 40 + err * 7;
            speedY = (int)constrain(speedY, -limitGateSpeed, 100);
            gatePrev = err;
            gateIntegral += (err * gate_ki);
            gateIntegral = constrain(gateIntegral, -limitGateIntegral, limitGateIntegral);

            if (abs(ballAngle) > 80 && err < 0)
            {
                deltaAngle = (gateAngle == 360) ? -robotAngle * 0.5 : -(int)goodAngle(180 - gateAngle) * 0.25;
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

        ESP_LOGI("playGoalkeeperCamera", "speedY = %d", speedY);

        // if (abs(ballAngle) < 15)
        //     ballAngle *= 2;

        ballAngle = constrain(ballAngle, -maxGoalkeeperAngle - robotAngle, maxGoalkeeperAngle - robotAngle);

        float ball_err = ballAngle;

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

        menu.writeLineClean(2, "sp " + std::to_string(speedX) + ";" + std::to_string(speedY));

        deltaAngle = -(int)goodAngle(180 - gateAngle) * 0.25;

        // prevBallStrength = ball_strength;

        // speedX = constrain(speedX, -50, 50);
        // speedY = constrain(speedY, -50, 50);
        int sp = sqrt(speedX * speedX + speedY * speedY);
        if (sp > 100)
        {
            speedX = speedX * 100 / sp;
            speedY = speedY * 100 / sp;
        }

        drv.driveXY(speedX, speedY, (int)deltaAngle);
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

        int cam_angle = -sensor.Cam.gate(color).center_angle;
        int cam_dist = sensor.Cam.gate(color).distance;

        int st = sensor.Locator.getStrength();
        int ballAngle = sensor.Locator.getBallAngleLocal();
        int lineAngle = sensor.LineSensor.getAngleDelayed();

        int robotAngle = sensor.IMU.getYaw();

        if (abs(robotAngle) > 80)
        {
            menu.writeLineClean(0, "rotate");
            menu.writeLineClean(1, "");
            menu.writeLineClean(2, "");
            drv.drive(0, -robotAngle * 0.3, 0);
        }

        else if (!isBall())
        {
            //dribble((abs(ballAngle) < 40) ? 30 : 0);
            int deltaAngle = -robotAngle; //ballAngle * 0.3;
            if (lineAngle == 360)
            {
                moveAngle = ballAngle + (ballAngle > 0) ? 15 : -15;
                if (abs(ballAngle) < 25)
                {
                    moveAngle = ballAngle;
                }
                else
                    moveAngle = ballAngle + constrain(st * goRoundBallCoefGk, 0, 90);
                // double k = 0.5;

                // int delta = 0;
                // // if (abs(angle) <= 45){
                // //   k = 3;
                // // }
                // if (ballAngle >= 30) {
                //     //delta = sqrt(st) * k;
                //     delta = st * k;
                //     //speed = 150;
                // }
                // else if (ballAngle <= -30) {
                //     //delta = -sqrt(st) * k;
                //     delta = -st * k;
                //     // speed = 150;
                // }
                // else {
                //     delta = 0;
                //     // speed = 230;
                // }

                // moveAngle = ballAngle + delta;

                menu.writeLineClean(0, "drive2ball");
                menu.writeLineClean(1, std::to_string(st));
                menu.writeLineClean(2, std::to_string(ballAngle));

                drv.drive(moveAngle, (int)deltaAngle, 50);
            }
            else
            {
                drv.drive(goodAngle(lineAngle + 180), deltaAngle, 60);
                menu.writeLineClean(0, "LINE 1");
                menu.writeLineClean(1, std::to_string(lineAngle));
                menu.writeLineClean(2, "");
            }
        }
        else
        {
            make_pause(0);

            while (isBall())
            {
                sensor.update();

                cam_angle = -sensor.Cam.gate(color).center_angle;
                cam_dist = sensor.Cam.gate(color).distance;

                st = sensor.Locator.getStrength();
                ballAngle = sensor.Locator.getBallAngleLocal();
                lineAngle = sensor.LineSensor.getAngleDelayed();

                robotAngle = sensor.IMU.getYaw();

                if (lineAngle == 360)
                {
                    int delta_angle = constrain(cam_angle * 0.5, -20, 20);
                    drv.drive(cam_angle, delta_angle, 60);
                    menu.writeLineClean(0, "move2gate");
                    menu.writeLineClean(1, std::to_string(cam_angle));
                    menu.writeLineClean(2, std::to_string(cam_dist));
                    if (abs(cam_angle) < 10 && cam_dist < 35)
                    {
                        drv.drive(0, 0, 0, 0);
                        menu.writeLineClean(0, "WAIT");
                        menu.writeLineClean(1, "");
                        menu.writeLineClean(2, "");
                        // make_pause(1000);
                    }
                }
                else
                {
                    drv.drive(goodAngle(lineAngle + 180), 0, 60);
                    menu.writeLineClean(0, "LINE 2");
                    menu.writeLineClean(1, std::to_string(lineAngle));
                    menu.writeLineClean(2, "");
                }
            }
        }
    }
}

void goalRotate(int color){
    int sign = (sensor.IMU.getYaw() < 0) ? -1 : 1;
    while (isBall()){
        sensor.update();
        int rotateSpeed = abs(sensor.Cam.gate(color).center_angle) > 140 ? 42 : 80; // 42 : 100
        dribbler.dribble(abs(sensor.Cam.gate(color).center_angle) > 120 ? 110 : 60); // 110
        drv.drive(0, rotateSpeed * sign, 0);

        // if (omnicam().gates[color].center_angle > 0){
        //     drive(0, -40, 0);
        // }
        // else {
        //     drive(0, 40, 0);
        // }
        // Debug.SendInfo();
    }
}

void goalDriveBack(int color){
    dribbler.dribble(0);
    while (isBall()){
        sensor.update();
        drv.driveXY(0, -100, 0);
    }

    ballAngle = sensor.Locator.getBallAngleLocal();
    int sign = (sensor.IMU.getYaw() < 0) ? -1 : 1;
    while (abs(ballAngle) < 40 && !isBall()){
        sensor.update();
        ballAngle = sensor.Locator.getBallAngleLocal();
        drv.driveXY(40 * sign, 0, 0);
    }
}

void playForwardDribble2(int color)
{
    while (true)
    {
    fwDribbleBegin:

        vTaskDelay(10 / portTICK_PERIOD_MS);

        sensor.update();

        int cam_angle = -sensor.Cam.gate(color).center_angle;
        int cam_dist = sensor.Cam.gate(color).distance;

        int st = sensor.Locator.getStrength();
        int ballAngle = sensor.Locator.getBallAngleLocal();
        int lineAngle = sensor.LineSensor.getAngleDelayed();

        if (!isBall())
        {
            dribbler.dribble((abs(ballAngle) < 40) ? 40 : 0);
            int deltaAngle = ballAngle * 0.6;
            if (lineAngle == 360)
            {
                moveAngle = ballAngle * 0.5;

                drv.drive(moveAngle, (int)(deltaAngle * 0.5), 60);
            }
            else
            {
                drv.drive(goodAngle(lineAngle + 180), (int)(deltaAngle * 0.5), 80);
            }
        }
        else
        {
            dribbler.dribble(50);
            drv.drive(0, 0, 0, 0);
            int tt = millis();
            make_pause(500);
            while (isBall())
            {
                sensor.update();

                cam_angle = -sensor.Cam.gate(color).center_angle;
                cam_dist = sensor.Cam.gate(color).distance;
                lineAngle = sensor.LineSensor.getAngleDelayed();
                dribbler.dribble((abs(cam_angle) > 10 || cam_dist > 50) ? 50 : 0);

                if (lineAngle == 360)
                {
                    while (abs(cam_angle) < 130 && isBall())
                    {
                        sensor.update();
                        lineAngle = sensor.LineSensor.getAngleDelayed();
                        moveAngle = goodAngle(lineAngle + 180);
                        speed = (lineAngle == 360) ? 0 : 60;
                        cam_angle = -sensor.Cam.gate(color).center_angle;
                        deltaAngle = ((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                        deltaAngle = constrain(deltaAngle, -20, 20);
                        drv.drive(moveAngle, (int)(deltaAngle * 0.5), speed);
                    }

                    if (!isBall())
                    {
                        goto fwDribbleBegin;
                    }

                    // return;
                    int delta_angle = ((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                    delta_angle = constrain(delta_angle, -20, 20);
                    drv.drive(cam_angle, delta_angle, 80);

                    sensor.update();

                    lineAngle = sensor.LineSensor.getAngleDelayed();
                    if (lineAngle != 360)
                    {
                        drv.drive(goodAngle(lineAngle + 180), delta_angle, 80);
                        continue;
                    }
                    // Serial.println(cam_dist);
                    if (abs(cam_angle) > 100 && cam_angle != 360 && cam_dist < 90)
                    {
                        // return;
                        drv.drive(0, 0, 0, 0);
                        sensor.update();
                        dribbler.dribble(80);

                        make_pause(100);

                        cam_angle = -sensor.Cam.gate(color).center_angle;

                        while (abs(cam_angle) < 130 && isBall())
                        {
                            sensor.update();
                            cam_angle = -sensor.Cam.gate(color).center_angle;
                            deltaAngle = ((cam_angle > 0) ? -(180 - cam_angle) : -(-180 - cam_angle)) * 0.5;
                            deltaAngle = constrain(deltaAngle, -40, 40);
                            drv.drive(0, (int)(deltaAngle * 0.5), 0);
                        }

                        if (!isBall())
                        {
                            goto fwDribbleBegin;
                        }

                        make_pause(200);

                        if (abs(sensor.IMU.getYaw()) < 150 || cam_dist < 60)
                            goalRotate(color);
                        else
                            goalDriveBack(color);

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
