#include "logics.h"
#include "esp_timer.h"

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

int criticalObstacleAngle = 150;
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

float gb_kp = 1.;
float gb_ki = 0;
float gb_kd = 15;
float gkBallIntegral = 0;
float gkBallPrev = 0;

float gkReactOnBallDiap = 0;

// Вратарь по камере
int lastGateAngle = 180;
int lastGateTime = 0;
int lastGateReset = 5000;

float gate_kp = 1;
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

int millis(){
    return micros() / 1000;
}

int constrain(int val, int minim, int maxim){
    if (val < minim)
        return minim;
    if (val > maxim)
        return maxim;
    return val;
}

int goodAngle(int angle) {
    angle %= 360;
    if (angle < -180)
        angle += 360;
    if (angle > 180)
        angle -= 360;
    return angle;
}

void make_pause(int ms){
    int start = millis();
    while (millis() - start > 0){
        sensor.update();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void projectSpeedOnLineXY(int speedX, int speedY, float lineX, float lineY, float& resSpeedX, float& resSpeedY)
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

void projectSpeedOnLine(float speed, float moveAngle, float lineX, float lineY, float& resSpeedX, float& resSpeedY)
{
    int speedX = (int)(speed * sin(moveAngle * DEG_TO_RAD));
    int speedY = (int)(speed * cos(moveAngle * DEG_TO_RAD));
    projectSpeedOnLineXY(speedX, speedY, lineX, lineY, resSpeedX, resSpeedY);
}

void killerFeature(int color)
{
    while (true){
        vTaskDelay(10 / portTICK_PERIOD_MS);

        if (stateGame != 1){
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
            gateAngle = (int)goodAngle(gateAngle + 180);
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

void playGoalkeeperCamera(int color)
{
    menu.clearDisplay();
    while (true){
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

        ballAngle = sensor.Locator.getBallAngleLocal();
        int robotAngle = sensor.IMU.getYaw();

        float lineX, lineY;
        //getLineDirection_Delayed(lineX, lineY, true);
        sensor.LineSensor.getDirectionDelayed(lineX, lineY);

        int gateAngle = goodAngle(sensor.Cam.gate(color).center_angle + 180);        
        int globalGateAngle = goodAngle(gateAngle + robotAngle);
        int cam_height = sensor.Cam.gate(color).height;
        
        menu.writeLineClean(1, "GK " + std::to_string(gateAngle) + "  " + std::to_string(cam_height));

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
            int err = cam_height - 90;
            speedY = (int)(err * gate_kp + (err - gatePrev) * gate_kd + gateIntegral);
            speedY = (int)constrain(speedY, -limitGateSpeed, 100);
            gatePrev = err;
            gateIntegral += (err * gate_ki);
            gateIntegral = constrain(gateIntegral, -limitGateIntegral, limitGateIntegral);

            if (abs(ballAngle) > 80 && err < 0)
            {
                deltaAngle = (gateAngle == 360) ? -robotAngle * 0.5 : -(int)goodAngle(180 - gateAngle) * 0.25;
                ball_strength = sensor.Locator.getStrength();
                if (ballAngle > 0){
                    moveAngle = ballAngle + constrain(ball_strength * goRoundBallCoefGk, 0, 90);
                }
                else{
                    moveAngle = ballAngle - constrain(ball_strength * goRoundBallCoefGk, 0, 90);
                }
                moveAngle = goodAngle(moveAngle);
                drv.drive(moveAngle, (int)deltaAngle, (int)(50));
                //Debug.Log("go round");
                continue;
            }
        }

        ESP_LOGI("playGoalkeeperCamera", "speedY = %d", speedY);

        ballAngle = constrain(ballAngle, -maxGoalkeeperAngle - robotAngle, maxGoalkeeperAngle - robotAngle);
        float ballSpeed = gb_kp * ballAngle + gkBallIntegral + gb_kd * (ballAngle - gkBallPrev);
        // if (ball_strength > prevBallStrength){
        //     ballSpeed += constrain((ball_strength - prevBallStrength) * gk_st_kd, -50, 50);
        // }
        gkBallIntegral += (ballAngle) * gb_ki;
        gkBallPrev = ballAngle;
        speedX += (int)(ballSpeed);

        deltaAngle = -(int)goodAngle(180 - gateAngle) * 0.25;

        // prevBallStrength = ball_strength;

        // speedX = constrain(speedX, -50, 50);
        // speedY = constrain(speedY, -50, 50);
        int sp = sqrt(speedX * speedX + speedY * speedY);
        if (sp > 100){
            speedX = speedX * 100 / sp;
            speedY = speedY * 100 / sp;
        }

        drv.driveXY(speedX, speedY, (int)deltaAngle);
    }
}

