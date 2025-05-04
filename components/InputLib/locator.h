#pragma once

#include <driver/i2c.h>
#include "esp_log.h"

class locator_t
{
private:
    const uint8_t LOCATOR_ADDRESS = 0x0E;
    const TickType_t I2C_TIMEOUT_TIME_TICS = 100 / portTICK_PERIOD_MS;

public:
    void init(int offset);
    void update();
    void testUpdate(){ball_angle = rand() % 360; ESP_LOGI("Locator", "r center_angle: %d", ball_angle);}
    int getBallAngleLocal() { return ball_angle;}
    int getStrength() { return strength; }

    locator_t(/* args */);
    ~locator_t();
public:
    const int& BallAngleLocal = ball_angle;

private:
    int ball_angle = 0;
    int strength = 0;
    int offset = 0;
private:
    uint8_t ReadHeading_1200();
    uint8_t ReadHeading_600();
    uint8_t ReadStrenght_1200();
    uint8_t ReadStrenght_600();  
};