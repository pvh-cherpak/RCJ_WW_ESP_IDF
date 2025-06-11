#include "locator.h"

void locator_t::init(int offset, bool inverse)
{
    this->inverse=inverse;
    this->offset=offset;
    uint8_t command = 0x00;
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_NUM_0, LOCATOR_ADDRESS, &command, 1, I2C_TIMEOUT_TIME_TICS));
    for (int i = 0; i < BALL_ANGLE_BUF_LEN; ++i)
        ball_angle_buf[i] = 0;
}

void locator_t::update()
{
    int angle;
    if (ReadStrenght_600() < 15){
        angle = ReadHeading_1200();
        strength = ReadStrenght_1200();
    }
    else{
        angle = ReadHeading_600();
        strength = ReadStrenght_1200();
    }
    int dir = 360 - (5 * angle) - offset;
    if (dir > 180)
        dir -= 360;
    if(dir < -180)
        dir += 360;
    // if (dir == -180)
    // return lastBallSeenAngle - mpuGetDegree();
    // lastBallSeenAngle = dir + mpuGetDegree();
    if(inverse)
        ball_angle = -dir;
    else
        ball_angle = dir;

    if (abs(ball_angle) > 180)
        ball_angle = 360;
    else{
        // int angle_sum = 0;
        // for (int i = 0; i < BALL_ANGLE_BUF_LEN - 1; ++i){
        //     ball_angle_buf[i] = ball_angle_buf[i + 1];
        //     angle_sum += ball_angle_buf[i];
        // }
        // ball_angle_buf[BALL_ANGLE_BUF_LEN - 1] = ball_angle;
        // angle_sum += ball_angle;

        // ball_angle = angle_sum / BALL_ANGLE_BUF_LEN;
    }
}

uint8_t locator_t::ReadHeading_1200()
{
    uint8_t heading;
    uint8_t command = 0x04;
    i2c_master_write_read_device(I2C_NUM_0, LOCATOR_ADDRESS, &command, 1, &heading, 1, I2C_TIMEOUT_TIME_TICS);
    return heading;
}

uint8_t locator_t::ReadHeading_600()
{
    uint8_t heading;
    uint8_t command = 0x06;
    i2c_master_write_read_device(I2C_NUM_0, LOCATOR_ADDRESS, &command, 1, &heading, 1, I2C_TIMEOUT_TIME_TICS);
    return heading;
}

uint8_t locator_t::ReadStrenght_1200()
{
    uint8_t heading;
    uint8_t command = 0x05;
    i2c_master_write_read_device(I2C_NUM_0, LOCATOR_ADDRESS, &command, 1, &heading, 1, I2C_TIMEOUT_TIME_TICS);
    return heading;
}

uint8_t locator_t::ReadStrenght_600()
{
    uint8_t heading;
    uint8_t command = 0x07;
    i2c_master_write_read_device(I2C_NUM_0, LOCATOR_ADDRESS, &command, 1, &heading, 1, I2C_TIMEOUT_TIME_TICS);
    return heading;
}

locator_t::locator_t(/* args */)
{
}

locator_t::~locator_t()
{
}