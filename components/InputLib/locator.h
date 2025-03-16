#include <driver/i2c.h>

class locator_t
{
private:
    const uint8_t LOCATOR_ADDRESS = 0x0E;
    const TickType_t I2C_TIMEOUT_TIME_TICS = 100 / portTICK_PERIOD_MS;

public:
    void init();
    void update();
    int getBallAngleLocal() { return ball_angle;}
    const int& BallAngleLocal = ball_angle; 

    locator_t(/* args */);
    ~locator_t();

private:
    int ball_angle = 0;

private:
    uint8_t ReadHeading_1200();
    uint8_t ReadHeading_600();
    uint8_t ReadStrenght_1200();
    uint8_t ReadStrenght_600();  
};