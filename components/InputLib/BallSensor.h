#include "driver/gpio.h"

class BallSensor_t
{
private:
    const gpio_num_t GPIO = GPIO_NUM_36;
private:
    int isBall = false;
    int64_t lastIsBallTime = 0;
    int64_t IsBallDuration = 500 * 100;
public:
    void init(int64_t Duration = 500 * 100, gpio_pull_mode_t pull_mode = GPIO_FLOATING);
    void update();
    bool ballCatched();

    const int &IsBall = isBall;
    
    BallSensor_t();
    ~BallSensor_t();
};
