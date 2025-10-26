#include "driver/gpio.h"

class BallSensor_t
{
private:
    const gpio_num_t GPIO = GPIO_NUM_36;
private:
    int isBall = false;
    int lastIsBallTime = 0;
public:
    void init();
    void update();
    bool ballCatched();

    const int &IsBall = isBall;
    
    BallSensor_t(/* args */);
    ~BallSensor_t();
};
