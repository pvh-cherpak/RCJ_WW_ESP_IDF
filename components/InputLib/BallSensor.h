#include "driver/gpio.h"

class BallSensor_t
{
private:
    const gpio_num_t GPIO = GPIO_NUM_33;
private:
    int isBall = false;
public:
    void init();
    void update();

    const int &IsBall = isBall;
    
    BallSensor_t(/* args */);
    ~BallSensor_t();
};
