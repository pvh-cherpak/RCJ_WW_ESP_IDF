#include "BallSensor.h"
#include "esp_timer.h"
#include "esp_log.h"

void BallSensor_t::init(int64_t Duration, gpio_pull_mode_t pull_mode)
{
    IsBallDuration = Duration;
    gpio_reset_pin(GPIO); //  ЕСП рекомендует перед использованием сбрасывать пины
    ESP_ERROR_CHECK(gpio_set_direction(GPIO, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO, pull_mode));
}

void BallSensor_t::update()
{
    isBall = gpio_get_level(GPIO);
    if (isBall){
        lastIsBallTime = esp_timer_get_time();
    }
}

bool BallSensor_t::ballCatched()
{
    if (IsBallDuration != 0) {
        if (esp_timer_get_time() - lastIsBallTime <= IsBallDuration) // 200 ms
            return true;
        return false;
    }
    else 
        return isBall;
}

BallSensor_t::BallSensor_t(/* args */)
{
}

BallSensor_t::~BallSensor_t()
{
}
