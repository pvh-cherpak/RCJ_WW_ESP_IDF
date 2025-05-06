#include "BallSensor.h"
#include "esp_timer.h"
#include "esp_log.h"

void BallSensor_t::init()
{
    gpio_reset_pin(GPIO); //  ЕСП рекомендует перед использованием сбрасывать пины
    ESP_ERROR_CHECK(gpio_set_direction(GPIO, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO, GPIO_FLOATING));
}

void BallSensor_t::update()
{
    isBall = gpio_get_level(GPIO);
    if (isBall){
        ESP_LOGI("ball", " %d", (int)esp_timer_get_time());
        lastIsBallTime = esp_timer_get_time();
    }
}

bool BallSensor_t::ballCatched()
{
    if (esp_timer_get_time() - lastIsBallTime <= 10000000) // 200 ms
        return true;
    return false;
}

BallSensor_t::BallSensor_t(/* args */)
{
}

BallSensor_t::~BallSensor_t()
{
}
