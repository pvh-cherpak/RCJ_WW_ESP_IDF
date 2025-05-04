#include "BallSensor.h"

void BallSensor_t::init()
{
    gpio_reset_pin(GPIO); //  ЕСП рекомендует перед использованием сбрасывать пины
    ESP_ERROR_CHECK(gpio_set_direction(GPIO, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO, GPIO_FLOATING));
}

void BallSensor_t::update()
{
    isBall = gpio_get_level(GPIO);
}

BallSensor_t::BallSensor_t(/* args */)
{
}

BallSensor_t::~BallSensor_t()
{
}
