#ifndef _LIGHT_GATES_
#define _LIGHT_GATES_

#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_timer.h"

class LightGates_t{
    gpio_num_t lightPin = GPIO_NUM_36;
	adc_oneshot_unit_handle_t adc_light;
    adc_unit_t adc_unit_light = ADC_UNIT_1;
    adc_channel_t adc_channel_light = ADC_CHANNEL_0;
    int isBallThreshold = 200;
    
    bool isBallValue = false;
    int64_t lastIsBallTime = 0;

    public:
        void init(gpio_num_t pin_num);
        void update();
        bool isBall();
        bool ballCatched();
};

#endif