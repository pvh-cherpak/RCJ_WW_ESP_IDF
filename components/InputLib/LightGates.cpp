#include "LightGates.h"

void LightGates_t::init(gpio_num_t pin_num){
    lightPin = pin_num;

    gpio_reset_pin(lightPin);

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = adc_unit_light,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_light));

    adc_oneshot_chan_cfg_t ADC_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_light, adc_channel_light, &ADC_config));
}

void LightGates_t::update(){
    int luminosity;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_light, adc_channel_light, &luminosity));
    isBallValue = (luminosity >= isBallThreshold);
}

bool LightGates_t::isBall(){
    return isBallValue;
}