#include "DribblerMicroswitch.h"

#include "esp_log.h"

DribblerMicroswitch_t::DribblerMicroswitch_t(/* args */)
{
}

void DribblerMicroswitch_t::init(int pin, uint16_t hold_time, bool active_state)
{
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = hold_time,
        .short_press_time = 0,
        .gpio_button_config = {
            .gpio_num = pin,
            .active_level = active_state,
        },
    };

    microswitch = iot_button_create(&gpio_btn_cfg);
    if (NULL == microswitch)
    {
        ESP_LOGE("DribblerMicroswitch", "Button create failed");
    }

}

bool DribblerMicroswitch_t::ballCatched()
{
    return  iot_button_get_event(microswitch) == BUTTON_LONG_PRESS_HOLD;
}

DribblerMicroswitch_t::~DribblerMicroswitch_t()
{
}

