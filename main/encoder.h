#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
// #include "driver/pcnt.h"
#include "esp_sleep.h"

#define PCNT_LOW_LIMIT  -10000
#define PCNT_HIGH_LIMIT 10000

#define EXAMPLE_EC11_GPIO_A 32
#define EXAMPLE_EC11_GPIO_B 35

class Encoder_t{
private:
    int min_value = 0, max_value = 0;
    int step = 1; // на сколько меняется value за один поворот
    
    pcnt_unit_handle_t pcnt_unit = NULL;
    pcnt_unit_config_t unit_config = {
            .low_limit = PCNT_LOW_LIMIT,
            .high_limit = PCNT_HIGH_LIMIT,
        };
    
    bool can_skip = false;
    int cur_value = 0;
    int prev_pcnt = 0;

public:
    void init();
    int getCurValue();
    void setNewLimits(int new_min, int new_max, int new_step, int start_value, bool skip = false);
};

#endif