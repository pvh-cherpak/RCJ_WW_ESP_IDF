#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
// #include "driver/pcnt.h"
#include "esp_sleep.h"

#define EXAMPLE_PCNT_HIGH_LIMIT 100
#define EXAMPLE_PCNT_LOW_LIMIT  -100

#define EXAMPLE_EC11_GPIO_A 32
#define EXAMPLE_EC11_GPIO_B 35

int encoder_pos();

void encoder_init(int item_count);

void encoder_set_max_value(int new_max_value, int new_act_value);