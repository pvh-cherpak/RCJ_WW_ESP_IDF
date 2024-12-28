#include "encoder.h"

static int max_pos = 0;

static const char *TAG = "encoder";
static pcnt_unit_handle_t pcnt_unit = NULL;
static pcnt_unit_config_t unit_config = {
        .low_limit = -(max_pos * 4 + 1),
        .high_limit = max_pos * 4 + 1,
    };


int encoder_pos(){
    int pulse_count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
    int pos = pulse_count / 4;
    if (pos < 0)
        pos += max_pos + 1;
    
    if (pos > max_pos)
        pos = max_pos;
    ESP_LOGV(TAG, "encoder position: %d", pos);

    return pos; 
}

void encoder_init(int item_count)
{
    if (pcnt_unit != NULL){
        pcnt_unit_disable(pcnt_unit);
        pcnt_del_unit(pcnt_unit);
        pcnt_unit = NULL;
    }

    max_pos = item_count;
    unit_config = {
        .low_limit = -(max_pos * 4 + 4),
        .high_limit = max_pos * 4 + 4,
    };

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
        .level_gpio_num = EXAMPLE_EC11_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
        .level_gpio_num = EXAMPLE_EC11_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // ESP_LOGI(TAG, "add watch points and register callbacks");
    // int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT};
    // for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
    //     ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    // }
    // pcnt_event_callbacks_t cbs = {
    //     .on_reach = example_pcnt_on_reach,
    // };
    // QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    // ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif
  
}


// void encoder_set_max_value(int new_max_value, int new_act_value){
//     max_pos = new_max_value;

//     ESP_ERROR_CHECK(pcnt_set_event_value(pcnt_unit, PCNT_EVT_L_LIM, -max_pos*4)); 
//     ESP_ERROR_CHECK(pcnt_event_enable(pcnt_unit, PCNT_EVT_L_LIM)); // Обновляем максимальный лимит 
    
//     ESP_ERROR_CHECK(pcnt_set_event_value(pcnt_unit, PCNT_EVT_H_LIM, max_pos*4)); 
    
//     ESP_ERROR_CHECK(pcnt_event_enable(pcnt_unit, PCNT_EVT_H_LIM)); // Обновляем конфигурацию 

//     ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));

//     pcnt_counter_set_value(pcnt_unit, new_act_value * 4);
// }
