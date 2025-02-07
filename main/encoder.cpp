#include "encoder.h"

static const char *TAG = "encoder";

int Encoder_t::getCurValue(){
    int cur_pcnt = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &cur_pcnt));

    // prev_pcnt меняется только на число, кратное 4, чтобы избежать проблем из-за шума

    // если cur_pcnt больше максимального значения, двигаем prev_pcnt вверх
    int max_possible_pcnt = prev_pcnt + (max_value - cur_value) / step * 4;
    if (cur_pcnt > max_possible_pcnt)
        prev_pcnt += (cur_pcnt - max_possible_pcnt) / 4 * 4;
    
    // если cur_pcnt меньше минимального значения, двигаем prev_pcnt вниз
    int min_possible_pcnt = prev_pcnt - (cur_value - min_value) / step * 4;
    if (cur_pcnt < min_possible_pcnt)
        prev_pcnt -= (min_possible_pcnt - cur_pcnt) / 4 * 4;

    int complete = (cur_pcnt - prev_pcnt) / 4; // количество накопленных сдвигов энкодера

    if (can_skip){
        // можно пропускать значения при быстрой перемотке
        cur_value += complete * step;
    }
    else{
        // значение меняется не более чем на 1 за кадр
        if (complete >= 1)
            cur_value += step;
        if (complete <= -1)
            cur_value -= step;
    }

    ESP_LOGI(TAG, "cur_value: %d (+ %d * %d)", cur_value, complete, step);

    prev_pcnt += complete * 4;
    
    return cur_value;
}

void Encoder_t::setNewLimits(int new_min, int new_max, int new_step, int start_value, bool skip){
    ESP_LOGI(TAG, "setNewLimits(min=%d, max=%d, step=%d, start=%d)", new_min, new_max, new_step, start_value);

    min_value = new_min;
    max_value = new_max;
    step = new_step;
    can_skip = skip;

    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));

    prev_pcnt = 0;
    cur_value = start_value;
}

void Encoder_t::init()
{
    if (pcnt_unit != NULL){
        pcnt_unit_disable(pcnt_unit);
        pcnt_del_unit(pcnt_unit);
        pcnt_unit = NULL;
    }

    min_value = 0;
    max_value = 0;
    step = 0;

    unit_config = {
        .low_limit = PCNT_LOW_LIMIT,
        .high_limit = PCNT_HIGH_LIMIT,
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
