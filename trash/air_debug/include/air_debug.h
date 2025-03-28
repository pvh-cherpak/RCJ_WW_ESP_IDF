#pragma once

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

extern QueueHandle_t bt_queue;

void bt_task(void *arg);

void init_bluetooth(void);

void prepair_buff( gebug_data_t& msg);