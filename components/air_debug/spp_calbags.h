#pragma once

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

void esp_spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

bool can_write(uint32_t *handle);