#pragma once

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_system.h"

int16_t* get_MPU_offsets_blob();
void save_MPU_offsets_blob(int16_t offsets[]);
void restore_MPU_offsets_blob();
