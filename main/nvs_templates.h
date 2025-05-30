#pragma once

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_system.h"

#include "stdio.h"
#include "string.h"

bool get_variable(const char* NVS_namespace, const char* NVS_key, int16_t& value);

void get_MPU_offsets_blob(int16_t *offsets);
void save_MPU_offsets_blob(int16_t offsets[]);
void restore_MPU_offsets_blob();

void get_OpenMV_offset(float* offset_x, float* offset_y);
void set_OpenMV_offset(float offset_x, float offset_y);
void restore_OpenMV_offset();
