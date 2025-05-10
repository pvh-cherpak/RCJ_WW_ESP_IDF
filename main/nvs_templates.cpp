#include "nvs_templates.h"

const char *NVS_UMU = "mpu5060";

void get_MPU_offsets_blob(int16_t *offsets)
{

    nvs_handle_t nvs_handle;
	esp_err_t err = nvs_open(NVS_UMU, NVS_READONLY, &nvs_handle);
	if (err == ESP_ERR_NVS_NOT_FOUND){
		restore_MPU_offsets_blob();
		esp_restart();
	}
	else
		ESP_ERROR_CHECK(err);
		
	size_t length = sizeof(int16_t) * 6;
	err = nvs_get_blob(nvs_handle, "offsets", offsets, &length);
	nvs_close(nvs_handle);
	if (err == ESP_ERR_NVS_NOT_FOUND){
		restore_MPU_offsets_blob();
		esp_restart();
	}
	else
		ESP_ERROR_CHECK(err);
}

void save_MPU_offsets_blob(int16_t offsets[])
{
	nvs_handle_t nvs_handle;
	ESP_ERROR_CHECK(nvs_open(NVS_UMU, NVS_READWRITE, &nvs_handle));
	ESP_ERROR_CHECK (nvs_set_blob(nvs_handle, "offsets", offsets, sizeof(int16_t) * 6));
	nvs_commit(nvs_handle);
	nvs_close(nvs_handle);
}

void restore_MPU_offsets_blob()
{
	int16_t offset[6] = {-183, -3592, 1408, 41, -45, -46};
    //XAxes YAxes ZAxes XGyro YDyro ZGyro
	save_MPU_offsets_blob(offset);
}

