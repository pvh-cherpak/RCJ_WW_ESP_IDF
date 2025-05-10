#include "nvs_templates.h"

const char *NVS_UMU = "mpu5060";
const char *NVS_CAM = "openMV";

bool get_variable(const char *NVS_namespace, const char *NVS_key, int16_t *value)
{
    nvs_handle_t nvs_handle;
	esp_err_t err = nvs_open(NVS_namespace, NVS_READONLY, &nvs_handle);
	if (err == ESP_ERR_NVS_NOT_FOUND)
		return false;
	else
		ESP_ERROR_CHECK(err);

	err = nvs_get_i16(nvs_handle, NVS_key, value);
	nvs_close(nvs_handle);
	if (err == ESP_ERR_NVS_NOT_FOUND)
		return false;
	else
		ESP_ERROR_CHECK(err);
	
	return true;
}

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

void get_OpenMV_offset(int16_t *offset_x, int16_t *offset_y)
{
	if(!get_variable(NVS_CAM, "x", offset_x)){
		restore_OpenMV_offset();
		esp_restart();
	}
	if(!get_variable(NVS_CAM, "y", offset_y)){
		restore_OpenMV_offset();
		esp_restart();
	}	
}

void set_OpenMV_offset(int16_t offset_x, int16_t offset_y)
{
	nvs_handle_t nvs_handle;
	ESP_ERROR_CHECK(nvs_open(NVS_CAM, NVS_READWRITE, &nvs_handle));
	ESP_ERROR_CHECK(nvs_set_i16(nvs_handle, "x", offset_x));
	ESP_ERROR_CHECK(nvs_set_i16(nvs_handle, "y", offset_y));
	nvs_commit(nvs_handle);
	nvs_close(nvs_handle);
}

void restore_OpenMV_offset()
{
	set_OpenMV_offset(0,0);
}
