#include "display.h"
#include "global.h"

/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"

#include <stdlib.h>
#include "esp_log.h"
#include <string>
#include <vector>

#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "example";

extern const char *NVS_WHITE_VALUE_GROUP;
extern const char *NVS_GREEN_VALUE_GROUP;

extern "C"
{
	void app_main(void)
	{

		// start_i2c_legacy();
		// senser.init();
		// init_display_legacy();

		esp_err_t err = nvs_flash_init();
		if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND))
		{
			ESP_LOGW("NVS", "Erasing NVS partition...");
			nvs_flash_erase();
			err = nvs_flash_init();
			if (err == ESP_OK)
			{
				nvs_handle_t nvs_handle;
				nvs_open(NVS_WHITE_VALUE_GROUP, NVS_READWRITE, &nvs_handle);
				for (int i = 0; i < 16; i++)
					ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, ("w" + std::to_string(i)).c_str(), i));
				nvs_close(nvs_handle);

				nvs_open(NVS_GREEN_VALUE_GROUP, NVS_READWRITE, &nvs_handle);
				for (int i = 0; i < 16; i++)
					ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, ("g" + std::to_string(i)).c_str(), i));
				nvs_close(nvs_handle);
			}
		};
		if (err == ESP_OK)
		{
			ESP_LOGI("NVS", "NVS partition initilized");
		}
		else
		{
			ESP_LOGE("NVS", "NVS partition initialization error: %d (%s)", err, esp_err_to_name(err));
		};

		// 
		// ESP_LOGI("GROUP NAME: ", "%s", NVS_WHITE_VALUE_GROUP);
		// nvs_open(NVS_WHITE_VALUE_GROUP, NVS_READWRITE, &nvs_handle);
		// for (int i = 0; i < 16; i++)
		// 	ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, ("w" + std::to_string(i)).c_str(), i));
		// nvs_close(nvs_handle);

		// nvs_open(NVS_GREEN_VALUE_GROUP, NVS_READWRITE, &nvs_handle);
		// for (int i = 0; i < 16; i++)
		// 	ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, ("g" + std::to_string(i)).c_str(), i));
		// nvs_close(nvs_handle);

		senser.LineSensor.init();
		
		while (true)
		{
			senser.LineSensor.update();
			senser.LineSensor.wrightValues();
			vTaskDelay(100/portTICK_PERIOD_MS);
		}
		

		// start_menu();
		vTaskDelete(NULL);
		// esp_restart();
	}
}
