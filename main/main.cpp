#include "display.h"
#include "global.h"
// #include "air_debug.h"
// #include "debug_data.h"
#include "motorControl.h"
#include "debug_log.h"

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

void nvs_set_variables();

extern "C"
{
	void app_main(void)
	{
		// nvs_set_variables();


		// NVS - Non-Volatile Storage Library, в есп нету EEPROMa поэтому в место него используется
		// флеш память, её количество можно менять поэтому существует вероятность что место зарезервиролванное под
		// данные не размечено нижестоящий код проверяет, размечена ли память под NVS и если нет пробует разметить
		// после заполняя значениями, из-за особенностей работы NVS мы работаем не с адресами а с парами ключ-значение
		// в случае если пары не созданные то при попытке чтения это вызовет ошибку которую я не хочу обрабатывать
		// потому проще заполнять пары, хоть какими-то значениями
		esp_err_t err = nvs_flash_init();
		if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND))
		{
			ESP_LOGW("NVS", "Erasing NVS partition...");
			nvs_flash_erase();
			err = nvs_flash_init();
		};
		if (err == ESP_OK)
		{
			ESP_LOGI("NVS", "NVS partition initilized");
		}
		else
		{
			esp_restart();
		};

		start_i2c_legacy();
		menu.init();
		BTDebug.init();
		sensor.init();
		drv.init();
		err_log.init();

		// // это тесты камеры
		// vTaskDelay(1000 / portTICK_PERIOD_MS);
		// OpenMVCommunication_t cam;
		// cam.init();
		// while (true)
		// {
		// 	cam.update();
		// 	ESP_LOGI("cam", "r center_angle: %d", cam.yellow.center_angle);
		// 	ESP_LOGI("cam", "r clos_angle: %d", cam.yellow.clos_angle);
		// 	ESP_LOGI("cam", "r distance: %d", cam.yellow.distance);
		// 	ESP_LOGI("cam", "r height: %d", cam.yellow.height);
		// 	ESP_LOGI("cam", "r left_angle: %d", cam.yellow.left_angle);
		// 	ESP_LOGI("cam", "r right_angle: %d", cam.yellow.right_angle);
		// 	ESP_LOGI("cam", "r width: %d", cam.yellow.width);

		// 	ESP_LOGI("cam", "r center_angle: %d", cam.blue.center_angle);
		// 	ESP_LOGI("cam", "r clos_angle: %d", cam.blue.clos_angle);
		// 	ESP_LOGI("cam", "r distance: %d", cam.blue.distance);
		// 	ESP_LOGI("cam", "r height: %d", cam.blue.height);
		// 	ESP_LOGI("cam", "r left_angle: %d", cam.blue.left_angle);
		// 	ESP_LOGI("cam", "r right_angle: %d", cam.blue.right_angle);
		// 	ESP_LOGI("cam", "r width: %d", cam.blue.width);
		// 	vTaskDelay(500 / portTICK_PERIOD_MS);
		// }

		// тест блютуза
		/*

		bool flag = false;
		while (1)
		{
			flag = !flag;
			if (flag)
				BTDebug.ddCString("GOOOO000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000L");
			else
				BTDebug.addCString("GOJDAAAAAAA");

			BTDebug.send();
			sensor.update();
			vTaskDelay(pdMS_TO_TICKS(100)); // Задержка 1 секунда
		}
		*/

		//err_log.print_all_errors();

		// int speed = 30;
		// menu.clearDisplay();
		// while (true){
		// 	//drv.drive(0, speed);
		// 	//menu.writeLineClean(0, std::to_string(speed), false);
		// 	sensor.update();

		// 	menu.writeLineClean(0, "blue: " + std::to_string(sensor.Cam.blue.center_angle));
		// 	menu.writeLineClean(1, "yellow: " + std::to_string(sensor.Cam.yellow.center_angle));
		// 	menu.writeLineClean(2, "errors: " + std::to_string(err_count));

		// 	ESP_LOGI("cam", "r center_angle: %d", sensor.Cam.yellow.center_angle);
		// 	ESP_LOGI("cam", "r height: %d", sensor.Cam.yellow.height);
		// 	ESP_LOGI("cam", "r width: %d", sensor.Cam.yellow.width);

		// 	ESP_LOGI("cam", "r center_angle: %d", sensor.Cam.blue.center_angle);
		// 	ESP_LOGI("cam", "r height: %d", sensor.Cam.blue.height);
		// 	ESP_LOGI("cam", "r width: %d", sensor.Cam.blue.width);

		// 	vTaskDelay(30 / portTICK_PERIOD_MS);
		// 	//speed = -speed;
		// }

		// vTaskDelay(5000 / portTICK_PERIOD_MS);
		// sensor.init();
		start_menu();

		vTaskDelete(NULL);
		// esp_restart();
	}
}

void nvs_set_variables()
{
	nvs_handle_t nvs_handle;
	nvs_open(NVS_WHITE_VALUE_GROUP, NVS_READWRITE, &nvs_handle);

	for (int i = 0; i < 16; i++) // символы w и g выбраны не потому что я жадный, а из-за ограничения размера ключа
		ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, ("w" + std::to_string(i)).c_str(), i));
	nvs_close(nvs_handle);

	nvs_open(NVS_GREEN_VALUE_GROUP, NVS_READWRITE, &nvs_handle);
	for (int i = 0; i < 16; i++)
		ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, ("g" + std::to_string(i)).c_str(), i));
	nvs_close(nvs_handle);
}