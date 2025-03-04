#include "display.h"
#include "global.h"
#include "air_debug.h"
#include "debug_data.h"
#include "motorControl.h"

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

extern QueueHandle_t bt_queue;

extern "C"
{
	void app_main(void)
	{

		start_i2c_legacy();
		init_display_legacy();

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
			if (err == ESP_OK)
			{
				nvs_handle_t nvs_handle;
				nvs_open(NVS_WHITE_VALUE_GROUP, NVS_READWRITE, &nvs_handle);
				// ещё одна особеность все пары ключ-значение должны быть разбиты на группы
				// коеми являются NVS_WHITE_VALUE_GROUP, NVS_GREEN_VALUE_GROUP
				for (int i = 0; i < 16; i++) // символы w и g выбраны не потому что я жадный, а из-за ограничения размера ключа
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
		// bt_queue = xQueueCreate(10, sizeof(gebug_data_t));
		// if (bt_queue == NULL)
		// {
		// 	ESP_LOGE(TAG, "Failed to create queue");
		// 	return;
		// }

		// // Инициализация Bluetooth
		// init_bluetooth();

		// // Запуск задачи Bluetooth на Core 1
		// xTaskCreatePinnedToCore(bt_task, "Bluetooth Task", 4096, NULL, 5, NULL, 0);

		// gebug_data_t msg;
		// msg.ball_angle = 260;
		// msg.is_ball = false;
		// while (1)
		// {
		// 	msg.ball_angle++;
		// 	if (msg.ball_angle > 360)
		// 		msg.ball_angle = -100;
		// 	for(int i =0 ; i < 16; i++)
		// 		msg.line_sensor[i] = rand() % 2;
		// 	// snprintf(msg.message, sizeof(msg.message), "Data: %d", esp_random() % 100);
		// 	xQueueSend(bt_queue, &msg, portMAX_DELAY);

		// 	vTaskDelay(pdMS_TO_TICKS(1000)); // Задержка 1 секунда
		// }

		sensor.init();
		drv.init();

		int speed = 30;
		menu.clearDisplay();
		while (true){
			//drv.drive(0, speed);
			//menu.writeLineClean(0, std::to_string(speed), false);
			sensor.update();
			menu.writeLineClean(0, "blue: " + std::to_string(sensor.Cam.blue.center_angle));
			menu.writeLineClean(1, "yellow: " + std::to_string(sensor.Cam.yellow.center_angle));
			// ESP_LOGI("cam", "yellow: %d", sensor.Cam.yellow.center_angle);
			// ESP_LOGI("cam", "blue: %d", sensor.Cam.yellow.center_angle);
			
			ESP_LOGI("cam", "r center_angle: %d", sensor.Cam.yellow.center_angle);
			ESP_LOGI("cam", "r clos_angle: %d", sensor.Cam.yellow.clos_angle);
			ESP_LOGI("cam", "r distance: %d", sensor.Cam.yellow.distance);
			ESP_LOGI("cam", "r height: %d", sensor.Cam.yellow.height);
			ESP_LOGI("cam", "r left_angle: %d", sensor.Cam.yellow.left_angle);
			ESP_LOGI("cam", "r right_angle: %d", sensor.Cam.yellow.right_angle);
			ESP_LOGI("cam", "r width: %d", sensor.Cam.yellow.width);

			ESP_LOGI("cam", "r center_angle: %d", sensor.Cam.blue.center_angle);
			ESP_LOGI("cam", "r clos_angle: %d", sensor.Cam.blue.clos_angle);
			ESP_LOGI("cam", "r distance: %d", sensor.Cam.blue.distance);
			ESP_LOGI("cam", "r height: %d", sensor.Cam.blue.height);
			ESP_LOGI("cam", "r left_angle: %d", sensor.Cam.blue.left_angle);
			ESP_LOGI("cam", "r right_angle: %d", sensor.Cam.blue.right_angle);
			ESP_LOGI("cam", "r width: %d", sensor.Cam.blue.width);
			vTaskDelay(500 / portTICK_PERIOD_MS);

			vTaskDelay(50 / portTICK_PERIOD_MS);
			//speed = -speed;
		}


		// vTaskDelay(5000 / portTICK_PERIOD_MS);
		motor_drv8833_task();
		start_menu();

		vTaskDelete(NULL);
		// esp_restart();
	}
}
