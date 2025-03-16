#include "display.h"
#include "global.h"
#include "air_debug.h"
#include "debug_data.h"

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
		};
		if (err == ESP_OK)
		{
			ESP_LOGI("NVS", "NVS partition initilized");
		}
		else
		{
			esp_restart();
		};

		// // это тесты камеры
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		/*OpenMVCommunication_t cam;
		cam.init();
		while (true)
		{
			cam.update();
			ESP_LOGI("cam", "r center_angle: %d", cam.yellow.center_angle);
			ESP_LOGI("cam", "r clos_angle: %d", cam.yellow.clos_angle);
			ESP_LOGI("cam", "r distance: %d", cam.yellow.distance);
			ESP_LOGI("cam", "r height: %d", cam.yellow.height);
			ESP_LOGI("cam", "r left_angle: %d", cam.yellow.left_angle);
			ESP_LOGI("cam", "r right_angle: %d", cam.yellow.right_angle);
			ESP_LOGI("cam", "r width: %d", cam.yellow.width);

			ESP_LOGI("cam", "r center_angle: %d", cam.blue.center_angle);
			ESP_LOGI("cam", "r clos_angle: %d", cam.blue.clos_angle);
			ESP_LOGI("cam", "r distance: %d", cam.blue.distance);
			ESP_LOGI("cam", "r height: %d", cam.blue.height);
			ESP_LOGI("cam", "r left_angle: %d", cam.blue.left_angle);
			ESP_LOGI("cam", "r right_angle: %d", cam.blue.right_angle);
			ESP_LOGI("cam", "r width: %d", cam.blue.width);
			vTaskDelay(500 / portTICK_PERIOD_MS);
		}*/


		// тест блютуза
		bt_queue = xQueueCreate(10, sizeof(gebug_data_t));
		if (bt_queue == NULL)
		{
			ESP_LOGE(TAG, "Failed to create queue");
			return;
		}

		// Инициализация Bluetooth
		init_bluetooth();

		// Запуск задачи Bluetooth на Core 1
		xTaskCreatePinnedToCore(bt_task, "Bluetooth Task", 4096, NULL, 5, NULL, 0);

		gebug_data_t msg;
		msg.ball_angle = 260;
		msg.is_ball = false;
		while (1)
		{
			msg.ball_angle++;
			msg.speed_x =  rand() % 128;
			msg.speed_y =  rand() % 128;
			if (msg.ball_angle > 360)
				msg.ball_angle = -100;
			for(int i =0 ; i < 16; i++)
				msg.line_sensor[i] = rand() % 2;
			
			int64_t start_time = esp_timer_get_time();
			xQueueSend(bt_queue, &msg, 0);
			int64_t end_time = esp_timer_get_time();
            int64_t elapsed_time = end_time - start_time;
            ESP_LOGI(TAG, "Vrema dobavlenia v ochered %llu", elapsed_time);

			vTaskDelay(pdMS_TO_TICKS(1000)); // Задержка 1 секунда
		}




		// vTaskDelay(5000 / portTICK_PERIOD_MS);
		// sensor.init();
		// start_menu();

		vTaskDelete(NULL);
		// esp_restart();
	}
}
