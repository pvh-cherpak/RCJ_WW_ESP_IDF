#include "display.h"
#include "global.h"
// #include "air_debug.h"
// #include "debug_data.h"
#include "motorControl.h"
#include "debug_log.h"
#include "pictures.h"

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
#include "nvs_templates.h"

extern const char *NVS_WHITE_VALUE_GROUP;
extern const char *NVS_GREEN_VALUE_GROUP;
extern const char *NVS_IDENTIFIER_GROUP;

void nvs_set_variables(uint8_t robot_type);
// 1 - goalkepper 2 - forward
uint8_t get_identifier();

void sensor_init(uint8_t robot_type)
{
	sensor_config_t conf;

	int16_t offset[6];
	get_MPU_offsets_blob(offset);
	conf.offsets = offset;

	// ESP_LOGI("MPU", "offsets_pointer: %p", conf.offsets);
	ESP_LOGI("MPU", "XAxes YAxes ZAxes XGyro YDyro ZGyro");
	ESP_LOGI("MPU", "offsets: %d, %d, %d, %d, %d, %d", offset[0], offset[1], offset[2], offset[3], offset[4], offset[5]);

	float x, y;
	get_OpenMV_offset(&x, &y);
	sensor.Cam.dist_offset_x = x;
	sensor.Cam.dist_offset_y = y;
	//ESP_LOGI("GP", "dist offsets:  %d, %d", sensor.Cam.dist_offset_x, sensor.Cam.dist_offset_y);

	if (robot_type == 1)
	{ //keeper
		conf.LineSensor_config = {
			{(gpio_num_t)13,
			 (gpio_num_t)12,
			 (gpio_num_t)26,
			 (gpio_num_t)27},
			ADC_UNIT_2,
			ADC_CHANNEL_6,
			true,
			false,
			true};
		conf.robotType = robot_type;
		conf.CAM_GPIO = 36;
		conf.locator_offset = 0;
		conf.IMU_active = true;
		conf.inverse_locator = false;


		sensor.init(conf);
	}
	else
	{ //forward
		conf.LineSensor_config = {{GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_28, GPIO_NUM_27}, ADC_UNIT_2, ADC_CHANNEL_6, false, true, true};
		conf.CAM_GPIO = 19;
		conf.robotType = robot_type;
		conf.locator_offset = 90;
		conf.IMU_active = false;
		conf.inverse_locator = true;
		conf.LineSensor_config.offset = 22;

		// sensor.init(conf);

		drv.~MotorControl();
		new (&drv) MotorControl(GPIO_NUM_33, GPIO_NUM_32, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_17, GPIO_NUM_16);

		// sensor.IMU_active=false;
		// sensor.Cam.init(conf.CAM_GPIO);
		// sensor.LineSensor.init(conf.LineSensor_config);
		// sensor.Locator.init(conf.locator_offset);
		// sensor.BallSensor.init();
		// sensor.cfg = conf;
	}
}

extern "C"
{
	void app_main(void)
	{
		// nvs_set_variables(1);
		

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

		// sensor.LineSensor.init({{GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_13, GPIO_NUM_12}, ADC_UNIT_2, ADC_CHANNEL_6, false, true});
		// while (true){
		// 	sensor.LineSensor.update();
		// 	sensor.LineSensor.writeValues();
		// 	vTaskDelay(100 / portTICK_PERIOD_MS);
		// }
		
		// gpio_reset_pin(GPIO_NUM_39);
		// adc_oneshot_unit_handle_t adc_ball;
		// adc_oneshot_unit_init_cfg_t init_config1 = {
		// 	.unit_id = ADC_UNIT_1,
		// 	.ulp_mode = ADC_ULP_MODE_DISABLE,
		// };
		// ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_ball));

		// adc_oneshot_chan_cfg_t ADC_config = {
		// 	.atten = ADC_ATTEN_DB_12,
		// 	.bitwidth = ADC_BITWIDTH_DEFAULT,
		// };

		// ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_ball, ADC_CHANNEL_3, &ADC_config));

		// while (true){
		// 	int ballValue;
		// 	ESP_ERROR_CHECK(adc_oneshot_read(adc_ball, ADC_CHANNEL_3, &ballValue));
		// 	ESP_LOGI("Balls", "%d", ballValue);
		// 	// drv.drive(50, 50, 50, 50);
		// 	vTaskDelay(100 / portTICK_PERIOD_MS);
		// }

		start_i2c_legacy();
		menu.init();
		menu.clearDisplay();
		// nvs_set_variables(2);

		uint8_t robot_type = get_identifier();
		if (!robot_type)
			esp_restart();
		sensor_init(robot_type);

		if (robot_type == 1)
			menu.showPicture(0, 0, shet, 128, 64, true);
		else if (robot_type == 2)
			menu.showPicture(0, 0, mechi, 128, 64, false);
		else
		{
			menu.writeLineClean(0, "Unknown robot type");
			vTaskDelay(1000);
			esp_restart();
		}

		// BTDebug.init();

		drv.init();
		if (robot_type == 2)
			dribbler.init();
		else if (robot_type == 1)
			kicker.init(GPIO_NUM_23);
		//err_log.init();
		real_dist.init();

		int GPIO_A, GPIO_B;
		if (robot_type == 1)
		{
			GPIO_A = 32;
			GPIO_B = 35;
		}
		else
		{
			GPIO_A = 36;
			GPIO_B = 39;
		}

		// drv.drive(50, 50, 0, 0);

		// while (true){
		// 	sensor.LineSensor.update();
		// 	sensor.LineSensor.writeValues();
		// 	vTaskDelay(500 / portTICK_PERIOD_MS);
		// }

		start_menu(robot_type, GPIO_A, GPIO_B);

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
		BTDebug.init();
		bool flag = false;
		while (1)
		{
			for (int y = -110; y < 110; y += 10)
				for (int x = -80; x < 80; x += 10)
				{
					flag = !flag;
					if (flag)
						BTDebug.addCString("GOOOO000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000L");
					else
						BTDebug.addCString("GOJDAAAAAAA");
					
					BTDebug.setPosition(x, y);
					BTDebug.send();
					sensor.testUpdate();
					
				}
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

		vTaskDelete(NULL);
		// esp_restart();
	}
}

void nvs_set_variables(uint8_t robot_type)
{
	nvs_handle_t nvs_handle;
	esp_err_t err;

	nvs_open(NVS_WHITE_VALUE_GROUP, NVS_READWRITE, &nvs_handle);
	for (int i = 0; i < 16; i++)
		// символы w и g выбраны не потому что я жадный, а из-за ограничения размера ключа
		ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, ("w" + std::to_string(i)).c_str(), i));
	nvs_commit(nvs_handle);
	nvs_close(nvs_handle);

	nvs_open(NVS_GREEN_VALUE_GROUP, NVS_READWRITE, &nvs_handle);
	for (int i = 0; i < 16; i++)
		// символы w и g выбраны не потому что я жадный, а из-за ограничения размера ключа
		ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, ("g" + std::to_string(i)).c_str(), i));
	nvs_commit(nvs_handle);
	nvs_close(nvs_handle);

	ESP_ERROR_CHECK(nvs_open(NVS_IDENTIFIER_GROUP, NVS_READWRITE, &nvs_handle));
	ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "robot_type", robot_type));
	nvs_commit(nvs_handle);
	nvs_close(nvs_handle);

	restore_MPU_offsets_blob();
}

uint8_t get_identifier()
{
	uint8_t robot_type = 0;

	nvs_handle_t nvs_handle;

	nvs_open(NVS_IDENTIFIER_GROUP, NVS_READWRITE, &nvs_handle);
	esp_err_t err = nvs_get_u8(nvs_handle, "robot_type", &robot_type);
	if (err == ESP_OK)
		return robot_type;
	nvs_close(nvs_handle);

	if (err == ESP_ERR_NVS_NOT_FOUND)
	{
		nvs_set_variables(2);
		return 2;
	}

	return 0;
}
