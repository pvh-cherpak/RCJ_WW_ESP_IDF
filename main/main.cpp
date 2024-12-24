#include "display.h"

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

#include <stdlib.h>
#include "esp_log.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include <string>
#include <vector>

static const char *I2C_tag = "I2C";

extern "C" {
void app_main(void)
{
	SSD1306_t display;

#if CONFIG_I2C_INTERFACE
	ESP_LOGI(I2C_tag, "INTERFACE is i2c");
	ESP_LOGI(I2C_tag, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
	ESP_LOGI(I2C_tag, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
	ESP_LOGI(I2C_tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
	i2c_master_init(&display, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_I2C_INTERFACE

#if CONFIG_FLIP
	display._flip = true;
	ESP_LOGW(I2C_tag, "Flip upside down");
#endif

#if CONFIG_SSD1306_128x64
	ESP_LOGI(I2C_tag, "Panel is 128x64");
	ssd1306_init(&display, 128, 64);
#endif // CONFIG_SSD1306_128x64
	

    esp_restart();
}
}




