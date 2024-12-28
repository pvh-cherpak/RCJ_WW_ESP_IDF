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

static const char *TAG = "example";


extern "C"
{
	void app_main(void)
	{
		start_i2c_legacy();
		senser.init();
		init_display_legacy();
		

		start_menu();
		
		esp_restart();
	}
}
