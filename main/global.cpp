#include "global.h"

#include "esp_log.h"

#define I2C_MASTER_FREQ_HZ 400000 // I2C clock of SSD1306 can run at 400 kHz max.
#define I2C_TICKS_TO_WAIT 100	  // Maximum ticks to wait before issuing a timeout.
static const char *I2C_tag = "I2C";

i2c_master_bus_handle_t i2c_bus_handle = NULL;
SSD1306_t display;

void init_i2c(){
    ESP_LOGI(I2C_tag, "New i2c driver is used");
	i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_SCL_GPIO,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
        .flags = { 
            .enable_internal_pullup = true 
            }
	};
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle));
}

void init_display(){
i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = I2C_ADDRESS,
		.scl_speed_hz = I2C_MASTER_FREQ_HZ,
	};
	i2c_master_dev_handle_t i2c_dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_handle));

    // int16_t reset = CONFIG_RESET_GPIO;
	// if (reset >= 0) {
	// 	//gpio_pad_select_gpio(reset);
	// 	gpio_reset_pin(reset);
	// 	gpio_set_direction(reset, GPIO_MODE_OUTPUT);
	// 	gpio_set_level(reset, 0);
	// 	vTaskDelay(50 / portTICK_PERIOD_MS);
	// 	gpio_set_level(reset, 1);
	// }

	display._address = I2C_ADDRESS;
	display._flip = false;
	display._i2c_num = I2C_NUM_0;
	display._i2c_bus_handle = i2c_bus_handle;
	display._i2c_dev_handle = i2c_dev_handle;

    #if CONFIG_FLIP
	    display._flip = true;
	    ESP_LOGW(I2C_tag, "Flip upside down");
    #endif


    #if CONFIG_SSD1306_128x64
	ESP_LOGI(I2C_tag, "Panel is 128x64");
	ssd1306_init(&display, 128, 64);
    #endif // CONFIG_SSD1306_128x64
}