#include "ssd1306.h"

#include "driver/i2c_master.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

extern i2c_master_bus_handle_t i2c_bus_handle;
extern SSD1306_t display;

void init_i2c();
void init_display();