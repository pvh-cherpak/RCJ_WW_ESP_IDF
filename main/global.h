#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#include "ssd1306.h"

// #include "driver/i2c_master.h"
#include "driver/i2c.h" // deprecated
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "sensors.h"
#include "motorControl.h"
#include "display.h"
#include "debug_log.h"


// extern i2c_master_bus_handle_t i2c_bus_handle;
extern sensor_t sensor;
extern DisplayMenu_t menu;
extern MotorControl drv;
extern ErrLog_t err_log;

// void init_i2c();
void start_i2c_legacy(void);
void init_display_legacy();

// void init_display();

#endif