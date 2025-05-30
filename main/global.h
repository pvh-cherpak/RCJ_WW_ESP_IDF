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
#include "BTDebug.h"

const double DEG_TO_RAD = acos(-1) / 180;
const double RAD_TO_DEG = 180 / acos(-1);

// extern i2c_master_bus_handle_t i2c_bus_handle;
extern BTDebug_t BTDebug;
extern sensor_t sensor;
extern DisplayMenu_t menu;
extern MotorControl drv;
extern Dribbler dribbler;
extern Kicker kicker;
extern ErrLog_t err_log;
extern const char *NVS_IDENTIFIER_GROUP;
// extern BTDebug_t BTDebug;

// void init_i2c();
void start_i2c_legacy(void);
void init_display_legacy();

// void init_display();

#endif