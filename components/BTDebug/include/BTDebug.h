#pragma once

#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "sensors.h"

class BTDebug_t
{
private:
    sensor_t& sensor;
public:
    BTDebug_t(sensor_t &sensors);
    ~BTDebug_t();

    void init();
    void send();
public:
    void prepair_sensor_buff();
};
