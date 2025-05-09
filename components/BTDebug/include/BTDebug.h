#pragma once

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "sensors.h"

#include "string"
#include <limits.h>

int16_t f_coordinate2int16(float coord);

class BTDebug_t
{
private:
    static const int16_t mask1 = 0b1111111100000000;
    static const int16_t mask2 = 0b0000000011111111;

    static const int string_buf_start_pos = 44;
    static const int buffer_size = 1023 + string_buf_start_pos;
private:
    uint8_t outbuf[buffer_size + 1];

    sensor_t &sensor;

    int str_end_pos = string_buf_start_pos;

private:
    bool active = false;
    uint8_t state = 0;
    int16_t pos_x = 0;
    int16_t pos_y = 0;
public: 
    BTDebug_t(sensor_t &sensors);
    ~BTDebug_t();

    void setState(uint8_t state) { this->state = state; }

    void addCString(const char *s);

    void addString(const std::string &s);

    void setPosition(float x, float y) {pos_x = f_coordinate2int16(x); pos_y =f_coordinate2int16(y);};

    void init();

    void send();

public:
    void prepair_sensor_buff();
};
