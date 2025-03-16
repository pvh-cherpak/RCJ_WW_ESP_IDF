#pragma once

#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "sensors.h"

#include "string"

class BTDebug_t
{
private:
    sensor_t& sensor;

    uint8_t state = 0;
    char str[1024];
    int str_pos = 0;
public:
    BTDebug_t(sensor_t &sensors);
    ~BTDebug_t();

    void setState(uint8_t state){this->state = state;}
    /*
    void addString(string& s){
        int s_size = s.size();
        if(s_size + str_pos > 1024)
            return;
        for(int i = 0; i < 1024; str_pos++)
            str[str_pos + i] = s[i];
        str_pos+=s_size + 1;
    }
        */

    void init();
    void send();
public:
    void prepair_sensor_buff();
};
