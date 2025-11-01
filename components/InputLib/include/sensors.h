#pragma once

#include "../mpu6050.h"
#include "../LineSensor.h"
#include "../locator.h"
#include "../OpenMV.h"
#include "../BallSensor.h"
#include "../locator.h"
#include "../LightGates.h"

#include "esp_log.h"

struct sensor_config_t
{
    LineSensor_config_t LineSensor_config;
    int CAM_GPIO;
    uint8_t robotType;
    int locator_offset = 0;
    bool IMU_active = true;
    bool inverse_locator = false;
    int16_t *offsets = nullptr;
};

class sensor_t
{
public:
    IMU_t IMU;
    LineSensor_t LineSensor;
    locator_t Locator;
    OpenMVCommunication_t Cam;
    BallSensor_t BallSensor;
    LightGates_t LightGates;

    bool IMU_active = true;
public:
    sensor_config_t cfg;

    sensor_t():Cam(IMU){}
    ~sensor_t(){}
    
    void init(sensor_config_t config){
        cfg = config;
        IMU_active = config.IMU_active;
        
        ESP_LOGI("sensor init", "start IMU init");
        if (IMU_active)
            IMU.init(config.offsets);
        
        ESP_LOGI("sensor init", "start LineSensor init");
        LineSensor.init(config.LineSensor_config);

        ESP_LOGI("sensor init", "start Locator init");
        Locator.init(config.locator_offset, config.inverse_locator); 

        ESP_LOGI("sensor init", "start Camera init");
        Cam.init(config.CAM_GPIO, 30);

        if (cfg.robotType == 2)
            BallSensor.init();
        else if (cfg.robotType == 1)
            LightGates.init(GPIO_NUM_39);
    }
    void update(){
        if(IMU_active)
            IMU.update();
        LineSensor.update(); Locator.update(); Cam.update();
        if (cfg.robotType == 2)
            BallSensor.update();
        else if (cfg.robotType == 1)
            LightGates.update();
    }
    void testUpdate() {IMU.testUpdate(); LineSensor.testUpdate(); Locator.testUpdate();}
};
