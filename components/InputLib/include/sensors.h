#pragma once

#include "../mpu6050.h"
#include "../LineSensor.h"
#include "../locator.h"
#include "../OpenMV.h"
#include "../BallSensor.h"
#include "../locator.h"
#include "../LightGates.h"
#include "../DribblerMicroswitch.h"

#include "esp_log.h"

#define mS_to_uS(x) x * 100

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
    DribblerMicroswitch_t DribblerMicroswitch;

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

        if (cfg.robotType == 2) // forward
            // BallSensor.init(mS_to_uS(0), GPIO_PULLDOWN_ONLY);
            DribblerMicroswitch.init(36, 5, 1);
        else if (cfg.robotType == 1)
            BallSensor.init(mS_to_uS(1000));
    }
    void update(){
        if(IMU_active)
            IMU.update();
        LineSensor.update(); Locator.update(); Cam.update();
        
        if (cfg.robotType == 1) 
            BallSensor.update();
        // if (cfg.robotType == 2)
        //     BallSensor.update();
        // else if (cfg.robotType == 1)
        //     LightGates.update();
    }
    void testUpdate() {IMU.testUpdate(); LineSensor.testUpdate(); Locator.testUpdate();}
};
