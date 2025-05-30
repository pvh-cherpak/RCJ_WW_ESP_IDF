#pragma once

#include "../mpu6050.h"
#include "../LineSensor.h"
#include "../locator.h"
#include "../OpenMV.h"
#include "../BallSensor.h"
#include "../locator.h"

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

    bool IMU_active = true;
public:
    sensor_config_t cfg;

    sensor_t():Cam(IMU){}
    ~sensor_t(){}
    
    void init(sensor_config_t config){
        cfg = config;
        IMU_active = config.IMU_active;
        if (IMU_active)
            IMU.init(config.offsets);
        LineSensor.init(config.LineSensor_config); Locator.init(config.locator_offset, config.inverse_locator); 
        Cam.init(config.CAM_GPIO);
        BallSensor.init();
    }
    void update(){
        if(IMU_active)
            IMU.update();
        LineSensor.update(); Locator.update(); Cam.update();
        BallSensor.update();
    }
    void testUpdate() {IMU.testUpdate(); LineSensor.testUpdate(); Locator.testUpdate();}
};
