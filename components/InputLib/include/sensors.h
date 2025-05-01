#pragma once

#include "../mpu6050.h"
#include "../LineSensor.h"
#include "../locator.h"
#include "../OpenMV.h"
#include "../BallSensor.h"

struct sensor_config_t
{
    LineSensor_config_t LineSensor_config;
    int CAM_GPIO;
    uint8_t robotType;
};


class sensor_t
{
public:
    IMU_t IMU;
    LineSensor_t LineSensor;
    locator_t Locator;
    OpenMVCommunication_t Cam;
    BallSensor_t BallSensor;
public:
    sensor_t():Cam(IMU){}
    ~sensor_t(){}
    
    void init(sensor_config_t config){/*IMU.init();*/ LineSensor.init(config.LineSensor_config); Locator.init(); 
        Cam.init(config.CAM_GPIO);}
    void update(){/*IMU.update();*/ LineSensor.update(); Locator.update(); Cam.update();}
    void testUpdate() {IMU.testUpdate(); LineSensor.testUpdate(); Locator.testUpdate();}
};
