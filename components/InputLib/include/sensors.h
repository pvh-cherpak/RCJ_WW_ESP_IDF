#pragma once

#include "../mpu6050.h"
#include "../LineSensor.h"
#include "../locator.h"
#include "../OpenMV.h"

#include "debug_data.h"

class sensor_t
{
private:
    gebug_data_t air_debug_data;
public:
    IMU_t IMU;
    LineSensor_t LineSensor;
    locator_t Locator;
    OpenMVCommunication_t Cam;
public:
    sensor_t(/* args */): Cam(IMU){}
    ~sensor_t(){}
    
    void init(){IMU.init(); LineSensor.init(); Locator.init(); Cam.init();}
    void update(){IMU.update(); LineSensor.update(); Locator.update(); Cam.update();}
    void testUpdate() {IMU.testUpdate(); LineSensor.testUpdate(); Locator.testUpdate();}
};
