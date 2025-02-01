#include "../mpu6050.h"
#include "../LineSensor.h"
#include "../locator.h"
#include "../OpenMV.h"

class sensor_t
{
private:
    
public:
    IMU_t IMU;
    LineSensor_t LineSensor;
    locator_t Locator;
    OpenMVCommunication_t Cam;
public:
    sensor_t(/* args */){}
    ~sensor_t(){}
    
    void init(){IMU.init(); LineSensor.init(); Locator.init(); Cam.init();}
    void uptdate(){IMU.uptdate(); LineSensor.update(); Locator.update(); Cam.update();}
};
