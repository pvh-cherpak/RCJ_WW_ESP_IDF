#include "../mpu6050.h"
#include "../LineSensor.h"

class sensor_t
{
private:

public:
    IMU_t IMU;
    LineSensor_t LineSensor;
public:
    sensor_t(/* args */){}
    ~sensor_t(){}
    
    void init(){IMU.init(); LineSensor.init();}
    void uptdate(){IMU.uptdate();}
};
