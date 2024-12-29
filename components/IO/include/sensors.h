#include "../mpu6050.h"

class sensor_t
{
private:

public:
    IMU_t IMU;
public:
    sensor_t(/* args */){}
    ~sensor_t(){}
    
    void init(){IMU.init();}
    void uptdate(){IMU.uptdate();}
};
