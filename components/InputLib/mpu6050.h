#pragma once
#include "mpu6050.h"
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

//#include "MPU6050.h" // not necessary if using MotionApps include file
#include "MPU6050_6Axis_MotionApps20.h"


    class IMU_t
    {
    private:
        int16_t offset[6] = {-183, -3592, 1408, 41, -45, -46};
        //XAxes YAxes ZAxes XGyro YDyro ZGyro
        float quatx;
        float quaty;
        float quatz;
        float quatw;
        float roll;
        float pitch;
        float yaw;

        /////////////////////////////////////////////////////////////////////////////////////////////////
        //я не разбирался зачем нужны эти переменные но пусть будут
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        // MPU control/status vars
        bool dmpReady = false;  // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer

        // orientation/motion vars
        Quaternion q;        // [w, x, y, z]			quaternion container
        VectorInt16 aa;      // [x, y, z]			accel sensor measurements
        VectorInt16 aaReal;  // [x, y, z]			gravity-free accel sensor measurements
        VectorInt16 aaWorld; // [x, y, z]			world-frame accel sensor measurements
        VectorFloat gravity; // [x, y, z]			gravity vector
        float euler[3];      // [psi, theta, phi]	Euler angle container
        float ypr[3];        // [yaw, pitch, roll]	yaw/pitch/roll container and gravity vector

        // packet structure for InvenSense teapot demo
        uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};
        //////////////////////////////////////////////////////////////////////////////////////
        //сюда же идут тестовые функции непонятно для чего, скорее всего это тупо примеры
        // я их наверное все закоменчу но внесу в класс
        //////////////////////////////////////////////////////////////////////////////////
        // void getWorldAccel();
        // void getRealAccel();
        // void getEuler();
        // void getQuaternion()
    private:
        MPU6050 mpu;
        void mpu6050_init();
        void mpu6050();
        void mpu6059_calibrate();
        void getYawPitchRoll();

    public:
        int getYaw(){return yaw;} 
        void init(){mpu6050_init();}
        void uptdate(){mpu6050();}
        IMU_t(){}
    };

