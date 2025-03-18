// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//		2023-03-10 - Fit to esp-idf v5
//		2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//		2016-04-18 - Eliminated a potential infinite loop
//		2013-05-08 - added seamless Fastwire support
//				   - added note about gyro calibration
//		2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//		2012-06-20 - improved FIFO overflow handling and simplified read process
//		2012-06-19 - completely rearranged DMP initialization code and simplification
//		2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//		2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//		2012-06-05 - add gravity-compensated initial reference frame acceleration output
//				   - add 3D math helper file to DMP6 example sketch
//				   - add Euler output and Yaw/Pitch/Roll output formats
//		2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//		2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//		2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#include "mpu6050.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_err.h"
#include "driver/i2c.h"


static const char *TAG = "IMU";


#define RAD_TO_DEG (180.0/3.1415926)
#define DEG_TO_RAD 0.0174533

void IMU_t::mpu6050_init(){
	// Initialize mpu6050
	mpu.initialize();
	// Get Device ID
	uint8_t buffer[1];
	I2Cdev::readByte(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_WHO_AM_I, buffer);
	ESP_LOGI(TAG, "getDeviceID=0x%x", buffer[0]);

	// Initialize DMP
	devStatus = mpu.dmpInitialize();
	ESP_LOGI(TAG, "devStatus=%d", devStatus);
	if (devStatus != 0) {
		ESP_LOGE(TAG, "DMP Initialization failed [%d]", devStatus);
		while(1) {
			vTaskDelay(1);
		}
	}

	// This need to be setup individually
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXAccelOffset(offset[0]);
	mpu.setYAccelOffset(offset[1]);
	mpu.setZAccelOffset(offset[2]);
	mpu.setXGyroOffset(offset[3]);
	mpu.setYGyroOffset(offset[4]);
	mpu.setZGyroOffset(offset[5]);

	// Calibration Time: generate offsets and calibrate our MPU6050
	mpu.CalibrateAccel(7);
	mpu.CalibrateGyro(7);
	mpu.setDMPEnabled(true);
	ESP_LOGI(TAG, "End Calibration: %d", mpu.getZGyroOffset());
}


void IMU_t::mpu6050(){
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
		getYawPitchRoll();
		roll = ypr[2] * RAD_TO_DEG;
		pitch = ypr[1] * RAD_TO_DEG;
		yaw = ypr[0] * RAD_TO_DEG;
		i_yaw = yaw;
			//getQuaternion();
			//getEuler();
			//getRealAccel();
			//getWorldAccel();
	}

		// Best result is to match with DMP refresh rate
		// Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
		// Now its 0x13, which means DMP is refreshed with 10Hz rate
}





// display quaternion values in easy matrix form: w x y z
/*
void IMU_t::getQuaternion() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	printf("quat x:%6.2f y:%6.2f z:%6.2f w:%6.2f\n", q.x, q.y, q.z, q.w);
}
*/

// display Euler angles in degrees
/*
void IMU_t::getEuler() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetEuler(euler, &q);
	printf("euler psi:%6.2f theta:%6.2f phi:%6.2f\n", euler[0] * RAD_TO_DEG, euler[1] * RAD_TO_DEG, euler[2] * RAD_TO_DEG);
}
*/

// display Euler angles in degrees
void IMU_t::getYawPitchRoll() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	ESP_LOGV(TAG, "roll:%f pitch:%f yaw:%f",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
}


// display real acceleration, adjusted to remove gravity
/*
void IMU_t::getRealAccel() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	printf("areal x=%d y:%d z:%d\n", aaReal.x, aaReal.y, aaReal.z);
}
*/

// display initial world-frame acceleration, adjusted to remove gravity
// and rotated based on known orientation from quaternion
/*
void IMU_t::getWorldAccel() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
	printf("aworld x:%d y:%d z:%d\n", aaWorld.x, aaWorld.y, aaWorld.z);
}
*/
