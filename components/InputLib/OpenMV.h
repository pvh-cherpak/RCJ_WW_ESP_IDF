// #include <driver/i2c.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "cam_types.h"

#include "mpu6050.h"

class OpenMVCommunication_t
{
private:
    // const uint8_t CAM_ADDRESS = 0x42;
    // const TickType_t I2C_TIMEOUT_TIME_TICS = 50 / portTICK_PERIOD_MS;
    const uart_port_t uart_num = UART_NUM_1;
    int GPIO_CAM_UART = 36;
    const TickType_t UART_READ_TIMEOUT_TIME_TICS = 50 / portTICK_PERIOD_MS;
public:
    const OmniCamData_t &CamDataOmni = cam_data;
    const OmniCamBlobInfo_t &Yellow = cam_data.Gates[0];
    const OmniCamBlobInfo_t &Blue = cam_data.Gates[1];

    const OmniCamData_t &GlobalCamDataOmni = globa_cam_data;
    const OmniCamBlobInfo_t &GlobalYellow = globa_cam_data.Gates[0];
    const OmniCamBlobInfo_t &GlobalBlue = globa_cam_data.Gates[1];

    float obst_angle = 0, obst_dist = 0;
    float g_obst_angle = 0, g_obst_dist = 0;

    int16_t dist_offset_x = 0, dist_offset_y = 0;
public:
    void init(int GPIO);
    void update();
    OpenMVCommunication_t(IMU_t& IMUU);
    ~OpenMVCommunication_t();
    const OmniCamBlobInfo_t& gate(int color) { return (color ? Blue : Yellow); };
private:
    void parseData(uint8_t *data);
    void calculate_global_values();
private:
    OmniCamData_t cam_data;
    OmniCamData_t globa_cam_data;
    IMU_t& IMU;
};