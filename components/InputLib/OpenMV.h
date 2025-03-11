// #include <driver/i2c.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "cam_types.h"

class OpenMVCommunication_t
{
private:
    // const uint8_t CAM_ADDRESS = 0x42;
    // const TickType_t I2C_TIMEOUT_TIME_TICS = 50 / portTICK_PERIOD_MS;
    const uart_port_t uart_num = UART_NUM_1; // использую только для тестов, так как нету паяльника
    const int GPIO_CAM_UART = 36;
    const TickType_t UART_READ_TIMEOUT_TIME_TICS = 50 / portTICK_PERIOD_MS;
public:
    const OmniCamData_t &camDataOmni = cam_data;
    const OmniCamBlobInfo_t &yellow = cam_data.gates[0];
    const OmniCamBlobInfo_t &blue = cam_data.gates[1];

public:
    void init();
    void update();
    OpenMVCommunication_t(/* args */);
    ~OpenMVCommunication_t();
    const OmniCamBlobInfo_t& gate(int color) { return (color ? blue : yellow); };
private:
    void parseData(uint8_t *data);
private:
    OmniCamData_t cam_data;
};