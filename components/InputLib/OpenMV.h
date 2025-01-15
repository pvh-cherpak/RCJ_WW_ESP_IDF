#include <driver/i2c.h>
#include "esp_log.h"

struct OmniCamBlobInfo_t
{
    int16_t left_angle = 0;
    int16_t right_angle = 0;
    int16_t center_angle = 0;
    int16_t width = 0;
    int16_t clos_angle = 0;
    int16_t distance = 0;
    int16_t height = 0;
};
struct OmniCamData_t
{
    OmniCamBlobInfo_t gates[2];
};

class OpenMVCommunication_t
{
private:
    const uint8_t CAM_ADDRESS = 0x42;
    const TickType_t I2C_TIMEOUT_TIME_TICS = 50 / portTICK_PERIOD_MS;

public:
    const OmniCamData_t &camDataOmni = cam_data;
    const OmniCamBlobInfo_t &yelow = cam_data.gates[0];
    const OmniCamBlobInfo_t &blue = cam_data.gates[1];

public:
    void init();
    void update();
    OpenMVCommunication_t(/* args */);
    ~OpenMVCommunication_t();

private:
    OmniCamData_t cam_data;
};