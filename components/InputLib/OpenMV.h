// #include <driver/i2c.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "cam_types.h"

#include "mpu6050.h"

struct point_t{
    int x = 0, y = 0;
};

struct segm_t{
    point_t beg, en;
    int a = 0, b = 0, c = 0;
};

struct blob_t{
    point_t p[4];
};

int local_good_angle(int angle);

segm_t segm_from_points(point_t p1, point_t p2);
int point_dist(point_t p1, point_t p2);

int dist_to_segm(point_t p, segm_t s);

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

    float dist_offset_x = 0, dist_offset_y = 0;

    int center_x = 160, center_y = 120;
public:
    void init(int GPIO);
    void update();
    OpenMVCommunication_t(IMU_t& IMUU);
    ~OpenMVCommunication_t();
    const OmniCamBlobInfo_t& gate(int color) { return (color ? Blue : Yellow); };
private:
    void parseData(uint8_t *data);
    void parseCorners(uint8_t *data);
    void calculate_global_values();
    void calcGateInfo(blob_t blob, OmniCamBlobInfo_t& gate);
    
    int dist_to_polygon(int angle, blob_t b);
private:
    OmniCamData_t cam_data;
    OmniCamData_t globa_cam_data;
    IMU_t& IMU;
};