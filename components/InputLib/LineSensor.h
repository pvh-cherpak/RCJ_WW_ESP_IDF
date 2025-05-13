#include "driver/gpio.h"
#include "sdkconfig.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

extern const char *NVS_WHITE_VALUE_GROUP;
extern const char *NVS_GREEN_VALUE_GROUP;

struct LineSensor_config_t
{
    gpio_num_t mult_in[4];
    adc_unit_t ADC_unit;
    adc_channel_t ADC_chanel;
    bool stupid_pin;
    bool inversed_without_offset;
};


class LineSensor_t
{
private: // константы
    const TickType_t LINE_RETENTION_TIME_TICS = 1500 / portTICK_PERIOD_MS;
    const float RAD_TO_DEG = 180.0f / acos(-1);
    const float DEG_TO_RAD = acos(-1) / 180.0f;
    const uint32_t MULT_CHANEL[16][4] = {
        {0, 0, 0, 0}, //channel 0
        {1, 0, 0, 0}, //channel 1
        {0, 1, 0, 0}, //channel 2
        {1, 1, 0, 0}, //channel 3
        {0, 0, 1, 0}, //channel 4
        {1, 0, 1, 0}, //channel 5
        {0, 1, 1, 0}, //channel 6
        {1, 1, 1, 0}, //channel 7
        {0, 0, 0, 1}, //channel 8
        {1, 0, 0, 1}, //channel 9
        {0, 1, 0, 1}, //channel 10
        {1, 1, 0, 1}, //channel 11
        {0, 0, 1, 1}, //channel 12
        {1, 0, 1, 1}, //channel 13
        {0, 1, 1, 1}, //channel 14
        {1, 1, 1, 1}  //channel 15
    };

    LineSensor_config_t CONFIG;
public:
    LineSensor_t(LineSensor_config_t config)
        :CONFIG(config){}
    LineSensor_t():CONFIG{{(gpio_num_t)13,
        (gpio_num_t)12,
        (gpio_num_t)26,
        (gpio_num_t)27}, ADC_UNIT_2, ADC_CHANNEL_6, false, false}{}
    
    void init(LineSensor_config_t config);
    void update(){read_line_sensors(); saveLineDirection(); calculateLineAngle();}
    void testUpdate(){line_angle_delayed = rand() % 360; is_line_on_sensor[12]=!is_line_on_sensor[12]; ESP_LOGI("LineSensor TEST", "Angle: %d", line_angle_delayed);}
    // int getAngle();

    void calibrateGreen();
    void calibrateWhite();
    void whiteTo0();
    void saveGreenWhite();

    
    int getAngleDelayed(){return line_angle_delayed;};
    void getDirectionDelayed(float& x, float& y) { x = line_dir_x; y = line_dir_y; }
    // void getLineAngleAvg();
    void writeValues(){ESP_LOGI("Line values", ": %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", actual_value[0], actual_value[1], actual_value[2], actual_value[3], actual_value[4], actual_value[5], actual_value[6], actual_value[7], actual_value[8], actual_value[9], actual_value[10], actual_value[11], actual_value[12], actual_value[13], actual_value[14], actual_value[15]);}
    ~LineSensor_t() {}
public:
    bool is_line_on_sensor[16];
    int &LineAngleDelayed = line_angle_delayed;
private:
    void saveLineDirection();
    bool isLineOnSensor(int sensor);
    void read_line_sensors();
    void getLineDirection_Delayed(float& x, float& y);
    void calculateLineAngle();
private:
    // const gpio_num_t MULT_OUT = (gpio_num_t)CONFIG_GPIO_MULTIPLEXER_OUT;
    adc_oneshot_unit_handle_t adc_mult;

    uint16_t green_value[16];
    uint16_t white_value[16];
    int actual_value[16];
    
    TickType_t line_time [16];

    bool is_line_detected = false;
    int line_angle_delayed = 0;
    float line_dir_x = 0, line_dir_y = 0;
};
