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

class LineSensor_t
{
private: // константы
    const TickType_t LINE_RETENTION_TIME_TICS  = 1000 / portTICK_PERIOD_MS;
    const float RAD_TO_DEG = 180.0f / acos(-1);
    const float DEG_TO_RAD = acos(-1) / 180.0f;
public:
    void init();
    void update(){read_line_sensors(); saveLineDirection(); calculateLineAngle();}
    // int getAngle();

    // ничего лучше чем скопипастить Глебовы функции я не придумал
    // но они мне нравятся, надо как-то передлать
    void calibrateGreen();
    void calibrateWhite();
    void whiteTo0();
    void saveGreenWhite();
    // 
    
    int getAngleDelayed(){return line_angle_delayed;};
    // void getLineAngleAvg();
    void wrightValues(){ESP_LOGI("Line values", ": %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", actual_value[0], actual_value[1], actual_value[2], actual_value[3], actual_value[4], actual_value[5], actual_value[6], actual_value[7], actual_value[8], actual_value[9], actual_value[10], actual_value[11], actual_value[12], actual_value[13], actual_value[14], actual_value[15]);}
    LineSensor_t(/* args */) {}
    ~LineSensor_t() {}

private:
    void saveLineDirection();
    bool isLineOnSensor(int sensor);
    void read_line_sensors();
    void getLineDirection_Delayed(float& x, float& y);
    void calculateLineAngle();
private:
    // предполагалось, что для более удобного переезда на другую платформу все датчики можно будет
    // конфигурировать через конфиг ...
    const gpio_num_t MULT_IN[4] = {(gpio_num_t)CONFIG_GPIO_MULTIPLEXER_IN_0,
                                   (gpio_num_t)CONFIG_GPIO_MULTIPLEXER_IN_1,
                                   (gpio_num_t)CONFIG_GPIO_MULTIPLEXER_IN_2,
                                   (gpio_num_t)CONFIG_GPIO_MULTIPLEXER_IN_3};
    // const gpio_num_t MULT_OUT = (gpio_num_t)CONFIG_GPIO_MULTIPLEXER_OUT;
    adc_oneshot_unit_handle_t adc_mult;

    uint16_t green_value[16];
    uint16_t white_value[16];
    int actual_value[16];
    TickType_t line_time [16];
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

    bool is_line_detected = false;
    int line_angle_delayed = 0;
};
