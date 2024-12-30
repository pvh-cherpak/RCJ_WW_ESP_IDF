#include "driver/gpio.h"
#include "sdkconfig.h"
#include "nvs.h"
#include "esp_log.h"

extern const char* NVS_WHITE_VALUE_GROUP;
extern const char* NVS_GREEN_VALUE_GROUP;

class LineSensor_t
{
public:
    void init();
    void update();
    int getAngle();
    int getAngleDelayed();
    LineSensor_t(/* args */){}
    ~LineSensor_t(){}
private:
    const gpio_num_t MULT_IN[4] = {(gpio_num_t)CONFIG_GPIO_MULTIPLEXER_IN_0,
     (gpio_num_t)CONFIG_GPIO_MULTIPLEXER_IN_1,
     (gpio_num_t)CONFIG_GPIO_MULTIPLEXER_IN_2,
     (gpio_num_t)CONFIG_GPIO_MULTIPLEXER_IN_3};
    const gpio_num_t MULT_OUT = (gpio_num_t)CONFIG_GPIO_MULTIPLEXER_OUT;

    uint16_t green_value[16];
    uint16_t white_value[16];
    uint16_t actual_value[16];
};

