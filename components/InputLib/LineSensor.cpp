#include "LineSensor.h"
#include <string>

const char* NVS_WHITE_VALUE_GROUP = "WHITE_VALUES";
const char* NVS_GREEN_VALUE_GROUP = "GREEN_VALUES";

void LineSensor_t::init()
{
    for(int i = 0; i < 4; i++){
        gpio_reset_pin(MULT_IN[i]);
        ESP_ERROR_CHECK(gpio_set_direction(MULT_IN[i], GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_pull_mode(MULT_IN[i], GPIO_FLOATING));
        ESP_ERROR_CHECK(gpio_set_level(MULT_IN[i], 0));
    }
    gpio_reset_pin(MULT_OUT);
    ESP_ERROR_CHECK(gpio_set_direction(MULT_OUT, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(MULT_OUT, GPIO_PULLDOWN_ONLY));


    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open(NVS_WHITE_VALUE_GROUP, NVS_READWRITE, &nvs_handle));
    for(int i = 0; i < 16; i++)
      ESP_ERROR_CHECK(nvs_get_u16(nvs_handle, ("w" + std::to_string(i)).c_str(), &white_value[i]));
    nvs_close(nvs_handle);

    ESP_ERROR_CHECK(nvs_open(NVS_GREEN_VALUE_GROUP, NVS_READWRITE, &nvs_handle));
    for(int i = 0; i < 16; i++)
      ESP_ERROR_CHECK(nvs_get_u16(nvs_handle, ("g" + std::to_string(i)).c_str(), &green_value[i]));
    nvs_close(nvs_handle);

    ESP_LOGI("White values", ": %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", white_value[0], white_value[1], white_value[2], white_value[3], white_value[4], white_value[5], white_value[6], white_value[7], white_value[8], white_value[9], white_value[10], white_value[11], white_value[12], white_value[13], white_value[14], white_value[15]);
    ESP_LOGI("Green values", ": %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", green_value[0], green_value[1], green_value[2], green_value[3], green_value[4], green_value[5], green_value[6], green_value[7], green_value[8], green_value[9], green_value[10], green_value[11], green_value[12], green_value[13], green_value[14], green_value[15]);
}

/*
int readMux(int channel){
  int controlPin[] = {S0, S1, S2, S3};

  int muxChannel[16][4]={
    {0,0,0,0}, //channel 0
    {1,0,0,0}, //channel 1
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6
    {1,1,1,0}, //channel 7
    {0,0,0,1}, //channel 8
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14
    {1,1,1,1}  //channel 15
  };

  for(int i = 0; i < 4; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  return analogRead(SIG);
}
*/