#include "LineSensor.h"
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const char *NVS_WHITE_VALUE_GROUP = "WHITE_VALUES";
const char *NVS_GREEN_VALUE_GROUP = "GREEN_VALUES";

void LineSensor_t::init()
{
  for (int i = 0; i < 4; i++)
  {
    gpio_reset_pin(MULT_IN[i]);
    ESP_ERROR_CHECK(gpio_set_direction(MULT_IN[i], GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(MULT_IN[i], GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level(MULT_IN[i], 0));
  }
  gpio_reset_pin(GPIO_NUM_25);
  ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT));
  ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO_NUM_25, GPIO_PULLDOWN_ONLY));
  ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_25, 0));

  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_2,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_mult));
  adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_mult, ADC_CHANNEL_6, &config));
  // gpio_reset_pin(MULT_OUT);
  // ESP_ERROR_CHECK(gpio_set_direction(MULT_OUT, GPIO_MODE_INPUT));
  // ESP_ERROR_CHECK(gpio_set_pull_mode(MULT_OUT, GPIO_PULLDOWN_ONLY));

  nvs_handle_t nvs_handle;
  ESP_ERROR_CHECK(nvs_open(NVS_WHITE_VALUE_GROUP, NVS_READWRITE, &nvs_handle));
  for (int i = 0; i < 16; i++)
    ESP_ERROR_CHECK(nvs_get_u16(nvs_handle, ("w" + std::to_string(i)).c_str(), &white_value[i]));
  nvs_close(nvs_handle);

  ESP_ERROR_CHECK(nvs_open(NVS_GREEN_VALUE_GROUP, NVS_READWRITE, &nvs_handle));
  for (int i = 0; i < 16; i++)
    ESP_ERROR_CHECK(nvs_get_u16(nvs_handle, ("g" + std::to_string(i)).c_str(), &green_value[i]));
  nvs_close(nvs_handle);

  ESP_LOGI("White values", ": %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", white_value[0], white_value[1], white_value[2], white_value[3], white_value[4], white_value[5], white_value[6], white_value[7], white_value[8], white_value[9], white_value[10], white_value[11], white_value[12], white_value[13], white_value[14], white_value[15]);
  ESP_LOGI("Green values", ": %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", green_value[0], green_value[1], green_value[2], green_value[3], green_value[4], green_value[5], green_value[6], green_value[7], green_value[8], green_value[9], green_value[10], green_value[11], green_value[12], green_value[13], green_value[14], green_value[15]);
}

void LineSensor_t::calibrateGreen()
{
  read_line_sensors();
  for (int i = 0; i < 16; i++)
    green_value[i] = actual_value[i];
}

void LineSensor_t::read_line_sensors()
{
  for (int channel = 0; channel < 16; channel++)
  {
    for (int bit = 0; bit < 4; bit++)
      ESP_ERROR_CHECK(gpio_set_level(MULT_IN[bit], MULT_CHANEL[channel][bit]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc_mult, ADC_CHANNEL_6, &actual_value[channel]));
  }
}

/*
void lineCalibration(){
  oled.clear();
  oled.home();
  oled.print("Waiting for click...");
  oled.update();
  enc.tick();
  while (!enc.click())
    enc.tick();
  getGreen();
  delay(100);
  getGreen();
  oled.clear();
  oled.home();
  oled.print("Green calibration done");
  oled.update();

  whiteTo0();
  
  while (!enc.click()){
    getWhite();
    enc.tick();
  }
  EEPROMSaveLines();
  oled.clear();
  oled.home();
  oled.print("White calibration done");
  oled.update();
}
*/