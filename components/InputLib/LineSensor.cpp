#include "LineSensor.h"
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const char *NVS_WHITE_VALUE_GROUP = "WHITE_VALUES";
const char *NVS_GREEN_VALUE_GROUP = "GREEN_VALUES";

void LineSensor_t::init(LineSensor_config_t config)
{
  CONFIG = config;

  for(int i = 0; i < 16; i++)
     actual_value[i] = 0;
  
  for (int i = 0; i < 4; i++)
  {
    gpio_reset_pin(CONFIG.mult_in[i]); //  ЕСП рекомендует перед использованием сбрасывать пины
    ESP_ERROR_CHECK(gpio_set_direction(CONFIG.mult_in[i], GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(CONFIG.mult_in[i], GPIO_FLOATING)); // я не особо понял, надо ли настраивать подтяжку на output порты но няхай будет
    ESP_ERROR_CHECK(gpio_set_level(CONFIG.mult_in[i], 0));
  }
  // какойто непонятный разрещаюший сигнал, гениальная трата бесценных ножек GPIO
  if(CONFIG.stupid_pin){
    gpio_reset_pin(GPIO_NUM_25);
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(GPIO_NUM_25, GPIO_PULLDOWN_ONLY));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_25, 0));
  }

  // У есп есть 2 АЦП каждый можно комутировать на определённый диапазон прописанный в доках
  // для нашего пина нужен 2
  // ulp_mode это режим колибровки показаний АЦП, я думаю оно нам не надо
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = CONFIG.ADC_unit,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_mult));
  // atten - Атеньюатор, рабочекрестьянским языком уменьшитель сигнала (это не я вас принижаю, в душе не ебу как это работает, думается мне что это не просто делитель напряжения)
  // bitwidth - разрядность в битах, чем больше битов тем выше точность ADC_BITWIDTH_DEFAULT должен обеспечивать максимальную точность на какую способен АЦП
  adc_oneshot_chan_cfg_t ADC_config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  // у АЦП не ножки а каналы у нас по таблице ADC_CHANNEL_6
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_mult, CONFIG.ADC_chanel, &ADC_config));
  // gpio_reset_pin(MULT_OUT);
  // ESP_ERROR_CHECK(gpio_set_direction(MULT_OUT, GPIO_MODE_INPUT));
  // ESP_ERROR_CHECK(gpio_set_pull_mode(MULT_OUT, GPIO_PULLDOWN_ONLY));

  // Чиатаем значения белого и зелёного, что почему и как я в мейне распинался
  bool need_to_comit = false;
  nvs_handle_t nvs_handle;
  ESP_ERROR_CHECK(nvs_open(NVS_WHITE_VALUE_GROUP, NVS_READWRITE, &nvs_handle));
  for (int i = 0; i < 16; i++)
  {
    esp_err_t err = nvs_get_u16(nvs_handle, ("w" + std::to_string(i)).c_str(), &white_value[i]);

    if (err == ESP_OK)
      continue;

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
      err = nvs_set_u16(nvs_handle, ("w" + std::to_string(i)).c_str(), i);
      if (err == ESP_OK)
        need_to_comit = true;
      else
        ESP_ERROR_CHECK(err);
    }
  }
  if(need_to_comit)
    nvs_commit(nvs_handle);
  nvs_close(nvs_handle);

  need_to_comit = false;
  ESP_ERROR_CHECK(nvs_open(NVS_GREEN_VALUE_GROUP, NVS_READWRITE, &nvs_handle));
  for (int i = 0; i < 16; i++)
  {
    esp_err_t err = nvs_get_u16(nvs_handle, ("g" + std::to_string(i)).c_str(), &green_value[i]);

    if (err == ESP_OK)
      continue;

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
      err = nvs_set_u16(nvs_handle, ("g" + std::to_string(i)).c_str(), i);
      if (err == ESP_OK)
        need_to_comit = true;
      else
        ESP_ERROR_CHECK(err);
    }
  }
  if(need_to_comit)
    nvs_commit(nvs_handle);
  nvs_close(nvs_handle);

  // согласитесь это ШЕДЕВР
  ESP_LOGI("White values", ": %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", white_value[0], white_value[1], white_value[2], white_value[3], white_value[4], white_value[5], white_value[6], white_value[7], white_value[8], white_value[9], white_value[10], white_value[11], white_value[12], white_value[13], white_value[14], white_value[15]);
  ESP_LOGI("Green values", ": %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", green_value[0], green_value[1], green_value[2], green_value[3], green_value[4], green_value[5], green_value[6], green_value[7], green_value[8], green_value[9], green_value[10], green_value[11], green_value[12], green_value[13], green_value[14], green_value[15]);
  // return corection_counter;
}

void LineSensor_t::calibrateGreen()
{
  read_line_sensors();
  for (int i = 0; i < 16; i++)
    green_value[i] = actual_value[i];
}

void LineSensor_t::calibrateWhite()
{
  read_line_sensors();
  for (int i = 0; i < 16; i++)
    white_value[i] = std::max((int)white_value[i], actual_value[i]);
}

void LineSensor_t::whiteTo0()
{
  for (int i = 0; i < 16; i++)
    white_value[i] = 0;
}

void LineSensor_t::saveGreenWhite()
{
  nvs_handle_t nvs_handle;
  ESP_ERROR_CHECK(nvs_open(NVS_WHITE_VALUE_GROUP, NVS_READWRITE, &nvs_handle));
  for (int i = 0; i < 16; i++)
    ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, ("w" + std::to_string(i)).c_str(), white_value[i]));
  nvs_commit(nvs_handle);
  nvs_close(nvs_handle);

  ESP_ERROR_CHECK(nvs_open(NVS_GREEN_VALUE_GROUP, NVS_READWRITE, &nvs_handle));
  for (int i = 0; i < 16; i++)
    ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, ("g" + std::to_string(i)).c_str(), green_value[i]));
  nvs_commit(nvs_handle);
  nvs_close(nvs_handle);
}

void LineSensor_t::calculateLineAngle()
{
  getLineDirection_Delayed(line_dir_x, line_dir_y);
  if (line_dir_x == 0 && line_dir_y == 0){
    line_angle_delayed = 360;
    return;
  }
  line_angle_delayed = atan2(line_dir_x, line_dir_y) * RAD_TO_DEG;
}

bool LineSensor_t::isLineOnSensor(int sensor)
{
  is_line_on_sensor[sensor] = ((actual_value[sensor] - green_value[sensor]) >= ((white_value[sensor] - green_value[sensor]) / 2));
  return is_line_on_sensor[sensor];
}

void LineSensor_t::saveLineDirection()
{
  bool mass[16];
  is_line_detected = false;
  for (int i = 0; i < 16; i++){
    mass[i] = 0;
    if (isLineOnSensor(i) && (white_value[i] - green_value[i] > 700))
    {
      line_time[i] = xTaskGetTickCount();
      is_line_detected = true;
      mass[i] = 1;
    }
  }
  // ESP_LOGI("Line  ", ": %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", mass[0], mass[1], mass[2], mass[3], mass[4], mass[5], mass[6], mass[7], mass[8], mass[9], mass[10], mass[11], mass[12], mass[13], mass[14], mass[15]);
}

void LineSensor_t::getLineDirection_Delayed(float &x, float &y)
{
  if (!is_line_detected){
    x = 0;
    y = 0;
    return;
  }
  float sumX = 0;
  float sumY = 0;
  int k = 0;

  for (int i = 0; i < 16; i++)
    if (line_time[i] != portMAX_DELAY && xTaskGetTickCount() - line_time[i] < LINE_RETENTION_TIME_TICS)
    {
      int delay = /*LINE_RETENTION_TIME_TICS - */(xTaskGetTickCount() - line_time[i]) + 1;
      float ang;
      if(CONFIG.inversed_without_offset)
        ang = i * 22.5f;
      else
        ang = (16 - i) * 22.5f;
      k += delay;
      sumX += sin(ang * DEG_TO_RAD) * delay;
      sumY += cos(ang * DEG_TO_RAD) * delay;
    }

  if (k == 0)
  {
    for (int i = 0; i < 16; i++)
    {
      line_time[i] = portMAX_DELAY;
    }
    x = 0;
    y = 0;
  }
  else
  {
    x = sumX / k;
    y = sumY / k;
    float length = sqrt(x * x + y * y);
    x /= length;
    y /= length;
  }
}

void LineSensor_t::read_line_sensors()
{
  for (int channel = 0; channel < 16; channel ++)
  {
    if(!rabotaet[channel] && !CONFIG.al_seners)
      continue;
    for (int bit = 0; bit < 4; bit++)
      ESP_ERROR_CHECK(gpio_set_level(CONFIG.mult_in[bit], MULT_CHANEL[channel][bit]));
    ESP_ERROR_CHECK(adc_oneshot_read(adc_mult, CONFIG.ADC_chanel, &actual_value[channel]));
  }
}