#include "LightGates.h"
#include "AdcManager.h"
#include "esp_log.h"

static const char *TAG = "LightGates";

esp_err_t LightGates_t::init(gpio_num_t pin_num) {
    lightPin = pin_num;

    // 1. Автоматически определяем Unit и Channel по номеру GPIO
    esp_err_t err = adc_oneshot_io_to_channel(lightPin, &adc_unit_light, &adc_channel_light);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Pin GPIO %d is not a valid ADC pin: %s", lightPin, esp_err_to_name(err));
        return err;
    }

    // 2. Получаем хэндл нужного блока АЦП у AdcManager
    adc_light = AdcManager::get_unit(adc_unit_light);
    if (adc_light == nullptr) {
        ESP_LOGE(TAG, "Failed to get handle for ADC unit %d", adc_unit_light);
        return ESP_FAIL;
    }

    // 3. Настраиваем канал
    adc_oneshot_chan_cfg_t ADC_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    err = adc_oneshot_config_channel(adc_light, adc_channel_light, &ADC_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel %d: %s", adc_channel_light, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Initialized on GPIO %d (Unit %d, Channel %d, threshold %d)", 
             lightPin, adc_unit_light, adc_channel_light, isBallThreshold);
    return ESP_OK;
}

void LightGates_t::update() {
    if (adc_light == nullptr) return;

    int luminosity = 0;
    esp_err_t err = adc_oneshot_read(adc_light, adc_channel_light, &luminosity);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC read failed: %s", esp_err_to_name(err));
        return;
    }

    currentLuminosity = luminosity;
    ESP_LOGV(TAG, "lumin: %d", luminosity);
    isBallValue = (luminosity <= isBallThreshold);

    if (isBallValue) {
        lastIsBallTime = esp_timer_get_time();
    }
}

bool LightGates_t::isBall() const {
    return isBallValue;
}

bool LightGates_t::ballCatched() const {
    return (esp_timer_get_time() - lastIsBallTime <= holdTimeUs);
}