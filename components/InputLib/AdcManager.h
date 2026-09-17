#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

class AdcManager {
private:
    static constexpr const char *TAG = "AdcManager";

public:
    /**
     * @brief Возвращает существующий хэндл блока АЦП или инициализирует новый при первом обращении.
     * @param unit Номер блока АЦП (ADC_UNIT_1 или ADC_UNIT_2)
     * @return adc_oneshot_unit_handle_t или nullptr в случае ошибки
     */
    static adc_oneshot_unit_handle_t get_unit(adc_unit_t unit) {
        static adc_oneshot_unit_handle_t adc1_handle = nullptr;
        static adc_oneshot_unit_handle_t adc2_handle = nullptr;

        adc_oneshot_unit_handle_t *target = nullptr;
        if (unit == ADC_UNIT_1) {
            target = &adc1_handle;
        } else if (unit == ADC_UNIT_2) {
            target = &adc2_handle;
        } else {
            ESP_LOGE(TAG, "Invalid ADC unit: %d", unit);
            return nullptr;
        }

        if (*target == nullptr) {
            adc_oneshot_unit_init_cfg_t init_config = {
                .unit_id = unit,
                .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
                .ulp_mode = ADC_ULP_MODE_DISABLE,
            };
            esp_err_t ret = adc_oneshot_new_unit(&init_config, target);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to initialize ADC unit %d: %s", unit, esp_err_to_name(ret));
                return nullptr;
            }
            ESP_LOGI(TAG, "Successfully initialized ADC unit %d", unit);
        }

        return *target;
    }
};
