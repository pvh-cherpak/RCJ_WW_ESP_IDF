idf_component_register(SRCS "OpenMV.cpp" "LineSensor.cpp" "sensors.cpp" "mpu6050.cpp" "locator.cpp" "OpenMV.cpp"
                      INCLUDE_DIRS "include"
                      REQUIRES MPU6050 I2Cdev esp_driver_gpio nvs_flash esp_adc esp_driver_uart
                      PRIV_REQUIRES driver esp_timer)
