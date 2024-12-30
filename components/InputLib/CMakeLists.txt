idf_component_register(SRCS "LineSensor.cpp" "sensors.cpp" "mpu6050.cpp"
                      INCLUDE_DIRS "include"
                      REQUIRES MPU6050 I2Cdev esp_driver_gpio nvs_flash
                      PRIV_REQUIRES driver esp_timer)
