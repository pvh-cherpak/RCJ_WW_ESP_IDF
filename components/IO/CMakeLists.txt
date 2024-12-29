idf_component_register(SRCS "sensors.cpp" "mpu6050.cpp"
                      INCLUDE_DIRS "include"
                      REQUIRES MPU6050 I2Cdev
                      PRIV_REQUIRES driver esp_timer)
