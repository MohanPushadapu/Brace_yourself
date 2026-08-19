idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES freertos esp_driver_i2c MPU6050_Driver)
