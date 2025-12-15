idf_component_register(SRCS "mpu_driver.c"
                       INCLUDE_DIRS "include"
                       REQUIRES common esp_driver_i2c driver)