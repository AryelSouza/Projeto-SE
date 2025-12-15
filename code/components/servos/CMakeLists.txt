idf_component_register(SRCS "servo_driver.c"
                       INCLUDE_DIRS "include"
                       REQUIRES common esp_driver_ledc)