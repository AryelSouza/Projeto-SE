idf_component_register(SRCS "joystick_driver.c"
                       INCLUDE_DIRS "include"
                       REQUIRES common esp_adc esp_driver_gpio)