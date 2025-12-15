idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES common joystick servos mpu6050 game_logic)