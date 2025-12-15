#ifndef SYSTEM_COMMON_H
#define SYSTEM_COMMON_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Definições Globais úteis
#define RAD_TO_DEG 57.2957795131

typedef struct {
    int joy_x_raw; 
    int joy_y_raw; 
    bool btn_pressed;
    float filtered_pitch;
    float filtered_roll;
    bool calibration_mode;
    bool calibration_trigger;
    int servo_min_x;
    int servo_max_x;
    int servo_min_y;
    int servo_max_y;
    bool game_won;
} system_data_t;

// Declaração externa (quem incluir isso sabe que essas variáveis existem em algum lugar)
extern system_data_t global_data;
extern SemaphoreHandle_t data_mutex;

#endif