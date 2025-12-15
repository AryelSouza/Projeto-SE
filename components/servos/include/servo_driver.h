#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

void servos_init(void);
void servo_control_task(void *pvParameters);
void auto_calibration_task(void *pvParameters);

#endif