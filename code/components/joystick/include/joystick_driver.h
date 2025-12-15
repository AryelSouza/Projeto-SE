#ifndef JOYSTICK_DRIVER_H
#define JOYSTICK_DRIVER_H

void joystick_init(void);
void joystick_read_task(void *pvParameters);

#endif