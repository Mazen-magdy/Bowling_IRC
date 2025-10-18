#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_GPIO      26    // GPIO pin connected to the servo signal line

#define SERVO_MIN    1     // Minimum pulse width in milliseconds (0 deg)
#define SERVO_MAX    2    // Maximum pulse width in milliseconds (180 deg)
#define SERVO_FREQ      50      // Typical servo PWM frequency (Hz)
#define LEDC_TIMER      LEDC_TIMER_0   // choosing timer 0->3
#define LEDC_CHANNEL    LEDC_CHANNEL_0  // choosing channel 0->7

void start_servo();

#endif