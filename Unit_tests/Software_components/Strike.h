#ifndef STRIKE_H
#define STRIKE_H
#include <stdio.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define SERVO_MIN    1     // Minimum pulse width in milliseconds (0 deg)
#define SERVO_MAX    2    // Maximum pulse width in milliseconds (180 deg)
#define SERVO_FREQ      50      // Typical servo PWM frequency (Hz)
#define LEDC_TIMER      LEDC_TIMER_0   // choosing timer 0->3
#define LEDC_CHANNEL    LEDC_CHANNEL_0  // choosing channel 0->7

void strike_init(struct StrikeSettings* settings, int servo_pin, int motor1_pin, int motor2_pin);

void strike_execute(struct StrikeSettings* settings);

struct StrikeSettings
{
    int servo_pin;
    int motor1_pin;
    int motor2_pin;
};

#endif // STRIKE_H