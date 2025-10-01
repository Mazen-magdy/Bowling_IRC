#include "Motor.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <math.h>

#define PWM_FREQ_HZ 5000
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define PWM_MAX_DUTY 255

void motor_init(struct Motor* motor, int pin_PWM, int pin_IN1, int pin_IN2, float min_PWM, float max_PWM,int channel_In) {
    motor->pin_PWM = pin_PWM;
    motor->pin_IN1 = pin_IN1;
    motor->pin_IN2 = pin_IN2;
    motor->min_PWM = min_PWM;
    motor->max_PWM = max_PWM;
    motor->current_speed = 0.0f;
    if(channel_In){
    motor->channel = LEDC_CHANNEL_1;
    } else{
    motor->channel = LEDC_CHANNEL_0;
    }
    // Initialize GPIO pins for direction control
    gpio_set_direction(pin_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_IN2, GPIO_MODE_OUTPUT);

    // Set initial state (motor stopped)
    gpio_set_level(pin_IN1, 0);
    gpio_set_level(pin_IN2, 0);
    
    // Configure LEDC timer (shared by both channels)
    static bool timer_configured = false;
    if (!timer_configured) {
        ledc_timer_config_t ledc_timer = {
            .duty_resolution = PWM_RESOLUTION,
            .freq_hz = PWM_FREQ_HZ,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_0,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&ledc_timer);
        timer_configured = true;
    }

    // Determine which channel to use based on PWM pin
    ledc_channel_t channel;
    channel = motor->channel;
    
    // Configure LEDC channel for this motor
    ledc_channel_config_t ledc_channel = {
        .channel = channel,
        .duty = 0,
        .gpio_num = pin_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);
}

void motor_set_ratio(struct Motor* motor, float ratio) {
    // Clamp ratio between -1.0 and 1.0 using math.h functions
    ratio = fmaxf(fminf(ratio, 1.0f), -1.0f);
    
    uint32_t duty = 0;
    
    if (ratio > 0.0f) {
        // Forward direction
        gpio_set_level(motor->pin_IN1, 1);
        gpio_set_level(motor->pin_IN2, 0);
        
        // Apply minimum PWM if needed using math.h
        float min_ratio = motor->min_PWM / 255.0f;
        ratio = fmaxf(ratio, min_ratio);
        
        duty = (uint32_t)(ratio * PWM_MAX_DUTY);
    }
    else if (ratio < 0.0f) {
        // Reverse direction
        gpio_set_level(motor->pin_IN1, 0);
        gpio_set_level(motor->pin_IN2, 1);
        
        // Apply minimum PWM if needed using math.h
        float min_ratio = motor->min_PWM / 255.0f;
        float abs_ratio = fabsf(ratio);  // Use fabsf from math.h
        abs_ratio = fmaxf(abs_ratio, min_ratio);
        
        duty = (uint32_t)(abs_ratio * PWM_MAX_DUTY);
    }
    else {
        // Stop motor
        gpio_set_level(motor->pin_IN1, 0);
        gpio_set_level(motor->pin_IN2, 0);
        duty = 0;
    }
    
    // Set PWM duty cycle
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
    
    // Update current speed
    motor->current_speed = ratio * 100.0f; // Convert to percentage
}

float motor_speed_ratio(struct Motor* motor, float speed){
    speed = fmaxf(fminf(speed, 12.0f), -12.0f);
    return speed / 12.0f; // Normalize to -1.0 to 1.0
}