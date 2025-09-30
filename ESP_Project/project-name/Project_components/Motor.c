#include "Motor.h"
#include "driver/gpio.h"
#include "driver/ledc.h"


void motor_init(struct Motor* motor, int pin_PWM, int pin_IN1, int pin_IN2, float min_PWM, float max_PWM) {
    motor->pin_PWM = pin_PWM;
    motor->pin_IN1 = pin_IN1;
    motor->pin_IN2 = pin_IN2;
    motor->min_PWM = min_PWM;
    motor->max_PWM = max_PWM;
    motor->current_speed = 0.0f;

    // Initialize GPIO pins
    gpio_set_direction(pin_PWM, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_IN2, GPIO_MODE_OUTPUT);

    // Set initial state
    gpio_set_level(pin_IN1, 0);
    gpio_set_level(pin_IN2, 0);
    ledcSetup(0, 5000, 8); // Channel 0, 5 kHz, 8-bit resolution
    ledcAttachPin(pin_PWM, 0);
    ledcWrite(0, 0); // Start with 0% duty cycle
}
void motor_set_speed(struct Motor * motor, float voltage){
    voltage = fmaxf(fminf(voltage,12.0f), -12.0f); // Clamp voltage to -12V to 12V
    float PWM = (voltage / 12.0f) * 255.0f; // Scale voltage (0-12V) to PWM (0-255)
    if(PWM > 0){
        gpio_set_level(motor->pin_IN1, 1);
        gpio_set_level(motor->pin_IN2, 0);
        PWM = fmaxf(PWM, motor->min_PWM); // Enforce minimum PWM
    }
    else if(PWM < 0){
        gpio_set_level(motor->pin_IN1, 0);
        gpio_set_level(motor->pin_IN2, 1);
        PWM = fminf(PWM, -motor->min_PWM); // Enforce minimum PWM
        PWM = -PWM; // Make PWM positive for ledcWrite
    }
    else{
        gpio_set_level(motor->pin_IN1, 0);
        gpio_set_level(motor->pin_IN2, 0);
        PWM = 0;
    }
    ledcWrite(0, PWM);
}