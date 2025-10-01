#ifndef MOTOR_H
#define MOTOR_H

struct Motor{
    int pin_PWM;
    int pin_IN1;
    int pin_IN2;
    float min_PWM;
    float max_PWM;
    float current_speed; // Current speed in percentage (-100 to 100)
};

void motor_init(struct Motor* motor, int pin_PWM, int pin_IN1, int pin_IN2, float min_PWM, float max_PWM) ;
void motor_set_speed(struct Motor * motor, float voltage);
#endif // MOTOR_H