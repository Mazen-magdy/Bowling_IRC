#ifndef MOTOR_H
#define MOTOR_H

struct Motor{
    int pin_PWM;
    int pin_IN1;
    int pin_IN2;
    float min_PWM;
    float max_PWM;
    float current_speed; // Current speed in percentage (-100 to 100)
    int channel; // PWM channel (0 or 1)
};

void motor_init(struct Motor* motor, int pin_PWM, int pin_IN1, int pin_IN2, float min_PWM, float max_PWM,int channel_In);
float motor_speed_ratio(struct Motor* motor, float speed);
void motor_set_ratio(struct Motor* motor, float ratio);

#endif // MOTOR_H