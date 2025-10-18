#include <stdio.h>
#include <driver/gpio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "../../Software_components/Encoder.h"
#include "../../Software_components/Motor_PID_encoded.h"
#include "../../Software_components/Motor.h"

struct Encoder E1;
void encoder_isr_1(void* arg)
{
    encoder_update(&E1);
}
struct Encoder E2;
void encoder_isr_2(void* arg)
{
    encoder_update(&E2);
}

void pid_timer(void* arg)
{
    timer_flag = 1;
}
void app_main(void)
{
    // motor 1 
    printf("Encoder pin: %d\n", E1.pin_A);
    encoder_init(&E1, 22, 23, 360.0f, 31.4f);
    E1.target_distance = 400.0f;
    encoder_attach_isr(&E1);
    gpio_isr_handler_add(E1.pin_A, encoder_isr_1, NULL);

    struct PID pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f, 0.01f, -100.0f, 100.0f);

    // motor 2
    printf("Encoder pin: %d\n", E2.pin_A);
    encoder_init(&E2, 5, 17, 360.0f, 31.4f);
    E2.target_distance = 400.0f;
    encoder_attach_isr(&E2);
    gpio_isr_handler_add(E2.pin_A, encoder_isr_2, NULL);

    struct PID pid2;
    pid_init(&pid2, 1.0f, 0.0f, 0.0f, 0.01f, -100.0f, 100.0f);

    // setup timer interrupt
    

    esp_timer_handle_t periodic_timer;
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &pid_timer,
        .arg = NULL,
        .name = "pid_timer"
    };
    esp_err_t rc = esp_timer_create(&periodic_timer_args, &periodic_timer);
    if (rc != ESP_OK) {
        printf("Failed to create timer: %d\n", rc);
    } else {
        /* start periodic timer with 15ms period (microseconds) */
        esp_timer_start_periodic(periodic_timer, 15000); // 15ms
    }

    // motor initialization
    struct Motor motor1;
    motor_init(&motor1, 18, 19, 21, 0.0f, 255.0f, 0); // PWM pin, IN1, IN2, min PWM, max PWM, channel
    printf("Hello, World! \n");
    struct Motor motor2;
    motor_init(&motor2, 2, 4, 16, 0.0f, 255.0f, 0); // PWM pin, IN1, IN2, min PWM, max PWM, channel
    printf("Hello, World! \n");
    while (1)
    {
        /* code */
        if(timer_flag){
            timer_flag = 0;
            float error = encoder_get_error_distance(&E1);
            float error2 = encoder_get_error_distance(&E2);
            pid_compute(&pid, error, 0.015f);
            pid_compute(&pid2, error2, 0.015f);
            printf("PID1 output: %.2f ", pid.output);
            printf("PID2 output: %.2f \n", pid2.output);
            motor1.current_speed = motor_speed_ratio(&motor1, pid.output);
            motor2.current_speed = motor_speed_ratio(&motor2, pid2.output);
            motor_set_ratio(&motor1, motor1.current_speed);
            motor_set_ratio(&motor2, motor2.current_speed);
        }
        printf("Counts1: %ld ", E1.counts);
        printf("Counts2: %ld \n", E2.counts);
    }
    
}