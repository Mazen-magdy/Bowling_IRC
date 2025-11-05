#include <stdio.h>
#include <driver/gpio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "../../Software_components/Encoder.h"
#include "../../Software_components/Motor_PID_encoded.h"
#include "../../Software_components/Motor.h"
#include "../../Software_components/mpu.h"
#include "../../Software_components/Strike.h"

struct setup
{
    int region; // 0 for B, 1 for C
    int direction; // 0 for in front of me, 1 for at left , 2 for behind me, 3 for at right
};

struct setup s1;

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
    // rotate PID controller
    struct PID rotate;
    pid_init(&rotate, 1.0f, 0.0f, 0.0f, 0.01f, -12.0f, 12.0f);
    // motor 1 
    printf("Encoder pin: %d\n", E1.pin_A);
    encoder_init(&E1, 34, 35, 360.0f, 31.4f);
    E1.target_distance = 300.0f;
    encoder_attach_isr(&E1);
    gpio_isr_handler_add(E1.pin_A, encoder_isr_1, NULL);

    struct PID pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f, 0.01f, -12.0f, 12.0f);
    // motor 2
    printf("Encoder pin: %d\n", E2.pin_A);
    encoder_init(&E2, 32, 33, 360.0f, 31.4f);
    E2.target_distance = -300.0f;
    encoder_attach_isr(&E2);
    gpio_isr_handler_add(E2.pin_A, encoder_isr_2, NULL);

    struct PID pid2;
    pid_init(&pid2, 1.0f, 0.0f, 0.0f, 0.01f, -12.0f, 12.0f);
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
    struct Motor motor1; // 19 18 5
    motor_init(&motor1, 4, 16, 17, 0.0f, 255.0f, 0); // PWM pin, IN1, IN2, min PWM, max PWM, channel
    printf("Hello, World! \n");
    struct Motor motor2; // 17 16 4
    motor_init(&motor2, 19, 18, 5, 0.0f, 255.0f, 0); // PWM pin, IN1, IN2, min PWM, max PWM, channel
    printf("Hello, World! \n");

    // mpu init
    struct mpu6050_Data mpu_data;
    mpu_data.angle_target = 0.0f;
    // MPU6050();

    // Strike mechanism init
    struct StrikeSettings strike_settings;
    strike_init(&strike_settings, 10, 22, 23); // servo pin, motor1 pin, motor2 pin
    while (1)
    {
        /* code */
        // * * code are divided into blocks each block tests a unit from the code 

        // TODO: this part is for testing PID control with encoder feedback then send signal to motor
        // * running but needs high calibration
        // {     
        //     if(timer_flag){
        //         timer_flag = 0;
        //         float error = encoder_get_error_distance(&E1);
        //         float error2 = encoder_get_error_distance(&E2);
        //         pid_compute(&pid, error, 0.015f);
        //         pid_compute(&pid2, error2, 0.015f);
        //         printf("%.2f ", pid.output);
        //         printf("%.2f ", pid2.output);
        //         motor1.current_speed = motor_speed_ratio(&motor1, pid.output);
        //         motor2.current_speed = motor_speed_ratio(&motor2, pid2.output);
        //         motor_set_ratio(&motor1, motor1.current_speed);
        //         motor_set_ratio(&motor2, motor2.current_speed);
        //     }
        //     printf("%ld ", E1.counts);
        //     printf("%ld \n", E2.counts);
        //     // ? can we use 2 PIDs one for linear motion and the other for maintain zero angle shift and it satisfy both conditions
        // }
        // TODO: this part is for testing MPU6050 with PID control to balance the robot and rotate around the center
        {
            if(timer_flag){
                timer_flag = 0;
                update_angles(&mpu_data);
                float error = mpu_data.angle_error;
                printf("%.2f ", mpu_data.angle);
                pid_compute(&rotate, error, 0.015f);
                printf("%.2f ", rotate.output);
                motor1.current_speed = motor_speed_ratio(&motor1, rotate.output);
                motor2.current_speed = motor_speed_ratio(&motor2, -1 * rotate.output);
                motor_set_ratio(&motor1, motor1.current_speed);
                motor_set_ratio(&motor2, motor2.current_speed);
            }
        }
        // TODO: this part is for testing strike mechanism
        // {
        //     strike_execute(&strike_settings);
        //     vTaskDelay(5000 / portTICK_PERIOD_MS); // wait for 5 seconds before next strike
        // }
        // TODO: initialization
        {
            // send initialize message 
            // initialize params 
        }
        // TODO: get angle from camera
        {
            // send message to get angle 
            // retrieve angle
            // set it as an target 
        }

    }
    
}