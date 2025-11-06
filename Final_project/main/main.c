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
#include "../../Software_components/Wifi.h"

struct setup
{
    int region; // 0 for B, 1 for C
    int direction; // 0 for in front of me, 1 for at left , 2 for behind me, 3 for at right
};
int create_socket_flag = 0;
int Proccess = 0;
int set_angle_1 = 0;
int set_distance_1 = 0;
int set_angle_2 = 0;
int set_distance_2 = 0;
int set_angle_aim = 0;
float camera_angle = 0.0f;
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
    // setup
    s1.region = 0;
    s1.direction = 0;
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
    motor_init(&motor1, 19, 18, 5, 0.0f, 255.0f, 0); // PWM pin, IN1, IN2, min PWM, max PWM, channel
    printf("Hello, World! \n");
    struct Motor motor2; // 17 16 4
    motor_init(&motor2, 4, 16, 17, 0.0f, 255.0f, 0); // PWM pin, IN1, IN2, min PWM, max PWM, channel
    printf("Hello, World! \n");

    // mpu init
    struct mpu6050_Data mpu_data;
    mpu_data.angle_target = 0.0f;
    // MPU6050();

    // Strike mechanism init
    struct StrikeSettings strike_settings;
    strike_init(&strike_settings, 23, 2, 2); // servo pin, motor1 pin, motor2 pin
    while(1){
        // * this proccess loop is to control the robot based on predefined steps

        // Initialization
        if(Proccess == 0){
            // TODO: initialization
            Proccess = 1; // ! skip setup for testing
            {
            // if(create_socket_flag == 0)
            // {
            //     socket_create_and_connect();
            //     create_socket_flag = 1;
            // }
            char message[3]; // const 
            message[0] = 'setup';
            message[1] = '\0';
            send_or_receive(message); // recieves two values: region and direction 01 10 02 03 04 ...
            s1.region = atoi(&received_buffer[0]);
            s1.direction = atoi(&received_buffer[1]);
            printf("%s\n", received_buffer);
            printf("finish proccesss 0\n");
            // close_socket();
        }
        }
        // Rotation 1
        // ! you may rotate using motion pid via setting 2 motors speed the same way as move but with opposite signs and distance = .25 of the circle circumference
        else if(Proccess == 1){
            // TODO: Rotation
            // Proccess =2; // ! skip rotation for testing
            if(set_angle_1 == 0){
                    // TODO: set target angle based on setup
                    if(s1.region == 0){
                        // Region B
                        if(s1.direction == 0){
                            mpu_data.angle_target = -90.0f;
                        }
                        else if(s1.direction == 1){
                            mpu_data.angle_target = -180.0f;
                        }
                        else if(s1.direction == 2){
                            mpu_data.angle_target = 90.0f;
                        }
                        else if(s1.direction == 3){
                            mpu_data.angle_target = 0.0f;
                        }
                    }
                    else if(s1.region == 1){
                        // Region C
                        if(s1.direction == 0){
                            mpu_data.angle_target = 90.0f;
                        }
                        else if(s1.direction == 1){
                            mpu_data.angle_target = 180.0f;
                        }
                        else if(s1.direction == 2){
                            mpu_data.angle_target = -90.0f;
                        }
                        else if(s1.direction == 3){
                            mpu_data.angle_target = 0.0f;
                        }
                    }
            }
            if(timer_flag){
                timer_flag = 0;
                update_angles(&mpu_data);
                float error = mpu_data.angle_error;
                printf("%.2f ", mpu_data.angle);
                pid_compute(&rotate, error, 0.015f);
                printf("%.2f ", rotate.output);
                motor1.current_speed = motor_speed_ratio(&motor1, rotate.output);
                motor2.current_speed = motor_speed_ratio(&motor2, rotate.output);
                motor_set_ratio(&motor1, motor1.current_speed);
                motor_set_ratio(&motor2, motor2.current_speed);
                if(fabs(mpu_data.angle_error) < 2.0f){
                    // angle reached
                    printf("finish proccesss 1\n");
                    Proccess = 2;
                }
            }
        }
        // Move 1
        else if(Proccess == 2){
            // TODO: Move
            Proccess = 3; // ! skip move for testing
            // printf("proccess 2 ");
            if(set_distance_1 == 0){
                // TODO: set target distance based on setup
                E1.target_distance = -50.0f; // back
                E2.target_distance = 50.0f; // back
                E1.counts = 0;
                E2.counts = 0;
                set_distance_1 = 1;
            }   
            if(timer_flag){
                timer_flag = 0;
                float error = encoder_get_error_distance(&E1);
                float error2 = encoder_get_error_distance(&E2);
                printf("%0.2f ", error);
                printf("%0.2f ", error2);
                pid_compute(&pid, error, 0.015f);
                pid_compute(&pid2, error2, 0.015f);
                printf("%.2f ", pid.output);
                printf("%.2f ", pid2.output);
                motor1.current_speed = motor_speed_ratio(&motor1, pid.output);
                motor2.current_speed = motor_speed_ratio(&motor2, pid2.output);
                printf("%ld ", E1.counts);
                printf("%ld ", E2.counts);
                if(fabs(error) > 1.0f){
                  motor_set_ratio(&motor1,  motor1.current_speed);
                  printf("M1 ");
                }
                if (fabs(error2) > 1.0f)
                {
                    motor_set_ratio(&motor2,  motor2.current_speed);
                    printf("M2 \n");
                }
                if(fabs(error) < 1.0f && fabs(error2) < 1.0f){
                    // distance reached
                    printf("finish proccesss 2\n");
                    Proccess = 3;
                }
                printf("\n");
            }

            // ? can we use 2 PIDs one for linear motion and the other for maintain zero angle shift and it satisfy both conditions
        }
        // Rotation 2
        else if(Proccess == 3){
            Proccess = 4; // ! skip rotation for testing
            // TODO: Rotation
            if(set_angle_2 == 0){
                    // TODO: set target angle based on setup
                    if(s1.region == 0){
                        // region B
                        mpu_data.angle_target = 90.0f;
                    }
                    else if(s1.region == 1){
                        // region C
                        mpu_data.angle_target = -90.0f;
                    }
                    set_angle_2 = 1;
            }
            if(timer_flag){
                timer_flag = 0;
                update_angles(&mpu_data);
                float error = mpu_data.angle_error;
                printf("%.2f ", mpu_data.angle);
                printf("%0.2f ", error);
                pid_compute(&rotate, error, 0.015f);
                printf("%.2f ", rotate.output);
                motor1.current_speed = motor_speed_ratio(&motor1, rotate.output);
                motor2.current_speed = motor_speed_ratio(&motor2, rotate.output);
                motor_set_ratio(&motor1, motor1.current_speed);
                motor_set_ratio(&motor2, motor2.current_speed);
                if(fabs(mpu_data.angle_error) < 2.0f){
                    // angle reached
                    Proccess = 4;
            }
        }
        // Move 2
        else if(Proccess == 4){
            // TODO: Move
            Proccess = 5;
            if(set_distance_2 == 0){
                // TODO: set target distance based on setup
                E1.target_distance = 50.0f;
                E2.target_distance = -50.0f;
                E1.counts = 0;
                E2.counts = 0;
            }
            if(timer_flag){
                timer_flag = 0;
                float error = encoder_get_error_distance(&E1);
                float error2 = encoder_get_error_distance(&E2);
                pid_compute(&pid, error, 0.015f);
                pid_compute(&pid2, error2, 0.015f);
                printf("%.2f ", pid.output);
                printf("%.2f ", pid2.output);
                motor1.current_speed = motor_speed_ratio(&motor1, pid.output);
                motor2.current_speed = motor_speed_ratio(&motor2, pid2.output);
                motor_set_ratio(&motor1, motor1.current_speed);
                motor_set_ratio(&motor2, motor2.current_speed);
                if(fabs(error) < 1.0f && fabs(error2) < 1.0f){
                    // distance reached
                    printf("finish proccesss 4\n");
                    Proccess = 5;
                }
            }
            printf("%ld ", E1.counts);
            printf("%ld \n", E2.counts);
            // ? can we use 2 PIDs one for linear motion and the other for maintain zero angle shift and it satisfy both conditions
        }
        // Get Angle
        else if(Proccess == 5){
            // TODO: Get Angle
            Proccess = 6;
        }
        // Aim
        else if(Proccess == 6){
            // TODO: Aim
            Proccess = 7;
            if(set_angle_aim == 0){
                // TODO: set target angle based on camera angle
                char message[4]; // const
                message[0] = 'aim';
                message[3] = '\0';
                send_or_receive(message); // receives float angle as string
                camera_angle = atof(&received_buffer);
                printf("%s\n", received_buffer);
                printf("finish aim \n");
                set_angle_aim = 1;
                mpu_data.angle_target = camera_angle;
            }
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
                if(fabs(mpu_data.angle_error) < 2.0f){
                    // angle reached
                    printf("finish proccesss 6\n");
                    Proccess = 7;
                }
            }

        }
        // Strike
        else if(Proccess == 7){
            // TODO: Strike
            printf("strike! \n");
            strike_execute(&strike_settings);
            vTaskDelay(5000 / portTICK_PERIOD_MS); // wait for 5 seconds before next strike
            printf("finish strike \n");
            Proccess = 8;
        }
    }
    }
}