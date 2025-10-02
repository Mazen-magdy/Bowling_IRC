#include <stdio.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "../Project_components/PID.h"
#include "../Project_components/Motor.h"
#include "../Project_components/Encoder.h"
#include "../Project_components/Serial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Timer flag for periodic interrupt

#define E1_PIN_A 23
#define E1_PIN_B 22
#define M1_PIN_PWM 21
#define M1_PIN_IN1 19
#define M1_PIN_IN2 18

#define E2_PIN_A 36
#define E2_PIN_B 39
#define M2_PIN_PWM 4
#define M2_PIN_IN1 16
#define M2_PIN_IN2 17

// ============ Global Variables ============//
int timer_flag = 0;
esp_timer_handle_t Motor_Timer;  // Global timer handle for control

static const char *TAG = "MOTOR_CONTROL";

struct Encoder encoder1;
struct Encoder encoder2;

struct Motor motor1;
struct Motor motor2;

struct PID pid1;
struct PID pid2;
struct PID pid_outer_loop;

float k_sync = 0.0f; 
//============ ISR Handlers ============//

void  encoder_isr_1() {
    encoder_update(&encoder1);
    printf("ISR1 triggered! Count: %d\n", (int)encoder1.counts);  // Debug output
}

void  encoder_isr_2() {
    encoder_update(&encoder2);
    printf("ISR2 triggered! Count: %d\n", (int)encoder2.counts);  // Debug output
}

void periodic_timer_callback(void* arg) {
    // Timer interrupt code here
    timer_flag = 1;
}

void setup_periodic_timer() {
    const esp_timer_create_args_t timer_args = {
        .callback = &periodic_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "periodic_15ms"
    };
    esp_timer_create(&timer_args, &Motor_Timer);  // Use global handle
}

// Timer control functions
void timer_start() {
    esp_timer_start_periodic(Motor_Timer, 15000);
}

void timer_stop() {
    esp_timer_stop(Motor_Timer);
}

void timer_restart() {
    esp_timer_stop(Motor_Timer);
    esp_timer_start_periodic(Motor_Timer, 15000);
}

void app_main(void)
{
    serial_init();
    setup_periodic_timer();
    encoder_init(&encoder1, E1_PIN_A, E1_PIN_B, 990.0f, 40.84f);
    encoder_init(&encoder2, E2_PIN_A, E2_PIN_B, 990.0f, 40.84f);

    motor_init(&motor1, M1_PIN_PWM, M1_PIN_IN1, M1_PIN_IN2, 50.0f, 255.0f,0);
    motor_init(&motor2, M2_PIN_PWM, M2_PIN_IN1, M2_PIN_IN2, 50.0f, 255.0f,1);

    // float m1_p, m1_i, m1_d;
    // float m2_p, m2_i, m2_d;
    // float outer_p, outer_i, outer_d;
    // float clearance;
    // float target_distance1, target_distance2;  // Add target distances
    
    // // Read 12 parameters from serial: PID values + clearance + distances
    // printf("Enter PID parameters and target distances:\n");
    // fflush(stdout);
    
    // // Read 12 float parameters from serial input one by one
    // float* params[] = {&m1_p, &m1_i, &m1_d, &m2_p, &m2_i, &m2_d, &outer_p, &outer_i, &outer_d, &clearance, &target_distance1, &target_distance2};
    // const char* param_names[] = {
    //     "m1_p", "m1_i", "m1_d", "m2_p", "m2_i", "m2_d", 
    //     "outer_p", "outer_i", "outer_d", "clearance", 
    //     "target_distance1_cm", "target_distance2_cm"
    // };
    
    // for (int i = 0; i < 12; ++i) {
    //     serial_send_string("Enter value for:");
    //     serial_send_string(param_names[i]);
    //     printf("Enter %s: ", param_names[i]);
    //     fflush(stdout);
        
    //     // Use our serial function to receive float (blocking)
    //     *params[i] = serial_receive_float();
        
    //     printf("Received: %.6f\n", *params[i]);
    // }
    
    // printf("All Params received:\n");
    // printf("Motor1 PID: %.2f %.2f %.2f\n", m1_p, m1_i, m1_d);
    // printf("Motor2 PID: %.2f %.2f %.2f\n", m2_p, m2_i, m2_d);
    // printf("Outer PID: %.2f %.2f %.2f\n", outer_p, outer_i, outer_d);
    // printf("Clearance: %.2f\n", clearance);
    // printf("Target Distance 1: %.2f cm\n", target_distance1);
    // printf("Target Distance 2: %.2f cm\n", target_distance2);
    
    // Initialize PIDs with received parameters
    pid_init(&pid1, 1.0f, 0.0f, 0.0f, 0.5f, -12.0f, 12.0f);
    pid_init(&pid2, 1.0f, 0.0f, 0.0f, 0.5f, -12.0f, 12.0f);
    pid_init(&pid_outer_loop, 1.0f, 0.0f, 0.0f, 0.5f, -12.0f, 12.0f);
    encoder_attach_isr(&encoder1, encoder_isr_1);
    // encoder_attach_isr(&encoder2, encoder_isr_2);
    
    printf("ISR attached for encoder1 on pin %d\n", E1_PIN_A);
    printf("Initial pin A level: %d\n", gpio_get_level(E1_PIN_A));
    printf("Initial pin B level: %d\n", gpio_get_level(E1_PIN_B));
    
    // Set target distances from serial input
    encoder1.target_distance = 100; // Target distance in cm
    encoder2.target_distance = 100; // Target distance in cm
    timer_start();
    while (1) {
        
      
            
        
        if(timer_flag) {
            // Inner loop PID for speed
            float error1 = encoder_get_error_distance(&encoder1);
            ESP_LOGE(TAG, "Encoder 1 error distance: %.2f\n", error1); // Using error distance as a proxy for speed
            float error2 = encoder_get_error_distance(&encoder2);
            ESP_LOGE(TAG, "Encoder 2 error distance: %.2f\n", error2);
            if(error1 == 0 && error2 == 0){
                ESP_LOGI(TAG,"Target reached. Stopping motors.\n");
                motor_set_ratio(&motor1, 0.0f);
                motor_set_ratio(&motor2, 0.0f);
                timer_stop();  // Stop the timer when no error
                timer_flag = 0;
                continue;  // Skip PID computation
            }
            if(error1 != 0){
                motor_set_ratio(&motor1,0.0f); // Reset counts for next interval
            }
            if(error2 != 0){
                motor_set_ratio(&motor2,0.0f); // Reset counts for next interval
            }
            pid_compute(&pid1, error1, 0.015f);
            pid_compute(&pid2, error2, 0.015f);
            // Outer loop PID for position
            float outer_error = error1 - error2;
            pid_compute(&pid_outer_loop, outer_error, 0.015f);
            float motor1_speed_ratio = motor_speed_ratio(&motor1, pid1.output + k_sync * (pid_outer_loop.output));
            float motor2_speed_ratio = motor_speed_ratio(&motor2, pid2.output - k_sync * (pid_outer_loop.output));
            motor_set_ratio(&motor1, motor1_speed_ratio);
            motor_set_ratio(&motor2, motor2_speed_ratio);
            timer_flag = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}