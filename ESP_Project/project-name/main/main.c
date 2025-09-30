#include <stdio.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "../Project_components/PID.h"
#include "../Project_components/Motor.h"
#include "../Project_components/Encoder.h"

// Timer flag for periodic interrupt
extern int timer_flag;

#define LED_PIN 2
#define E1_PIN_A 18
#define E1_PIN_B 19
#define M1_PIN_PWM 5
#define M1_PIN_IN1 17
#define M1_PIN_IN2 16

#define E2_PIN_A 20
#define E2_PIN_B 21
#define M2_PIN_PWM 6
#define M2_PIN_IN1 7
#define M2_PIN_IN2 8

// ============ Global Variables ============//
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
}

void  encoder_isr_2() {
    encoder_update(&encoder2);
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
    esp_timer_handle_t timer_handle;
    esp_timer_create(&timer_args, &timer_handle);
    esp_timer_start_periodic(timer_handle, 15000); // 15ms in microseconds
}

void app_main(void)
{
    
    setup_periodic_timer();
    
    encoder_init(&encoder1, E1_PIN_A, E1_PIN_B, 990.0f, 40.84f);
    encoder_init(&encoder2, E2_PIN_A, E2_PIN_B, 990.0f, 40.84f);

    motor_init(&motor1, M1_PIN_PWM, M1_PIN_IN1, M1_PIN_IN2, 50.0f, 255.0f);
    motor_init(&motor2, M2_PIN_PWM, M2_PIN_IN1, M2_PIN_IN2, 50.0f, 255.0f);

    pid_init(&pid1, 1.0f, 0.1f, 0.01f, 0.0f, -12.0f, 12.0f);
    pid_init(&pid2, 1.0f, 0.1f, 0.01f, 0.0f, -12.0f, 12.0f);
    pid_init(&pid_outer_loop, 0.5f, 0.05f, 0.005f, 0.0f, -12.0f, 12.0f);
    
    encoder_attach_isr(&encoder1, encoder_isr_1);
    encoder_attach_isr(&encoder2, encoder_isr_2);
    while (1) {
        if(timer_flag) {
            // Inner loop PID for speed
            float error1 = encoder_get_error_distance(&encoder1); // Using error distance as a proxy for speed
            float error2 = encoder_get_error_distance(&encoder2);
            pid_compute(&pid1, error1, 0.015f);
            pid_compute(&pid2, error2, 0.015f);
            // Outer loop PID for position
            float outer_error = error1 - error2;
            pid_compute(&pid_outer_loop, outer_error, 0.015f);
            motor_set_speed(&motor1, pid1.output + k_sync * (pid_outer_loop.output));
            motor_set_speed(&motor2, pid2.output - k_sync * (pid_outer_loop.output));

            timer_flag = 0;
        }
    }
}