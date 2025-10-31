#include "Strike.h"

void set_servo_angle(uint8_t angle)
{
    uint32_t duty_milli = SERVO_MIN + ((SERVO_MAX - SERVO_MIN) * angle) / 180;
    uint32_t duty = (duty_milli * (1 << LEDC_TIMER_12_BIT)) / (1000 / SERVO_FREQ);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL);
}

void start_servo(struct StrikeSettings* strike)
{
        // configure the channel
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = SERVO_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // configure the timer
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = strike->servo_pin,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void strike_init(struct StrikeSettings* settings, int servo_pin, int motor1_pin, int motor2_pin)
{
    settings->servo_pin = servo_pin;
    settings->motor1_pin = motor1_pin;
    settings->motor2_pin = motor2_pin;
    gpio_set_direction(settings->motor1_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(settings->motor2_pin, GPIO_MODE_OUTPUT);
    start_servo(settings);
}

void strike_execute(struct StrikeSettings* settings){
    set_servo_angle(180); // move to strike position
    vTaskDelay(500 / portTICK_PERIOD_MS); // wait for 500ms
    set_servo_angle(0); // move back to initial position

    gpio_set_level(settings->motor1_pin, 1); // activate motor 1
    gpio_set_level(settings->motor2_pin, 1); // activate motor 2
    vTaskDelay(1000 / portTICK_PERIOD_MS); // run motors for 1 second
    gpio_set_level(settings->motor1_pin, 0); // deactivate motor 1
    gpio_set_level(settings->motor2_pin, 0); // deactivate motor 2
}