#include "main.h"



void set_servo_angle(uint8_t angle)
{
    uint32_t duty_milli = SERVO_MIN + ((SERVO_MAX - SERVO_MIN) * angle) / 180;
    uint32_t duty = (duty_milli * (1 << LEDC_TIMER_12_BIT)) / (1000 / SERVO_FREQ);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL);
}

void start_servo()
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
        .gpio_num       = SERVO_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    while (1) {
        set_servo_angle(180); // Move to 180 degrees
        // vTaskDelay(pdMS_TO_TICKS(2000));
        // set_servo_angle(0);   // Move back to 0 degrees
        // vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    start_servo();
}
