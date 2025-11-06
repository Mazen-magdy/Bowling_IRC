#include "main.h"



void set_servo_angle(int angle)
{
    uint32_t duty_milli = SERVO_MIN + ((SERVO_MAX - SERVO_MIN) * angle) / 180;
    uint32_t duty = (duty_milli * (1 << LEDC_TIMER_12_BIT)) / (1000 / SERVO_FREQ);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL);
}

void servo_setup()
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
        .duty           = 0, // initial value for pwm
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void app_main(void)
{
    servo_setup();
    int angle = 360;
    set_servo_angle(angle);
}
