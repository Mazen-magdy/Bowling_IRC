#include "main.h"
#define Channel LEDC_CHANNEL_0

#define timer LEDC_TIMER_0
#define frquency 5000

void pwm_signal_setup();

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
    pwm_signal_setup();
    servo_setup();
    // int angle = 0;
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, Channel, 4095);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, Channel);
    vTaskDelay(pdMS_TO_TICKS(5000));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, Channel, 2048);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, Channel); 
    set_servo_angle(90);
    vTaskDelay(pdMS_TO_TICKS(90000));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, Channel, 4095);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, Channel);
    // while (true)
    // {
    //     angle = 180;
    //     set_servo_angle(angle);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     angle = 0;
    //     set_servo_angle(angle);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
}



void pwm_signal_setup()
{
    // configure the timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = timer, 
        .duty_resolution = LEDC_TIMER_12_BIT, // setting the resolution 8-bit --> 12-bit
        .freq_hz = frquency, // setting the frequency from 5KHz --> 20KHz
        .clk_cfg = LEDC_AUTO_CLK, // LEDC source clock will be automatically selected based on resolution and duty
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    
    // configure channel 0
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = Channel,
        .timer_sel = timer,
        .intr_type = LEDC_INTR_DISABLE, // disable interrupt
        .gpio_num = 2, // the pin that we will use 
        .duty = 0,  // start with 0 or low
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // configure channel 0
    ledc_channel_config_t ledc_channel_1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = Channel,
        .timer_sel = timer,
        .intr_type = LEDC_INTR_DISABLE, // disable interrupt
        .gpio_num = 2, // the pin that we will use 
        .duty = 0,  // start with 0 or low
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_1));
}
