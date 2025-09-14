#include <driver/ledc.h>

#define pin 2

#define Channel LEDC_CHANNEL_0

#define timer LEDC_TIMER_0

int value = 0;
void pwm_signal_setup();

void app_main()
{
    
    pwm_signal_setup();


    // setting the pwm value like analogWrite() in arduino
    ledc_set_duty(LEDC_LOW_SPEED_MODE, Channel, value);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, Channel);
    
    


}


void pwm_signal_setup()
{
    // configure the timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = timer, 
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 10000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    
    // configure channel 0
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = Channel,
        .timer_sel = timer,
        .intr_type = LEDC_INTR_DISABLE, // disable interrupt
        .gpio_num = pin,
        .duty = 0,  // start with 0
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}