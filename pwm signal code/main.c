#include <driver/ledc.h>

#define pin 2

#define Channel LEDC_CHANNEL_0

#define timer LEDC_TIMER_0
#define frquency 15000

int pwm_value = 0;
void pwm_signal_setup();

void app_main()
{
    //setup the channel and the timer required for pwm signal
    pwm_signal_setup();


    // setting the pwm value like analogWrite() in arduino
    ledc_set_duty(LEDC_LOW_SPEED_MODE, Channel, pwm_value);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, Channel);
    
    


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
        .gpio_num = pin, // the pin that we will use 
        .duty = 0,  // start with 0 or low
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}