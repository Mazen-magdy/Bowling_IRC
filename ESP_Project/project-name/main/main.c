#include <stdio.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define LED_PIN 2

void app_main(void)
{
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    while (1) {
        gpio_set_level(LED_PIN, 1);  // LED ON
        esp_rom_delay_us(1000000);   // 1 second delay
        
        gpio_set_level(LED_PIN, 0);  // LED OFF
        esp_rom_delay_us(1000000);   // 1 second delay
    }
}