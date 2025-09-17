#include <stdio.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32/rom/ets_sys.h"

// Number of QTR_8RC sensors used
#define num_of_sensors 5
int sensor_pins[num_of_sensors] = {1,2,3,4,5}; // GPIO pins connected to sensors
int sensor_values[num_of_sensors] = {};        // Array to store sensor readings

void app_main(void)
{
    // Charge the capacitors by setting pins to output and high
    for (int i = 0; i < num_of_sensors; i++)
    {
        gpio_set_direction(sensor_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(sensor_pins[i], 1);
    }
    ets_delay_us(10); // Wait for capacitors to charge 10 us is enough

    // Set pins to input to start discharging (sensing phase)
    for (int i = 0; i < num_of_sensors; i++)
    {
        gpio_set_direction(sensor_pins[i], GPIO_MODE_INPUT);
    }

    int still_high = num_of_sensors; // Track how many sensors are still high
    uint64_t start_time = esp_timer_get_time(); // Record start time
    uint64_t timeout = 3000; // Timeout in microseconds which we will consider as no line detected to avoid large numbers

    // Initialize sensor values to timeout
    for (int i = 0; i < num_of_sensors; i++)
    {
        sensor_values[i] = timeout;
    }
    
    // Measure discharge time for each sensor
    while (still_high > 0 && (esp_timer_get_time() - start_time) < timeout) // Continue until all sensors are low or timeout
    {
        uint64_t now = esp_timer_get_time(); // Current time

        // Check each sensor
        for (int i = 0; i < num_of_sensors; i++)
        {
            // If sensor pin goes low, record the time it took to discharge
            if (gpio_get_level(sensor_pins[i]) == 0 && (now - start_time) < timeout) // If pin is low and within timeout
            {
                sensor_values[i] = now - start_time; // Record discharge time
                still_high--; // Decrement count of sensors still high
            }
        }
    }
}
    
        
//     printf("Hello, World!\n");
// }