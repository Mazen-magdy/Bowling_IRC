#include "main.h"


static const char *TAG = "QTR_8RC";

// GPIO pins connected to OUT0–OUT7 of the QTR-8RC
int sensor_pins[NUM_SENSORS] = {23, 22, 21, 19, 18, 17, 16, 15};

// Reads one sensor (returns discharge time)
uint32_t read_qtr_sensor(int gpio_num) {
    gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio_num, 1);
    ets_delay_us(10);  // Charge capacitor

    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);

    uint64_t start = esp_timer_get_time();
    while (gpio_get_level(gpio_num) == 1) {
        if ((esp_timer_get_time() - start) > 3000) {
            return 3000; // Timeout
        }
    }
    return (uint32_t)(esp_timer_get_time() - start);
}

void app_main(void) {
    ESP_LOGI(TAG, "QTR-8RC Test Start");

    bool on_white = true;
    bool was_white = true;
    int white_counter = 0;

    while (1) {
        uint32_t values[NUM_SENSORS];
        int black_sensor_count = 0;

        // Read all sensors once
        for (int i = 0; i < NUM_SENSORS; i++) {
            values[i] = read_qtr_sensor(sensor_pins[i]);

            if (values[i] > BLACK_THRESHOLD) {
                black_sensor_count++;
            }
        }

        // Decide if on black or white based on sensor count
        if (black_sensor_count >= BLACK_SENSOR_MIN_COUNT) {
            on_white = false;
            white_counter = 0;
        } else {
            white_counter++;
            if (white_counter >= WHITE_LOST_COUNT) {
                on_white = true;
            }
        }

        // Detect transitions and log
        if (!on_white && was_white) {
            ESP_LOGI(TAG, "Black line detected!");
        } else if (on_white && !was_white) {
            ESP_LOGI(TAG, "Line lost! Back on white.");
        } else {
            ESP_LOGI(TAG, "Tracking... %s", on_white ? "White" : "Black");
        }

        was_white = on_white;

        // Optional: print raw sensor values for tuning
        
        printf("Sensor values: ");
        for (int i = 0; i < NUM_SENSORS; i++) {
            printf("%lu ", values[i]);
        }
        printf("\n");
        

        vTaskDelay(pdMS_TO_TICKS(333));  // Delay between readings
    }
}




























































// #include <stdio.h>
// #include <esp_timer.h>
// #include <driver/gpio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp32/rom/ets_sys.h"
// #include "esp_log.h"


// static const char *TAG = "QTR_8RC";

// // Number of QTR_8RC sensors used
// #define num_of_sensors 7
// int sensor_pins[num_of_sensors] = {23,22,21,19,18,17,16}; // GPIO pins connected to sensors
// int sensor_values[num_of_sensors] = {};        // Array to store sensor readings

// void app_main(void)
// {
//     while(true)
//     {
//         // gpio_set_direction(19, GPIO_MODE_OUTPUT);
//         // gpio_set_level(19,1);
//                 // Charge the capacitors by setting pins to output and high
//         for (int i = 0; i < num_of_sensors; i++)
//         {
//             gpio_set_direction(sensor_pins[i], GPIO_MODE_OUTPUT);
//             gpio_set_level(sensor_pins[i], 1);
//         }
//         ets_delay_us(10); // Wait for capacitors to charge 10 us is enough

//         // Set pins to input to start discharging (sensing phase)
//         for (int i = 0; i < num_of_sensors; i++)
//         {
//             gpio_set_direction(sensor_pins[i], GPIO_MODE_INPUT);
//         }

//         int still_high = num_of_sensors; // Track how many sensors are still high
//         uint64_t start_time = esp_timer_get_time(); // Record start time
//         uint64_t timeout = 3000; // Timeout in microseconds which we will consider as no line detected to avoid large numbers

//         // Initialize sensor values to timeout
//         for (int i = 0; i < num_of_sensors; i++)
//         {
//             sensor_values[i] = timeout;
//         }
        
//         // Measure discharge time for each sensor
//         while (still_high > 0 && (esp_timer_get_time() - start_time) < timeout) // Continue until all sensors are low or timeout
//         {
//             uint64_t now = esp_timer_get_time(); // Current time

//             // Check each sensor
//             for (int i = 0; i < num_of_sensors; i++)
//             {
//                 // If sensor pin goes low, record the time it took to discharge
//                 if (gpio_get_level(sensor_pins[i]) == 0 && (now - start_time) < timeout) // If pin is low and within timeout
//                 {
//                     sensor_values[i] = now - start_time; // Record discharge time
//                     still_high--; // Decrement count of sensors still high
//                 }
//             }
//         }
//         for (size_t i = 0; i < num_of_sensors; i++)
//         {
//             ESP_LOGI(TAG,"The value of the sensor %d is %d", i+1, sensor_values[i]);
//         }
//         vTaskDelay(250/portTICK_PERIOD_MS);
//     }
    
// }


// //     printf("Hello, World!\n");
// // }