#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp32/rom/ets_sys.h"
#include "esp_log.h"

#define NUM_SENSORS 8
#define BLACK_THRESHOLD 2500   // Tune this based on surface
#define BLACK_SENSOR_MIN_COUNT 5  // How many sensors must detect black
#define WHITE_LOST_COUNT 2     // Hysteresis: how many consecutive white reads before triggering "line lost"


void QTR_8RC();

#endif // MAIN_H