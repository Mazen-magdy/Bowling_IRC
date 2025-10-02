#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h" // Non-Volatile Storage (NVS) is required to store Wi-Fi config
#include "esp_err.h"
#include "esp_system.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <string.h>


/*-----Functions declarations-----*/
void socket_client_setup_1();


// float response_from_vision = 0; // global variable
#define port 5555
#define ip_address "192.168.175.16"

// Your network credentials
#define WIFI_SSID      "bowling"
#define WIFI_PASS      "12345678"


// Event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data);

void socket_client_setup_1();

void wifi_start();

#endif // MAIN_H