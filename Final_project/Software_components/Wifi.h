#ifndef WIFI_H
#define WIFI_H

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
void socket_create_and_connect();
void send_or_receive(char *message);
void close_socket();
void wifi_start();


// Define the destination ip & port
#define port 5005
#define ip_address "192.168.88.78"

// Your network credentials
#define WIFI_SSID      "bowling"
#define WIFI_PASS      "12345678"


// Event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// The event handler is internal to the Wifi implementation (defined static in Wifi.c)
// Do not expose a static prototype from the public header.

extern int sock;
extern char received_buffer[128]; // the received message will be stored here

#endif // WIFI_H