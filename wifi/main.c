#include "main.h"


// FreeRTOS event group to handle Wi-Fi events
static EventGroupHandle_t s_wifi_event_group;


// TAG for ESP_LOG messages
static const char *TAG = "wifi_station";
static const char *SOCKET_TAG = "Socket_client";

// Wi-Fi event handler function
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // The station has been initialized, now we can connect
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // The connection was lost or failed
        esp_wifi_connect(); // Try to reconnect

        /*-------we will delete them for now and we will see-------------*/

        // xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        // xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);

        ESP_LOGI(TAG, "Failed to connect to AP. Retrying...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // Success! We got an IP address from the router
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

        // esp_ip = event->ip_info.ip;
        
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    // 1. Initialize the underlying TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // 2. Create the default event loop (handles system events like Wi-Fi)
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 3. Create the Wi-Fi station interface
    esp_netif_create_default_wifi_sta();

    // 4. Initialize the Wi-Fi driver with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 5. Register our event handler function for Wi-Fi and IP events
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // 6. Configure the Wi-Fi station with our SSID and password
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK, // Most common security and this is used when we're not sure about the security type
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // 7. Start the Wi-Fi driver
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished. Attempting to connect...");
}

void wifi_start()
{
        // Initialize NVS (Non-Volatile Storage) - essential for Wi-Fi to store settings
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create the event group to track connection status
    s_wifi_event_group = xEventGroupCreate();

    // Initialize and start the Wi-Fi
    wifi_init_sta();

    // Wait until we are connected (WIFI_CONNECTED_BIT) or fail (WIFI_FAIL_BIT)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, // Don't clear the bits on exit
            pdFALSE, // Wait for EITHER bit
            portMAX_DELAY); // Wait forever

    // Check which event caused us to exit the wait
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP successfully!");
        // YOUR APPLICATION CODE STARTS HERE!
        // e.g., Start a web server, connect to MQTT, etc.

        // socket_client_setup_1();

        while(1) {
            printf("Wi-Fi is connected! Doing my job...\n");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to AP.");
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void app_main(void)
{
    wifi_start();
}

void socket_client_setup_1()
{
    char *message = "Hello from esp!";
    char rx_buffer[128];
    struct sockaddr_in dest_addr;


    //configure client ?? address
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    dest_addr.sin_addr.s_addr = inet_addr(ip_address);   
    
    // creating socket
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Failed to create socket !");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(SOCKET_TAG, "Socket created successfuly, connecting to %s:%d...", ip_address, port);

    // connecting the socket
    /*connect the socket with the address given and the size of memory needed*/

    int feedback_fron_connect;
    feedback_fron_connect = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (feedback_fron_connect < 0)
    {
        ESP_LOGE(SOCKET_TAG, "Failed to connect to the socket with error no: %d", feedback_fron_connect);
        vTaskDelay(pdMS_TO_TICKS(500));
        // close(sock);
        // vTaskDelete(NULL);
        // return;
        }/* code */
    
    
    ESP_LOGI(SOCKET_TAG, "client connected (we connected to the server)! ");

    
    while (1)
    {
        // send a message with handling the feedback from it
        int feedack_from_send = send(sock, message, strlen(message), 0);
        if (feedack_from_send < 0)
        {
            ESP_LOGE(SOCKET_TAG, "Error sending the message: error no %d", feedack_from_send);
        }
        else if (feedack_from_send == 0)
        {
            ESP_LOGI(SOCKET_TAG, "Connection closed by the server!!");
        }
        else
        {
            ESP_LOGI(SOCKET_TAG, "message sent!");
        }
        
        
        
        // recieve response with handling the feedback from it
        int feedback_from_recieve = recv(sock, rx_buffer, sizeof(rx_buffer)-1, 0);
        if (feedback_from_recieve < 0)
        {
            ESP_LOGE(SOCKET_TAG, "Error recieving the message : error no %d", feedback_from_recieve);
        }
        else if (feedback_from_recieve == 0)
        {
            ESP_LOGI(SOCKET_TAG, "Connection closed by the server");
        }
        else
        {
            ESP_LOGI(SOCKET_TAG, "Message received: %s", rx_buffer);
            if (strcmp(rx_buffer, "close") == 0)
            {
                break;
            }
            // response_from_vision = atof(rx_buffer);
        }
        
        // Delay between the messages
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }

    // Close the socket 
    close(sock);
    ESP_LOGI(SOCKET_TAG, "Socket closed!");
}
