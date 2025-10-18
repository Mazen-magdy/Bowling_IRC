#include "Serial.h"
#include "driver/uart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define UART_PORT UART_NUM_1
#define BUF_SIZE 1024
#define TX_PIN 10
#define RX_PIN 9

void serial_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    // Configure UART parameters
    uart_param_config(UART_PORT, &uart_config);
    
    // Set UART pins (TX, RX, RTS, CTS)
    uart_set_pin(UART_PORT, TX_PIN, RX_PIN, -1, -1);
    
    // Install UART driver
    uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
}

void serial_send_float(float value) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.6f\n", value);
    uart_write_bytes(UART_PORT, buffer, strlen(buffer));
}

float serial_receive_float(void) {
    char buffer[32];
    int pos = 0;
    
    while (1) {
        uint8_t data;
        int len = uart_read_bytes(UART_PORT, &data, 1, portMAX_DELAY); // Wait forever
        
        if (len > 0) {
            if (data == '\n' || data == '\r') {
                if (pos > 0) {
                    buffer[pos] = '\0';
                    return atof(buffer);  // Convert string to float
                }
            } else if (pos < sizeof(buffer) - 1) {
                buffer[pos++] = data;
            }
        }
    }
}

bool serial_available(void) {
    size_t available;
    uart_get_buffered_data_len(UART_PORT, &available);
    return available > 0;
}

void serial_send_string(const char* str) {
    uart_write_bytes(UART_PORT, str, strlen(str));
    uart_write_bytes(UART_PORT, "\n", 1);  // Add newline
}

void serial_send_int(int value) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%d\n", value);
    uart_write_bytes(UART_PORT, buffer, strlen(buffer));
}