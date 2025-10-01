#ifndef SERIAL_H
#define SERIAL_H

#include <stdbool.h>

// Serial communication functions
void serial_init(void);
void serial_send_float(float value);
float serial_receive_float(void);
bool serial_available(void);
void serial_send_string(const char* str);
void serial_send_int(int value);

#endif // SERIAL_H