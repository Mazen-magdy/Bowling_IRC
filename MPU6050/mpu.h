#ifndef MPU6050_H
#define MPU6050_H
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_ADDR                0x68
#define MPU6050_PWR_MGMT_1         0x6B
#define MPU6050_ACCEL_XOUT_H       0x3B
#define RAD_TO_DEG                  57.2957795131

void MPU6050();


#endif