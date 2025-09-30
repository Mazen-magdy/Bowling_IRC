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

// I2C write (custom function, renamed to avoid conflict)
esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// I2C read multiple bytes
esp_err_t i2c_master_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void mpu6050_init() {
    // Wake up MPU6050
    mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void get_raw_data(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
    uint8_t data[6];
    i2c_master_read_bytes(MPU6050_ACCEL_XOUT_H, data, 6);

    *accel_x = (int16_t)(data[0] << 8 | data[1]);
    *accel_y = (int16_t)(data[2] << 8 | data[3]);
    *accel_z = (int16_t)(data[4] << 8 | data[5]);
}

void calculate_angles(int16_t ax, int16_t ay, int16_t az, float *pitch, float *roll) {
    *pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    *roll = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
}

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

#include "driver/i2c.h"

void i2c_scan() {
    printf("Scanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("Found device at 0x%02X\n", addr);
        }
    }
}




void app_main(void) {
    i2c_scan();
    i2c_master_init();
    mpu6050_init();

    while (1) {
        int16_t ax, ay, az;
        float pitch_before, roll_before, pitch_after, roll_after;

        get_raw_data(&ax, &ay, &az);
        calculate_angles(ax, ay, az, &pitch_before, &roll_before);

        // Apply a simple low-pass filter (LPF) to the angles
        if (abs(roll_after - roll_before) < 1.0)
        {
            roll_after = roll_before;
        }
        if (abs(pitch_after - pitch_before) < 1.0)
        {
            pitch_after = pitch_before;
        }
        

        printf("Pitch: %.2f°, Roll: %.2f°\n", pitch_before, roll_before);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
