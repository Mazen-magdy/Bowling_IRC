// #include <stdio.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/i2c.h"
// #include "esp_log.h"

// #define I2C_MASTER_SCL_IO    22    // GPIO pin for SCL
// #define I2C_MASTER_SDA_IO    21    // GPIO pin for SDA
// #define I2C_MASTER_FREQ_HZ   400000
// #define I2C_MASTER_PORT      I2C_NUM_0

// #define MPU6050_ADDR         0x68  // MPU6050 I2C address
// #define PWR_MGMT_1          0x6B
// #define ACCEL_XOUT_H        0x3B
// #define GYRO_XOUT_H         0x43

// static const char *TAG = "MPU6050";

// // Angle variables
// static float pitch = 0.0, roll = 0.0, yaw = 0.0;
// static float gyro_prev_roll, gyro_prev_pitch;
// static uint32_t last_time = 0;

// // I2C initialization
// static void i2c_master_init() {
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = I2C_MASTER_FREQ_HZ,
//     };
//     i2c_param_config(I2C_MASTER_PORT, &conf);
//     i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
// }

// // Write byte to MPU6050 register
// static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data) {
//     uint8_t write_buf[2] = {reg_addr, data};
//     return i2c_master_write_to_device(I2C_MASTER_PORT, MPU6050_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
// }

// // Read bytes from MPU6050
// static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len) {
//     return i2c_master_write_read_device(I2C_MASTER_PORT, MPU6050_ADDR, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
// }

// // Initialize MPU6050
// static void mpu6050_init() {
//     // Wake up MPU6050 (set to 0)
//     mpu6050_register_write_byte(PWR_MGMT_1, 0x00);
//     vTaskDelay(100 / portTICK_PERIOD_MS);
// }

// // Read accelerometer data
// static void read_accel_data(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
//     uint8_t data[6];
//     mpu6050_register_read(ACCEL_XOUT_H, data, 6);
    
//     *accel_x = (int16_t)((data[0] << 8) | data[1]);
//     *accel_y = (int16_t)((data[2] << 8) | data[3]);
//     *accel_z = (int16_t)((data[4] << 8) | data[5]);
// }

// // Read gyroscope data
// static void read_gyro_data(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
//     uint8_t data[6];
//     mpu6050_register_read(GYRO_XOUT_H, data, 6);
    
//     *gyro_x = (int16_t)((data[0] << 8) | data[1]);
//     *gyro_y = (int16_t)((data[2] << 8) | data[3]);
//     *gyro_z = (int16_t)((data[4] << 8) | data[5]);
// }

// // Calculate angles from accelerometer (pitch and roll)
// static void calculate_accel_angles(int16_t accel_x, int16_t accel_y, int16_t accel_z, float *accel_pitch, float *accel_roll) {
//     // Convert to g-force (assuming ±2g range)
//     float ax = accel_x / 16384.0;
//     float ay = accel_y / 16384.0;
//     float az = accel_z / 16384.0;
    
//     // Calculate pitch and roll in radians
//     *accel_pitch = atan2(-ax, sqrt(ay * ay + az * az));
//     *accel_roll = atan2(ay, az);
    
//     // Convert to degrees
//     // *pitch = *pitch * 180.0 / M_PI;
//     // *roll = *roll * 180.0 / M_PI;
// }

// // Calculate angles using gyroscope integration (including yaw)
// static void calculate_gyro_angles(int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, float dt, float *gyro_pitch,float *gyro_roll) {
//     // Convert to degrees per second (assuming ±250°/s range)
//     float gx = gyro_x / 131.0;
//     float gy = gyro_y / 131.0;
//     float gz = gyro_z / 131.0;

//     float phi_dot   = gx + gy*sinf(roll)*tanf(pitch) + gz*cosf(roll)*tanf(pitch);
//     float theta_dot = gy*cosf(roll) - gz*sinf(roll);
    
//     // Integrate angular velocity to get angle
    
//     *gyro_pitch = gyro_prev_pitch + theta_dot*dt;
//     *gyro_roll  = gyro_prev_roll + phi_dot*dt;
//     // yaw += gz * dt;
// }

// // Complementary filter to combine accelerometer and gyroscope data
// static void complementary_filter(int16_t accel_x, int16_t accel_y, int16_t accel_z, 
//                                 int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, float dt) {
//     float accel_pitch, accel_roll;
//     float gyro_pitch, gyro_roll;
//     const float alpha = 0.98;  // Complementary filter coefficient
    
//     // Calculate angles from accelerometer
//     calculate_accel_angles(accel_x, accel_y, accel_z, &accel_pitch, &accel_roll);
    
//     // Calculate angles from gyroscope
//     calculate_gyro_angles(gyro_x, gyro_y, gyro_z, dt, &gyro_pitch, &gyro_roll);
    
//     // Combine using complementary filter
//     pitch = alpha * gyro_pitch + (1 - alpha) * accel_pitch;
//     roll  = alpha * gyro_roll + (1 - alpha) * accel_roll;
//     // Yaw is only from gyroscope (no accelerometer reference)

//     //setting the previuos readings
//     gyro_prev_pitch = gyro_pitch;
//     gyro_prev_roll = gyro_roll;
// }

// void app_main(void) {
//     ESP_LOGI(TAG, "Initializing I2C and MPU6050...");
    
//     // Initialize I2C
//     i2c_master_init();
    
//     // Initialize MPU6050
//     mpu6050_init();
    
//     int16_t accel_x, accel_y, accel_z;
//     int16_t gyro_x, gyro_y, gyro_z;
    
//     last_time = xTaskGetTickCount();
    
//     while (1) {
//         // Calculate time difference
//         uint32_t current_time = xTaskGetTickCount();
//         float dt = (current_time - last_time) * portTICK_PERIOD_MS / 1000.0;
//         last_time = current_time;
        
//         // Read sensor data
//         read_accel_data(&accel_x, &accel_y, &accel_z);
//         read_gyro_data(&gyro_x, &gyro_y, &gyro_z);
        
//         // Calculate all three angles using complementary filter
//         complementary_filter(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt);
        
//         // Print results
//         ESP_LOGI(TAG, "Angles: Pitch=%.2f°, Roll=%.2f°, Yaw=%.2f°", pitch, roll, yaw);
//         ESP_LOGI(TAG, "----------------------------------------");
        
//         vTaskDelay(250 / portTICK_PERIOD_MS);  // 20Hz update
//     }
// }

























#include "main.h"

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

// wakeup mpu6050
void mpu6050_init() {
    // Wake up MPU6050
    mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
}


//get the data from both the accelerometer
void get_accel_data(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
    uint8_t data[6];
    i2c_master_read_bytes(MPU6050_ACCEL_XOUT_H, data, 6);

    *accel_x = (int16_t)(data[0] << 8 | data[1]);
    *accel_y = (int16_t)(data[2] << 8 | data[3]);
    *accel_z = (int16_t)(data[4] << 8 | data[5]);
}

//get the data from both the gyroscope
void get_gyro_data(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    uint8_t data[6];
    i2c_master_read_bytes(MPU6050_GYRO_XOUT_H, data, 6);

    *gyro_x = (int16_t)(data[0] << 8 | data[1]);
    *gyro_y = (int16_t)(data[2] << 8 | data[3]);
    *gyro_z = (int16_t)(data[4] << 8 | data[5]);
}

// calculate the angles from accelerometer
void calculate_angles(int16_t ax, int16_t ay, int16_t az, float *pitch, float *roll) {
    *pitch = atan2(ax, sqrtf((ay * ay + az * az))) * RAD_TO_DEG;
    *roll = atan2(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
}


// initialize i2c channel
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER, // set the type whether it is master of slave
        .sda_io_num = I2C_MASTER_SDA_IO, // SDA pin 21
        .scl_io_num = I2C_MASTER_SCL_IO, // SCL pin 22
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // pullup enable for SDA
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // pullup enable for SCL
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // frequency of I2C
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// optional scan for I2C address
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

double get_angle()
{
    // Optional but useful
    i2c_scan();

    // initialize the I2C protocol
    i2c_master_init();
    
    // wakes up the mpu6050
    mpu6050_init();

     // int16_t gx, gy, gz; //used if you want to use gyro data 
        int16_t ax, ay, az;
        float pitch, roll;

        get_accel_data(&ax, &ay, &az);
        calculate_angles(ax, ay, az, &pitch, &roll);

        return pitch;
}


void app_main(void) {
    while (true)
    {
        printf("%f\n", get_angle());
    }
    
    
}
