#include "mpu.h"


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
    // printf("Scanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            // printf("Found device at 0x%02X\n", addr);
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
void update_angles(struct mpu6050_Data* mpu_data) {
    mpu_data->angle = get_angle();
    mpu_data->angle_error = mpu_data->angle_target - mpu_data->angle;
}
