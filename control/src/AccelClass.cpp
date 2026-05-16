#include "AccelClass.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA 21
#define I2C_MASTER_SCL 22
#define I2C_MASTER_FREQ_HZ 400000
#define MPU6050_ADDR 0x68
static const char *TAG = "MPU6050";

static esp_err_t mpu_write(uint8_t reg, uint8_t data) {
    return i2c_master_write_to_device(
        I2C_MASTER_NUM, MPU6050_ADDR, (uint8_t[]){reg, data}, 2, 1000 / portTICK_PERIOD_MS);
}

static esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(
        I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

void mpu6050_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ
        },
        .clk_flags = 0,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    // Wake up the sensor
    mpu_write(0x6B, 0x00);

    // Accelerometer ±2g, gyro ±250dps
    mpu_write(0x1B, 0x00);
    mpu_write(0x1C, 0x00);

    ESP_LOGI(TAG, "MPU6050 initialized");
}

void mpu6050_read(imu_raw_t *d) {
    uint8_t raw[14];
    mpu_read(0x3B, raw, 14);

    int16_t ax = (raw[0] << 8) | raw[1];
    int16_t ay = (raw[2] << 8) | raw[3];
    int16_t az = (raw[4] << 8) | raw[5];
    int16_t gx = (raw[8] << 8) | raw[9];
    int16_t gy = (raw[10] << 8) | raw[11];
    int16_t gz = (raw[12] << 8) | raw[13];

    d->ax = ax / 16384.0f;
    d->ay = ay / 16384.0f;
    d->az = az / 16384.0f;
    d->gx = gx / 131.0f;
    d->gy = gy / 131.0f;
    d->gz = gz / 131.0f;
}
