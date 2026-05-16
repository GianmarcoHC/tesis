#pragma once

#include <stdint.h>

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} imu_raw_t;

void mpu6050_init();
void mpu6050_read(imu_raw_t *data);
