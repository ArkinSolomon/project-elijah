#pragma once

#include "bmp_280.h"
#include "mpu_6050.h"
#include "battery.h"

inline MPU6050* mpu6050 = nullptr;
inline BMP280* bmp280 = nullptr;
inline Battery* battery = nullptr;

void sensors_init();