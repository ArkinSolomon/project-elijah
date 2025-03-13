#pragma once

#include "battery.h"
#include "reliable_sensors/bmp_280/payload_reliable_bmp_280.h"
#include "reliable_sensors/mpu_6050/payload_reliable_mpu_6050.h"

inline PayloadReliableMPU6050* mpu6050 = nullptr;
inline PayloadReliableBMP280* bmp280 = nullptr;
inline Battery* battery = nullptr;

void sensors_init();