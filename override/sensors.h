#pragma once

#include "battery.h"
#include "reliable_sensors/bmp_280/override_reliable_bmp_280.h"
#include "reliable_sensors/mpu_6050/override_reliable_mpu_6050.h"

inline OverrideReliableMPU6050* mpu6050 = nullptr;
inline OverrideReliableBMP280* bmp280 = nullptr;
inline Battery* battery = nullptr;

void sensors_init();
