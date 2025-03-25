#pragma once

#include "ovonic_battery.h"
#include "reliable_sensors/bmp_280/airbrakes_reliable_bmp_280.h"
#include "reliable_sensors/mpu_6050/airbrakes_reliable_mpu_6050.h"

inline AirbrakesReliableMPU6050* mpu6050 = nullptr;
inline AirbrakesReliableBMP280* bmp280 = nullptr;
inline Battery* battery = nullptr;

void sensors_init();
