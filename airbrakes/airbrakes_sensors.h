#pragma once

#include "airbrakes_state_manager.h"
#include "ovonic_battery.h"
#include "reliable_bmp_280.h"
#include "reliable_mpu_6050.h"

inline ReliableBMP280<AIRBRAKES_STATE_TEMPLATE_TYPES>* bmp280 = nullptr;
inline ReliableMPU6050<AIRBRAKES_STATE_TEMPLATE_TYPES>* mpu6050 = nullptr;
inline Battery* battery = nullptr;

void sensors_init();
