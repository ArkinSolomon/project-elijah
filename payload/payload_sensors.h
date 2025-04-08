#pragma once

#include "battery.h"
#include "payload_state_manager.h"
#include "reliable_bmp_280.h"
#include "reliable_mpu_6050.h"
#include "reliable_clock/reliable_clock.h"

inline ReliableBMP280<PAYLOAD_STATE_TEMPLATE_TYPES>* bmp280 = nullptr;
inline ReliableMPU6050<PAYLOAD_STATE_TEMPLATE_TYPES>* mpu6050 = nullptr;
inline Battery* battery = nullptr;
inline ReliableClock* reliable_clock = nullptr;

void sensors_init();
