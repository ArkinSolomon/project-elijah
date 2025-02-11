#pragma once
#include <hardware/pio.h>

#include "usb_communication.h"
#include "sensors/ds_1307/ds_1307.h"

#define MAX_UPDATES_PER_SECOND 50

struct CollectionData
{
  tm time_inst;
  int32_t pressure;
  double temperature;
  double altitude;
  double accel_x, accel_y, accel_z;
  double gyro_x, gyro_y, gyro_z;
  double bat_voltage, bat_percent;
};

inline uint aprs_sm;
inline PIO aprs_pio;

void pin_init();
void flight_loop(CollectionData& collection_data, absolute_time_t last_loop_start_time );
void landed_loop(CollectionData& collection_data, absolute_time_t last_loop_start_time );