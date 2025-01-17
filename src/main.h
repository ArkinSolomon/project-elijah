#pragma once
#include "usb_communication.h"
#include "sensors/ds_1307/ds_1307.h"

#define MAX_UPDATES_PER_SECOND 50

struct CollectionData
{
  ds_1307::TimeInstance time_inst;
  int32_t pressure;
  double temperature;
  double altitude;
  double accel_x, accel_y, accel_z;
};

void pin_init();