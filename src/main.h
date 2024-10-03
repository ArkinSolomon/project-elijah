#pragma once
#include "usb_communication.h"
#include "sensors/ds_1307/ds_1307.h"

struct CollectionData
{
  ds_1307::TimeInstance time_inst;
  int32_t pressure;
  double temperature;
  double altitude;
};

void pin_init();