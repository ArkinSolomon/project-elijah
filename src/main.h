#pragma once
#include "sensors/ds_1307/ds_1307.h"

struct CollectionData
{
  ds_1307::TimeInstance time_inst;
  int32_t pressure;
  double temperature;
  double altitude;
};

void pin_init();
void clock_loop(CollectionData& collection_data);
void pressure_loop(CollectionData& collection_data);
