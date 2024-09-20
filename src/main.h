#pragma once
#include "sensors/ds_1307/ds_1307.h"

void pin_init();
void clock_loop(ds_1307::TimeInstance& time_inst);
void pressure_loop(int32_t& press, double& temp, double& altitude, double& alt_ft, double& temp_f);
