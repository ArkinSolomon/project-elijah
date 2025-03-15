#include "liperior_battery.h"

#include <cstdint>
#include <map>
#include <bits/algorithmfwd.h>

LiperiorBattery::LiperiorBattery(const uint8_t pin, const uint sample_count) : Battery(pin, sample_count, 8)
{
}

double LiperiorBattery::calc_charge_percent(const double voltage) const
{
  const static std::map<double, double> recorded_values = {
    {25.2, 1}, {24.3, 0.93}, {24, 0.86}, {23.7, 0.80}, {23.4, 0.73}, {23.1, 0.66}, {22.8, 0.59}, {22.5, 0.53},
    {22.5, 0.46}, {22.5, 0.39}, {22.2, 0.32}, {22.2, 0.25}, {22.2, 0.19}, {21.9, 0.12}, {21.3, 0.5}, {21, 0.0},
  };

  return voltage_map_interp(voltage, recorded_values);
}
