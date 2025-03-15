#include "liperior_battery.h"

#include <cstdint>
#include <map>
#include <bits/algorithmfwd.h>


LiperiorBattery::LiperiorBattery(const uint8_t pin, const uint sample_count) : Battery(pin, sample_count, 8)
{
}

double LiperiorBattery::calc_charge_percent(const double voltage) const
{
  constexpr static std::map<double, double> recorded_values = {
    {25.2, 1}, {24.3, 0.93}, {24, 0.86}, {23.7, 0.80}, {23.4, 0.73}, {23.1, 0.66}, {22.8, 0.59}, {22.5, 0.53},
    {22.5, 0.46}, {22.5, 0.39}, {22.2, 0.32}, {22.2, 0.25}, {22.2, 0.19}, {21.9, 0.12}, {21.3, 0.5}, {21, 0.0},
  };
  const auto min_it = recorded_values.rbegin();
  if (voltage <= min_it->first)
  {
    return 0;
  }

  const auto max_it = recorded_values.begin();
  if (voltage >= max_it->first)
  {
    return 1;
  }

  const auto upper = recorded_values.lower_bound(voltage);
  if (upper->first == voltage || upper == recorded_values.begin())
  {
    return upper->second; // Exact match
  }

  const auto lower = std::prev(upper);

  const double v1 = lower->first, p1 = lower->second;
  const double v2 = upper->first, p2 = upper->second;

  const double percent = p1 + ((voltage - v1) / (v2 - v1)) * (p2 - p1);

  return std::clamp(percent, 0.0, 1.0);
}
