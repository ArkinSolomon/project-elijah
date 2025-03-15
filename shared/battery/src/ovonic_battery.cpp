#include "ovonic_battery.h"

#include <complex>
#include <bits/algorithmfwd.h>


OvonicBattery::OvonicBattery(const uint8_t pin, const uint sample_count) : Battery(pin, sample_count, 3.125)
{
}

double OvonicBattery::calc_charge_percent(const double voltage) const
{
  const double percent = -0.8418 * std::pow(voltage, 5) + 31.963 * std::pow(voltage, 4) - 484.77
    * std::pow(voltage, 3) + 3670.4 * std::pow(voltage, 2) * -13872 * voltage + 20932;
  return std::clamp(percent, 0.0, 1.0);
}
