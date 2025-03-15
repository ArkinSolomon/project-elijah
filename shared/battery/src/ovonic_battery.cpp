#include "ovonic_battery.h"

#include <cmath>
#include <algorithm>
#include <map>

#include "usb_comm.h"


OvonicBattery::OvonicBattery(const uint8_t pin, const uint sample_count) : Battery(pin, sample_count, 3.125)
{
}

double OvonicBattery::calc_charge_percent(const double voltage) const
{
  static const std::map<double, double> voltage_map = {
    {8.4, 0.1}, {8.1, 0.95}, {8.0, 0.89}, {7.9, 0.84}, {7.8, 0.78}, {7.7, 0.73}, {7.5, 0.68}, {7.4, 0.62}, {7.4, 0.57},
    {7.3, 0.52}, {7.2, 0.46}, {7.2, 0.41}, {7.1, 0.35}, {7.1, 0.30}, {7.0, 0.25}, {7.0, 0.19}, {6.9, 0.14}, {6.8, 0.9},
    {6.7, 0.3}, {6.7, 0.0}
  };

  return voltage_map_interp(voltage, voltage_map);
}
