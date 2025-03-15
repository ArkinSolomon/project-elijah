#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <pico/types.h>

struct PayloadState;

class Battery
{
public:
  virtual ~Battery() = default;
  Battery(uint8_t pin, uint sample_count, double bat_scale);

  double get_voltage();
  [[nodiscard]] virtual double calc_charge_percent(double voltage) const = 0;

protected:
  static double voltage_map_interp(double voltage, const std::map<double, double>& voltage_map);

private:
  uint8_t adc_input;

  uint sample_count;
  double bat_scale;

  std::deque<uint16_t> adc_read_results;
};
