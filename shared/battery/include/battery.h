#pragma once

#include <cstdint>
#include <vector>
#include <pico/types.h>

struct PayloadState;

class Battery
{
public:
  virtual ~Battery() = default;
  Battery(uint8_t pin, uint sample_count, double bat_scale);

  double get_voltage();
  [[nodiscard]] virtual double calc_charge_percent(double voltage) const = 0;

private:
  uint8_t adc_input;

  uint sample_count;
  double bat_scale;

  std::vector<uint16_t> adc_read_results;
};
