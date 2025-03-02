#pragma once

#include <cstdint>
#include <vector>
#include <pico/types.h>

struct CollectionData;

class Battery
{
public:
  Battery(uint8_t pin, uint sample_count, double bat_scale);

  double get_voltage();
  double calc_charge_percent(double voltage);

private:
  uint8_t adc_input;

  uint sample_count;
  double bat_scale;

  std::vector<uint16_t> adc_read_results;
};
