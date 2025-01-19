#pragma once

#define ADC_SAMPLE_COUNT 128
#define VOLTAGE_DECIMAL_PLACES 2

#include <cstdint>
#include <vector>

struct CollectionData;

namespace battery
{
  inline std::vector<uint16_t> adc_read_results;

  void collect_bat_information(CollectionData& collection_data);
  double calc_charge_percent(double voltage);
}
