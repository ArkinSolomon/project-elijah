#include "battery.h"

#include <cmath>
#include <format>
#include <hardware/adc.h>

#include "main.h"

void battery::collect_bat_information(CollectionData& collection_data)
{
  if (adc_read_results.size() == ADC_SAMPLE_COUNT)
  {
    adc_read_results.erase(adc_read_results.begin());
  }

  const uint16_t result = adc_read();
  adc_read_results.push_back(result);

  uint32_t result_sum = 0;
  for (const uint16_t result_sample : adc_read_results)
  {
    result_sum += result_sample;
  }
  double average_result = static_cast<double>(result_sum) / adc_read_results.size();

  constexpr double round_factor = pow(10, VOLTAGE_DECIMAL_PLACES);
  average_result = round(average_result * round_factor) / round_factor;

  // Divide by 16 is kind of like dropping the last 4 bits off the end (12 bit adc, also cf = vref / 1 << (12-4))
  constexpr double conversion_factor = 3.3f / (1 << 8);
  const double voltage_result = average_result / 16 * conversion_factor;

  collection_data.bat_voltage = voltage_result * 3; // multiply by 3 due to voltage divider
  collection_data.bat_percent = calc_charge_percent(collection_data.bat_voltage);
}

double battery::calc_charge_percent(const double voltage)
{
  return 2.4095 * pow(voltage, 4) - 75.641 * pow(voltage, 3) + 889.29 * pow(voltage, 2) - 4639.7 * voltage + 9062.4;
}
