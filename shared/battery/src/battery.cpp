#include "battery.h"

#include <cmath>
#include <vector>
#include <bits/stl_algo.h>
#include <hardware/adc.h>

Battery::Battery(uint8_t pin, uint sample_count, double bat_scale) : sample_count(sample_count), bat_scale(bat_scale)
{
  if (pin == 26)
  {
    adc_input = 0;
  }
  else if (pin == 27)
  {
    adc_input = 1;
  }
  else if (pin == 28)
  {
    adc_input = 2;
  }
  else
  {
    assert(false);
  }

  adc_read_results.reserve(sample_count);
}

double Battery::get_voltage()
{
  if (adc_get_selected_input() != adc_input)
  {
    adc_select_input(adc_input);
  }

  if (adc_read_results.size() == sample_count)
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
  const double average_result = static_cast<double>(result_sum) / adc_read_results.size();

  // Divide by 16 is kind of like dropping the last 4 bits off the end (12 bit adc, also cf = vref / 1 << (12-4))
  constexpr double conversion_factor = 3.3f / (1 << 8);
  const double voltage_result = average_result / 16 * conversion_factor;

  return voltage_result * bat_scale; // we only read 32% of voltage
}