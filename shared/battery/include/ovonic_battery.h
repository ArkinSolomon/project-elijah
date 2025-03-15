#pragma once
#include "battery.h"

class OvonicBattery final : public Battery
{
public:
  OvonicBattery(uint8_t pin, uint sample_count);

  [[nodiscard]] double calc_charge_percent(double voltage) const override;
};
