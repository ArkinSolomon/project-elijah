#pragma once

#include "battery.h"

class LiperiorBattery final : public Battery
{
public:
  LiperiorBattery(uint8_t pin, uint sample_count);

  [[nodiscard]] double calc_charge_percent(double voltage) const override;
};
