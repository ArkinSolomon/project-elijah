#pragma once

#include "standard_flight_phase_controller.h"

struct OverrideState;

class OverrideFlightPhaseController final : public elijah_state_framework::std_helpers::StandardFlightPhaseController<OverrideState>
{
protected:
  [[nodiscard]] bool is_calibrated() const override;
  void log_message(const std::string& msg) const override;
};
