#pragma once

#include "standard_flight_phase_controller.h"

struct AirbrakesState;

class AirbrakesFlightPhaseController final : public elijah_state_framework::std_helpers::StandardFlightPhaseController<AirbrakesState>
{
protected:
  [[nodiscard]] bool is_calibrated() const override;
  void log_message(const std::string& msg) const override;
  void set_apogee(double apogee) const override;
};
