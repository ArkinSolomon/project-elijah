#pragma once

#include "standard_flight_phase_controller.h"

struct OverrideState;

class OverrideFlightPhaseController final : public StandardFlightPhaseController<OverrideState>
{
protected:
  void extract_state_data(OverrideState state, double& accel_x, double& accel_y, double& accel_z,
                          double& altitude) const override;
  [[nodiscard]] bool is_calibrated() const override;
  void log_message(const std::string& msg) const override;
};
