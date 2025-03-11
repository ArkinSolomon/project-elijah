#pragma once

#include "standard_flight_phase_controller.h"

struct OverrideState;

class OverrideFlightPhaseController final : public StandardFlightPhaseController<OverrideState>
{
  void extract_state_data(OverrideState state, double& accel_x, double& accel_y, double& accel_z,
                          double& altitude) const override;
};
