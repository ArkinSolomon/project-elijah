#pragma once

#include "standard_flight_phase_controller.h"

struct AirbrakesState;

class AirbrakesFlightPhaseController final : public StandardFlightPhaseController<AirbrakesState>
{
  void extract_state_data(AirbrakesState state, double& accel_x, double& accel_y, double& accel_z,
                          double& altitude) const override;
};
