#pragma once

#include "standard_flight_phase_controller.h"

struct PayloadState;

class PayloadFlightPhaseController final : public StandardFlightPhaseController<PayloadState>
{
  void extract_state_data(PayloadState state, double& accel_x, double& accel_y, double& accel_z,
                          double& altitude) const override;
};
