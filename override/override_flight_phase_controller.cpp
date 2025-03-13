#include "override_flight_phase_controller.h"

#include "override_state_manager.h"

void OverrideFlightPhaseController::extract_state_data(const OverrideState state, double& accel_x, double& accel_y, double& accel_z,
                                                       double& altitude) const
{
  accel_x = state.accel_x;
  accel_y = state.accel_y;
  accel_z = state.accel_z;
  altitude = state.altitude;
}
