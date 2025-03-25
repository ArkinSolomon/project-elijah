#include "airbrakes_flight_phase_controller.h"

#include "airbrakes_state_manager.h"

void AirbrakesFlightPhaseController::extract_state_data(const AirbrakesState state, double& accel_x, double& accel_y, double& accel_z,
                                                        double& altitude) const
{
  accel_x = state.accel_x;
  accel_y = state.accel_y;
  accel_z = state.accel_z;
  altitude = state.altitude;
}
