#include "payload_flight_phase_controller.h"

#include "payload_state_manager.h"

void PayloadFlightPhaseController::extract_state_data(const PayloadState state, double& accel_x, double& accel_y, double& accel_z,
                                                      double& altitude) const
{
  accel_x = state.accel_x;
  accel_y = state.accel_y;
  accel_z = state.accel_z;
  altitude = state.altitude;
}
