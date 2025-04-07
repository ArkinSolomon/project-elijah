#include "airbrakes_flight_phase_controller.h"

#include "airbrakes_state_manager.h"

void AirbrakesFlightPhaseController::extract_state_data(const AirbrakesState state, double& accel_x, double& accel_y,
                                                        double& accel_z,
                                                        double& altitude) const
{
  accel_x = state.accel_x;
  accel_y = state.accel_y;
  accel_z = state.accel_z;
  altitude = state.altitude;
}

bool AirbrakesFlightPhaseController::is_calibrated() const
{
  return airbrakes_state_manager->get_persistent_storage()->get_uint8(AirbrakesPersistentStateKey::IsCalibrated) > 0;
}

void AirbrakesFlightPhaseController::log_message(const std::string& msg) const
{
  airbrakes_state_manager->log_message(msg);
}
