#include "override_flight_phase_controller.h"

#include "override_state_manager.h"

void OverrideFlightPhaseController::extract_state_data(const OverrideState state, double& accel_x, double& accel_y,
                                                       double& accel_z,
                                                       double& altitude) const
{
  accel_x = state.accel_x;
  accel_y = state.accel_y;
  accel_z = state.accel_z;
  altitude = state.altitude;
}

bool OverrideFlightPhaseController::is_calibrated() const
{
  return override_state_manager->get_persistent_storage()->get_double(OverridePersistentStateKey::IsCalibrated) > 0;
}

void OverrideFlightPhaseController::log_message(const std::string& msg) const
{
  override_state_manager->log_message(msg);
}
