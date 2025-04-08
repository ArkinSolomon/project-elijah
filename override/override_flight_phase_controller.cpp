#include "override_flight_phase_controller.h"

#include "override_state_manager.h"

bool OverrideFlightPhaseController::is_calibrated() const
{
  return override_state_manager->get_persistent_storage()->get_double(OverridePersistentStateKey::IsCalibrated) > 0;
}

void OverrideFlightPhaseController::log_message(const std::string& msg) const
{
  override_state_manager->log_message(msg);
}
