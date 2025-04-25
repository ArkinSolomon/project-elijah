#include "airbrakes_flight_phase_controller.h"

#include "airbrakes_state_manager.h"

bool AirbrakesFlightPhaseController::is_calibrated() const
{
  return airbrakes_state_manager->get_persistent_storage()->get_uint8(AirbrakesPersistentKey::IsCalibrated) > 0;
}

bool AirbrakesFlightPhaseController::is_altimeter_faulted() const
{
  return airbrakes_state_manager->is_faulted(AirbrakesFaultKey::BMP280);
}

void AirbrakesFlightPhaseController::log_message(const std::string& msg) const
{
  airbrakes_state_manager->log_message(msg);
}

void AirbrakesFlightPhaseController::set_apogee(double apogee) const
{
  airbrakes_state_manager->get_persistent_storage()->set_double(AirbrakesPersistentKey::ApogeeReached, apogee);
  airbrakes_state_manager->get_persistent_storage()->commit_data();
}
