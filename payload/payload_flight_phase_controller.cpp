#include "payload_flight_phase_controller.h"

#include "payload_sensors.h"
#include "payload_state_manager.h"

elijah_state_framework::std_helpers::StandardFlightPhase PayloadFlightPhaseController::update_phase(const StandardFlightPhase current_phase, const std::deque<PayloadState>& state_history)
{
  const double last_coast_alt = max_coast_alt;
  const StandardFlightPhase new_phase = StandardFlightPhaseController::update_phase(current_phase, state_history);

  if (max_coast_alt != last_coast_alt)
  {
    reliable_clock->get_ds_1307().write_custom_register(
      MAX_COAST_ALT_REG, reinterpret_cast<uint8_t*>(&max_coast_alt), sizeof(max_coast_alt));
  }

  return new_phase;
}

elijah_state_framework::std_helpers::StandardFlightPhase PayloadFlightPhaseController::predict_phase(const StandardFlightPhase last_known_phase,
                                                                                                     const std::deque<PayloadState>& state_history)
{
  if (last_known_phase != StandardFlightPhase::PREFLIGHT)
  {
    reliable_clock->get_ds_1307().read_custom_register(MAX_COAST_ALT_REG, reinterpret_cast<uint8_t*>(&max_coast_alt),
                                                       sizeof(max_coast_alt));
  }
  return StandardFlightPhaseController::predict_phase(last_known_phase, state_history);
}

bool PayloadFlightPhaseController::is_calibrated() const
{
  return payload_state_manager->get_persistent_storage()->get_uint8(PayloadPersistentKey::IsCalibrated) > 0;
}

void PayloadFlightPhaseController::log_message(const std::string& msg) const
{
  payload_state_manager->log_message(msg);
}
