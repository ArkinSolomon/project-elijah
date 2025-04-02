#include "airbrakes_reliable_bmp_280.h"

#include "airbrakes_state_manager.h"
#include "pin_outs.h"

AirbrakesReliableBMP280::AirbrakesReliableBMP280(AirbrakesStateManager* override_state_manager): ReliableBMP280(
  override_state_manager, AirbrakesFaultKey::BMP280, spi0, BMP_CS_PIN, AirbrakesPersistentStateKey::SeaLevelPressure)
{
}

void AirbrakesReliableBMP280::update_state(AirbrakesState& state, const int32_t pressure, const double temperature,
                                          const double altitude) const
{
  state.pressure = pressure;
  state.temperature = temperature;
  state.altitude = altitude - this->get_framework()->get_persistent_storage()->get_double(
    AirbrakesPersistentStateKey::GroundAltitude);
}
