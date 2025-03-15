#include "payload_reliable_bmp_280.h"

#include "payload_state_manager.h"
#include "pin_outs.h"

PayloadReliableBMP280::PayloadReliableBMP280(PayloadStateManager* payload_state_manager): ReliableBMP280(
  payload_state_manager, OverrideFaultKey::BMP280, i2c0, BMP_280_ADDR, PayloadPersistentDataKey::SeaLevelPressure)
{
}

void PayloadReliableBMP280::update_state(PayloadState& state, const int32_t pressure, const double temperature,
                                         const double altitude) const
{
  state.pressure = pressure;
  state.temperature = temperature;
  state.altitude = altitude - this->get_framework()->get_persistent_data_storage()->get_double(
    PayloadPersistentDataKey::GroundAltitude);
}
