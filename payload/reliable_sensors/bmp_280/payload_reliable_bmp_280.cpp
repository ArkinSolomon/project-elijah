#include "payload_reliable_bmp_280.h"

#include "payload_state_manager.h"

PayloadReliableBMP280::PayloadReliableBMP280(PayloadStateManager* payload_state_manager): ReliableBMP280(
  payload_state_manager, PayloadFaultKey::BMP280, i2c0, BMP_280_ADDR, BMP280::FilterCoefficientSetting::Filter4x,
  BMP280::OssSettingPressure::PressureOss8, BMP280::OssSettingTemperature::TemperatureOss16,
  PayloadPersistentDataKey::GroundPressure, PayloadPersistentDataKey::GroundTemperature)
{
}

void PayloadReliableBMP280::update_state(PayloadState& state, const int32_t pressure, const double temperature,
                                         const double altitude) const
{
  state.pressure = pressure;
  state.temperature = temperature;
  state.altitude = altitude;
}
