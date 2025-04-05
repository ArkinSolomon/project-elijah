#include "override_reliable_bmp_280.h"

#include "override_state_manager.h"
#include "pin_outs.h"

OverrideReliableBMP280::OverrideReliableBMP280(OverrideStateManager* override_state_manager): ReliableBMP280(
override_state_manager, OverrideFaultKey::BMP280, spi0, BMP_CS_PIN, BMP280::FilterCoefficientSetting::Filter16x,
BMP280::OssSettingPressure::PressureOss16, BMP280::OssSettingTemperature::TemperatureOss16,
OverridePersistentStateKey::GroundPressure, OverridePersistentStateKey::GroundTemperature)
{
}

void OverrideReliableBMP280::update_state(OverrideState& state, const int32_t pressure, const double temperature,
                                          const double altitude) const
{
  state.pressure = pressure;
  state.temperature = temperature;
  state.altitude = altitude;
}
