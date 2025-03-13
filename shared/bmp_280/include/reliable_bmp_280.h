#pragma once
#include <hardware/gpio.h>

#include "bmp_280.h"
#include "reliable_component_helper.h"

FRAMEWORK_TEMPLATE_DECL
class ReliableBMP280 : public elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>
{
public:
  ReliableBMP280(
    elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework, const EFaultKey& fault_key,
    i2c_inst_t* i2c, uint8_t addr, EPersistentStorageKey sea_level_pressure_key);

  ReliableBMP280(
    elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework, const EFaultKey& fault_key,
    spi_inst_t* spi, uint8_t csn_gpio, EPersistentStorageKey sea_level_pressure_key);

  [[nodiscard]] BMP280& get_bmp280();

protected:
  std::string on_init(TStateData& state) override;
  std::string on_update(TStateData& state) override;
  virtual void update_state(TStateData& state, int32_t pressure, double temperature, double altitude) const = 0;

private:
  BMP280 bmp;

  EPersistentStorageKey sea_level_pressure_key;
};

FRAMEWORK_TEMPLATE_DECL
ReliableBMP280<FRAMEWORK_TEMPLATE_TYPES>::ReliableBMP280(
  elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework, const EFaultKey& fault_key,
  spi_inst_t* spi, const uint8_t csn_gpio, EPersistentStorageKey sea_level_pressure_key)
  : elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>(framework, fault_key),
    bmp(BMP280(spi, csn_gpio)), sea_level_pressure_key(sea_level_pressure_key)
{
}

FRAMEWORK_TEMPLATE_DECL
BMP280& ReliableBMP280<FRAMEWORK_TEMPLATE_TYPES>::get_bmp280()
{
  return bmp;
}

FRAMEWORK_TEMPLATE_DECL
ReliableBMP280<FRAMEWORK_TEMPLATE_TYPES>::ReliableBMP280(
  elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework, const EFaultKey& fault_key,
  i2c_inst_t* i2c, const uint8_t addr, EPersistentStorageKey sea_level_pressure_key)
  : elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>(framework, fault_key),
    bmp(BMP280(i2c, addr)), sea_level_pressure_key(sea_level_pressure_key)
{
}

FRAMEWORK_TEMPLATE_DECL
std::string ReliableBMP280<FRAMEWORK_TEMPLATE_TYPES>::
on_init(TStateData& state)
{
  if (!bmp.check_chip_id())
  {
    return "Failed to read chip id";
  }

  if (!bmp.read_calibration_data())
  {
    return "Failed to read calibration data";
  }

  if (!bmp.change_settings(BMP280::DeviceMode::NormalMode, BMP280::StandbyTimeSetting::Standby500us,
                           BMP280::FilterCoefficientSetting::Filter2x, BMP280::OssSettingPressure::PressureOss4,
                           BMP280::OssSettingTemperature::TemperatureOss4))
  {
    return "Failed to change settings";
  }

  return "";
}

FRAMEWORK_TEMPLATE_DECL
std::string ReliableBMP280<FRAMEWORK_TEMPLATE_TYPES>::on_update(TStateData& state)
{
  int32_t pressure;
  double temperature, altitude;
  if (!bmp.read_press_temp_alt(pressure, temperature, altitude,
                               this->get_framework()->get_persistent_data_storage()->get_double(
                                 sea_level_pressure_key)))
  {
    return "Failed to read pressure/temperature/altitude";
  }

  update_state(state, pressure, temperature, altitude);
  return "";
}
