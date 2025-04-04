#pragma once
#include <hardware/gpio.h>

#include "bmp_280.h"
#include "reliable_component_helper.h"

#define RELIABLE_BMP_280_SETTINGS_ARG_STR \
  BMP280::FilterCoefficientSetting filter_coefficient, \
  BMP280::OssSettingPressure oss_press, \
  BMP280::OssSettingTemperature oss_temp, \
  EPersistentStorageKey ground_pressure_key, \
  EPersistentStorageKey ground_temp_key

#define RELIABLE_BMP_280_SETTINGS_ARG_INIT_LIST \
  filter_coefficient(filter_coefficient), \
  oss_press(oss_press), \
  oss_temp(oss_temp), \
  ground_pressure_key(ground_pressure_key), \
  ground_temp_key(ground_temp_key)

FRAMEWORK_TEMPLATE_DECL
class ReliableBMP280 : public elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>
{
public:
  ReliableBMP280(
    elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework, const EFaultKey& fault_key,
    i2c_inst_t* i2c, uint8_t addr, RELIABLE_BMP_280_SETTINGS_ARG_STR);

  ReliableBMP280(
    elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework, const EFaultKey& fault_key,
    spi_inst_t* spi, uint8_t csn_gpio, RELIABLE_BMP_280_SETTINGS_ARG_STR);

  [[nodiscard]] BMP280& get_bmp280();

protected:
  std::string on_init(TStateData& state) override;
  std::string on_update(TStateData& state) override;
  virtual void update_state(TStateData& state, int32_t pressure, double temperature, double altitude) const = 0;

private:
  BMP280 bmp;

  BMP280::FilterCoefficientSetting filter_coefficient;
  BMP280::OssSettingPressure oss_press;
  BMP280::OssSettingTemperature oss_temp;
  EPersistentStorageKey ground_pressure_key, ground_temp_key;
};

FRAMEWORK_TEMPLATE_DECL
ReliableBMP280<FRAMEWORK_TEMPLATE_TYPES>::ReliableBMP280(
  elijah_state_framework::ElijahStateFramework<TStateData, EPersistentStorageKey, EFaultKey, EFlightPhase,
                                               TFlightPhaseController>* framework, const EFaultKey& fault_key,
  i2c_inst_t* i2c, const uint8_t addr, RELIABLE_BMP_280_SETTINGS_ARG_STR) :
  elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>(framework, fault_key),
  bmp(BMP280(i2c, addr)), RELIABLE_BMP_280_SETTINGS_ARG_INIT_LIST
{
}

FRAMEWORK_TEMPLATE_DECL
ReliableBMP280<FRAMEWORK_TEMPLATE_TYPES>::ReliableBMP280(
  elijah_state_framework::ElijahStateFramework<TStateData, EPersistentStorageKey, EFaultKey, EFlightPhase,
                                               TFlightPhaseController>* framework, const EFaultKey& fault_key,
  spi_inst_t* spi, const uint8_t csn_gpio, RELIABLE_BMP_280_SETTINGS_ARG_STR) :
  elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>(framework, fault_key),
  bmp(BMP280(spi, csn_gpio)), RELIABLE_BMP_280_SETTINGS_ARG_INIT_LIST
{
}

FRAMEWORK_TEMPLATE_DECL
BMP280& ReliableBMP280<FRAMEWORK_TEMPLATE_TYPES>::get_bmp280()
{
  return bmp;
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
                           filter_coefficient, oss_press, oss_temp))
  {
    return "Failed to change settings";
  }

  return "";
}

FRAMEWORK_TEMPLATE_DECL
std::string ReliableBMP280<FRAMEWORK_TEMPLATE_TYPES>::on_update(TStateData& state)
{
  const int32_t ground_pressure = this->get_framework()->get_persistent_storage()->get_int32(ground_pressure_key);
  const double ground_temperature = this->get_framework()->get_persistent_storage()->get_double(ground_temp_key);
  int32_t pressure;
  double temperature, altitude;
  if (!bmp.read_press_temp_alt(pressure, temperature, altitude, ground_pressure, ground_temperature))
  {
    return "Failed to read pressure/temperature/altitude";
  }
  update_state(state, pressure, temperature, altitude);
  return "";
}
