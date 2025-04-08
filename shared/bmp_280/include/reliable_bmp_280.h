#pragma once
#include <hardware/gpio.h>

#include "bmp_280.h"
#include "reliable_component_helper.h"

#define RELIABLE_BMP_280_SETTINGS_ARG_STR \
  BMP280::FilterCoefficientSetting filter_coefficient, \
  BMP280::OssSettingPressure oss_press, \
  BMP280::OssSettingTemperature oss_temp

#define RELIABLE_BMP_280_SETTINGS_ARG_INIT_LIST \
  filter_coefficient(filter_coefficient), \
  oss_press(oss_press), \
  oss_temp(oss_temp)

#define MAX_EXPECTED_SPEED_M_PER_S 200

FRAMEWORK_TEMPLATE_DECL
class ReliableBMP280 : public elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>
{
public:
  ReliableBMP280(
    elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework,
    i2c_inst_t* i2c, uint8_t addr, RELIABLE_BMP_280_SETTINGS_ARG_STR);

  ReliableBMP280(
    elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework,
    spi_inst_t* spi, uint8_t csn_gpio, RELIABLE_BMP_280_SETTINGS_ARG_STR);

  [[nodiscard]] BMP280& get_bmp280();

protected:
  std::string on_init(TStateData& state) override;
  std::string on_update(TStateData& state) override;

private:
  static constexpr EPersistentStorageKey ground_pressure_key = EPersistentStorageKey::GroundPressure;
  static constexpr EPersistentStorageKey ground_temp_key = EPersistentStorageKey::GroundTemperature;

  BMP280 bmp;

  absolute_time_t last_collection_time = nil_time;
  int32_t last_ground_pressure = 0;
  double last_ground_temperature = 0;
  double last_alt = 0;

  BMP280::FilterCoefficientSetting filter_coefficient;
  BMP280::OssSettingPressure oss_press;
  BMP280::OssSettingTemperature oss_temp;
};

FRAMEWORK_TEMPLATE_DECL
ReliableBMP280<FRAMEWORK_TEMPLATE_TYPES>::ReliableBMP280(
  elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework,
  i2c_inst_t* i2c, const uint8_t addr, RELIABLE_BMP_280_SETTINGS_ARG_STR) :
  elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>(framework, EFaultKey::BMP280),
  bmp(BMP280(i2c, addr)), RELIABLE_BMP_280_SETTINGS_ARG_INIT_LIST
{
}

FRAMEWORK_TEMPLATE_DECL
ReliableBMP280<FRAMEWORK_TEMPLATE_TYPES>::ReliableBMP280(
  elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework,
  spi_inst_t* spi, const uint8_t csn_gpio, RELIABLE_BMP_280_SETTINGS_ARG_STR) :
  elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>(framework, EFaultKey::BMP280),
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
  uint8_t read_id = 0xBB;
  if (!bmp.check_chip_id(read_id))
  {
    return std::format("Failed to read chip id (read 0x{:02X})", read_id);
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

  if (ground_pressure != last_ground_pressure || ground_temperature != last_ground_temperature)
  {
    last_collection_time = nil_time;
  }

  if (is_nil_time(last_collection_time))
  {
    last_ground_pressure = ground_pressure;
    last_ground_temperature = temperature;
    last_alt = altitude;
    last_collection_time = get_absolute_time();
  }
  else
  {
    const absolute_time_t now = get_absolute_time();
    const int64_t dt_us = absolute_time_diff_us(last_collection_time, now);
    const double dt_s = static_cast<double>(dt_us) / 1e6;
    const double max_alt_diff = MAX_EXPECTED_SPEED_M_PER_S * dt_s;

    double alt_diff = std::abs(last_alt - altitude);
    if (alt_diff > max_alt_diff)
    {
      return std::format(
        "Very large altitude detected! Previous: {:.5e}m, current: {:.5e}m, {:.5e}m difference ({:.5e}m max), {:.3}ms since last collection",
        last_alt, altitude, alt_diff, max_alt_diff, dt_s * 1000);
    }

    last_collection_time = now;
    last_alt = altitude;
  }

  state.pressure = pressure;
  state.temperature = temperature;
  state.altitude = altitude;
  return "";
}
