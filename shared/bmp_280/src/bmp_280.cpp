#include "bmp_280.h"

#include <format>
#include <ranges>
#include <cmath>
#include <math.h>
#include <hardware/flash.h>
#include <pico/flash.h>

#include "main.h"
#include "pin_outs.h"
#include "status_manager.h"
#include "i2c_util.h"

BMP280::BMP280(i2c_inst_t* i2c, uint8_t addr) : is_i2c_interface(true), i2c_inst(i2c), i2c_addr(addr)
{
}

BMP280::BMP280(spi_inst_t* spi, uint8_t csn_gpio) : is_i2c_interface(false), spi_inst(spi), csn_pin(csn_gpio)
{
}

const BMP280::CalibrationData& BMP280::get_calibration_data() const
{
  return calibration_data;
}

bool BMP280::check_chip_id()
{
  uint8_t read_id;
  const bool success = i2c_util::read_ubyte(I2C_BUS0, BMP_280_ADDR, BMP280::REG_CHIP_ID, read_id);
  return success && read_id == BMP_280_CHIP_ID;
}

bool BMP280::soft_reset()
{
  constexpr uint8_t data[2] = {REG_SOFT_RESET, BMP_280_RESET_VALUE};
  return i2c_write_blocking_until(I2C_BUS0, BMP_280_ADDR, data, 2, false, delayed_by_ms(get_absolute_time(), 32)) == 2;
}

bool BMP280::check_status(bool& is_measuring, bool& is_updating)
{
  uint8_t read_status;
  const bool success = i2c_util::read_ubyte(I2C_BUS0, BMP_280_ADDR, REG_STATUS, read_status);
  if (!success)
  {
    return false;
  }

  is_measuring = read_status & 0b1000 == 0;
  is_updating = read_status & 0b1 == 0;
  return true;
}

bool BMP280::change_settings(DeviceMode mode, StandbyTimeSetting standby_time, FilterCoefficientSetting filter_setting,
                             OssSettingPressure pressure_oss, OssSettingTemperature temperature_oss)
{
  const uint8_t ctrl_meas = static_cast<uint8_t>(temperature_oss) << 5 | static_cast<uint8_t>(pressure_oss) << 2 |
    static_cast<uint8_t>(mode);
  const uint8_t config_data = static_cast<uint8_t>(standby_time) << 5 | static_cast<uint8_t>(filter_setting) << 2;

  const uint8_t write_data[3] = {REG_CTRL_MEAS, ctrl_meas, config_data};
  const bool success = i2c_write_blocking_until(I2C_BUS0, BMP_280_ADDR, write_data, 3, false,
                                                delayed_by_ms(get_absolute_time(), 50)) == 3;
  return success;
}

bool BMP280::read_calibration_data()
{
  const bool success =
    i2c_util::read_ushort_reversed(I2C_BUS0, BMP_280_ADDR, REG_DIG_T1, calibration_data.dig_T1) &&
    i2c_util::read_short_reversed(I2C_BUS0, BMP_280_ADDR, REG_DIG_T2, calibration_data.dig_T2) &&
    i2c_util::read_short_reversed(I2C_BUS0, BMP_280_ADDR, REG_DIG_T3, calibration_data.dig_T3) &&
    i2c_util::read_ushort_reversed(I2C_BUS0, BMP_280_ADDR, REG_DIG_P1, calibration_data.dig_P1) &&
    i2c_util::read_short_reversed(I2C_BUS0, BMP_280_ADDR, REG_DIG_P2, calibration_data.dig_P2) &&
    i2c_util::read_short_reversed(I2C_BUS0, BMP_280_ADDR, REG_DIG_P3, calibration_data.dig_P3) &&
    i2c_util::read_short_reversed(I2C_BUS0, BMP_280_ADDR, REG_DIG_P4, calibration_data.dig_P4) &&
    i2c_util::read_short_reversed(I2C_BUS0, BMP_280_ADDR, REG_DIG_P5, calibration_data.dig_P5) &&
    i2c_util::read_short_reversed(I2C_BUS0, BMP_280_ADDR, REG_DIG_P6, calibration_data.dig_P6) &&
    i2c_util::read_short_reversed(I2C_BUS0, BMP_280_ADDR, REG_DIG_P7, calibration_data.dig_P7) &&
    i2c_util::read_short_reversed(I2C_BUS0, BMP_280_ADDR, REG_DIG_P8, calibration_data.dig_P8) &&
    i2c_util::read_short_reversed(I2C_BUS0, BMP_280_ADDR, REG_DIG_P9, calibration_data.dig_P9);

  return success;
}

bool BMP280::read_press_temp_alt(int32_t& pressure, double& temperature, double& altitude)
{
  bool measuring, writing;
  bool success = check_status(measuring, writing);
  if (!success)
  {
    return false;
  }

  if (writing)
  {
    do
    {
      sleep_us(50);
      success = check_status(measuring, writing);
      if (!success)
      {
        return false;
      }
    }
    while (writing);
  }

  uint8_t raw_data[6];
  success = i2c_util::read_bytes(I2C_BUS0, BMP_280_ADDR, REG_PRESS_MSB, raw_data, 6);
  if (!success)
  {
    return false;
  }

  // Do not touch or try to simplify... otherwise you'll have weird overflow things
  // ReSharper disable All
  const int32_t pressure_adc = raw_data[0] << 12 | raw_data[1] << 4 | raw_data[2] >> 4;
  const int32_t temperature_adc = raw_data[3] << 12 | raw_data[4] << 4 | raw_data[5] >> 4;
  double var1 = (((double)temperature_adc) / 16384.0 - ((double)calibration_data.dig_T1) / 1024.0) * ((double)
    calibration_data.dig_T2);
  double var2 = ((((double)temperature_adc) / 131072.0 - ((double)calibration_data.dig_T1) / 8192.0) * (((double)
    temperature_adc) / 131072.0 - ((double)calibration_data.dig_T1) / 8192.0)) * ((double)calibration_data.dig_T3);
  const auto t_fine = static_cast<int32_t>(var1 + var2);
  temperature = (var1 + var2) / 5120.0;

  var1 = ((double)t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)calibration_data.dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)calibration_data.dig_P5) * 2.0;
  var2 = (var2 / 4.0) + (((double)calibration_data.dig_P4) * 65536.0);
  var1 = (((double)calibration_data.dig_P3) * var1 * var1 / 524288.0 + ((double)calibration_data.dig_P2) * var1) /
    524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)calibration_data.dig_P1);
  double p = 1048576.0 - (double)pressure_adc;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double)calibration_data.dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double)calibration_data.dig_P8) / 32768.0;
  p = p + (var1 + var2 + ((double)calibration_data.dig_P7)) / 16.0;

  pressure = static_cast<int32_t>(std::round(p));
  altitude = 44330.0 * (1 - std::pow(p / calibration_data.baro_pressure, 1 / 5.255));
  // ReSharper restore All

  return true;
}

bool BMP280::read_byte(uint8_t reg_addr, uint8_t& value)
{
  if (is_i2c_interface)
  {
    return i2c_util::read_ubyte(i2c_inst, i2c_addr, reg_addr, value);
  }
  else
  {
    //TODO spi
  }
}

bool BMP280::read_short(uint8_t reg_addr, int16_t& value)
{
  if (is_i2c_interface)
  {
    return i2c_util::read_short_reversed(i2c_inst, i2c_addr, reg_addr, value);
  }
  else
  {
    //TODO spi
  }
}

bool BMP280::read_ushort(uint8_t reg_addr, uint16_t& value)
{
  if (is_i2c_interface)
  {
    return i2c_util::read_ushort_reversed(i2c_inst, i2c_addr, reg_addr, value);
  }
  else
  {
    //TODO spi
  }
}

bool BMP280::read_bytes(uint8_t reg_addr, uint8_t* data, uint8_t len)
{
  if (is_i2c_interface)
  {
    return i2c_util::read_bytes(i2c_inst, i2c_addr, reg_addr, data, len);
  }
  else
  {
    //TODO spi
  }
}
