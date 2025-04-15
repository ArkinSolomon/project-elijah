#include "bmp_280.h"

#include <cmath>
#include <format>
#include <hardware/gpio.h>

#include "elijah_state_framework.h"
#include "i2c_util.h"

// try i2c_inst -> i2c0
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

bool BMP280::uses_i2c() const
{
  return is_i2c_interface;
}

bool BMP280::check_chip_id(uint8_t& read_id) const
{
  const bool success = read_byte(REG_CHIP_ID, read_id);
  return success && read_id == BMP_280_CHIP_ID;
}

bool BMP280::soft_reset() const
{
  constexpr uint8_t reset_value = BMP_280_RESET_VALUE;
  return write_bytes_to_device(REG_SOFT_RESET, &reset_value, 1);
}

bool BMP280::check_status(bool& is_measuring, bool& is_updating) const
{
  uint8_t read_status;
  const bool success = read_byte(REG_STATUS, read_status);
  if (!success)
  {
    return false;
  }

  is_measuring = read_status & 0b1000 == 0;
  is_updating = read_status & 0b1 == 0;
  return true;
}

// ReSharper disable once CppMemberFunctionMayBeConst
bool BMP280::change_settings(DeviceMode mode, StandbyTimeSetting standby_time, FilterCoefficientSetting filter_setting,
                             OssSettingPressure pressure_oss, OssSettingTemperature temperature_oss)
{
  const uint8_t ctrl_meas = static_cast<uint8_t>(temperature_oss) << 5 | static_cast<uint8_t>(pressure_oss) << 2 |
    static_cast<uint8_t>(mode);
  const uint8_t config_data = static_cast<uint8_t>(standby_time) << 5 | static_cast<uint8_t>(filter_setting) << 2;

  uint8_t data_buff[2] = {ctrl_meas, config_data};
  const bool data_write_success = write_bytes_to_device(REG_CTRL_MEAS, data_buff, 2);
  if (!data_write_success)
  {
    return false;
  }

  const bool data_read_success = read_bytes(REG_CTRL_MEAS, data_buff, 2);
  if (!data_read_success)
  {
    return false;
  }

  // elijah_state_framework::log_serial_message(std::format("0x{:02X} == 0x{:02X} && 0x{:02X} == 0x{:02X}", data_buff[0], ctrl_meas, data_buff[1], config_data));
  return data_buff[0] == ctrl_meas && data_buff[1] == config_data;
}

bool BMP280::read_calibration_data()
{
  const bool success =
    read_ushort(REG_DIG_T1, calibration_data.dig_T1) &&
    read_short(REG_DIG_T2, calibration_data.dig_T2) &&
    read_short(REG_DIG_T3, calibration_data.dig_T3) &&
    read_ushort(REG_DIG_P1, calibration_data.dig_P1) &&
    read_short(REG_DIG_P2, calibration_data.dig_P2) &&
    read_short(REG_DIG_P3, calibration_data.dig_P3) &&
    read_short(REG_DIG_P4, calibration_data.dig_P4) &&
    read_short(REG_DIG_P5, calibration_data.dig_P5) &&
    read_short(REG_DIG_P6, calibration_data.dig_P6) &&
    read_short(REG_DIG_P7, calibration_data.dig_P7) &&
    read_short(REG_DIG_P8, calibration_data.dig_P8) &&
    read_short(REG_DIG_P9, calibration_data.dig_P9);

  return success;
}

bool BMP280::read_press_temp(int32_t& pressure, double& temperature) const
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
  success = read_bytes(REG_PRESS_MSB, raw_data, 6);
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
  // ReSharper restore All

  pressure = static_cast<int32_t>(std::round(p));
  return true;
}

bool BMP280::read_press_temp_alt(int32_t& pressure, double& temperature, double& altitude,
                                 const int32_t ground_pressure, double ground_temperature) const
{
  if (!read_press_temp(pressure, temperature))
  {
    return false;
  }

  altitude = (15 + 273.15) / 0.0065 * (1 - std::pow(static_cast<double>(pressure) / 101325.0,
                                                    1 / 5.255)) - (15 + 273.15) /
    0.0065 * (1 - std::pow(static_cast<double>(ground_pressure) / (101325.0), 1 / 5.255));
  return true;
}

bool BMP280::read_byte(const uint8_t reg_addr, uint8_t& value) const
{
  if (is_i2c_interface)
  {
    return i2c_util::read_ubyte(i2c_inst, i2c_addr, reg_addr, value);
  }

  // SPI
  read_spi_bytes(reg_addr, &value, 1);
  return true;
}

bool BMP280::read_short(const uint8_t reg_addr, int16_t& value) const
{
  if (is_i2c_interface)
  {
    return i2c_util::read_short_little_endian(i2c_inst, i2c_addr, reg_addr, value);
  }

  // SPI
  uint8_t read_data[2];
  read_spi_bytes(reg_addr, read_data, 2);
  value = *reinterpret_cast<int16_t*>(read_data);
  return true;
}

bool BMP280::read_ushort(const uint8_t reg_addr, uint16_t& value) const
{
  if (is_i2c_interface)
  {
    return i2c_util::read_ushort_little_endian(i2c_inst, i2c_addr, reg_addr, value);
  }

  // SPI
  uint8_t read_data[2];
  read_spi_bytes(reg_addr, read_data, 2);
  value = *reinterpret_cast<uint16_t*>(read_data);
  return true;
}

bool BMP280::read_bytes(const uint8_t reg_addr, uint8_t* data, const size_t len) const
{
  if (is_i2c_interface)
  {
    // elijah_state_framework::log_serial_message(std::format("addr: 0x{:02X} , reg: 0x{:02X}, {}", i2c_addr, reg_addr, i2c_inst == i2c0));
    return i2c_util::read_bytes(i2c_inst, i2c_addr, reg_addr, data, len);
  }

  // SPI
  read_spi_bytes(reg_addr, data, len);
  return true;
}

bool BMP280::write_bytes_to_device(uint8_t start_reg_addr, const uint8_t* data, const size_t len) const
{
  if (is_i2c_interface)
  {
    i2c_write_blocking_until(i2c_inst, i2c_addr, &start_reg_addr, 1, true,
                             delayed_by_ms(get_absolute_time(), 5));
    const size_t bytes_written = i2c_write_blocking_until(i2c_inst, i2c_addr, data, len, false,
                                                          delayed_by_ms(get_absolute_time(), 5 * len));
    return bytes_written == len;
  }

  // SPI
  const size_t write_len = len * 2;
  uint8_t write_data[write_len];

  for (size_t idx = 0, i = 0; i < len; idx += 2, i++)
  {
    write_data[idx] = static_cast<uint8_t>(start_reg_addr + static_cast<uint8_t>(i));
    write_data[idx] &= 0x7F;
    write_data[idx + 1] = data[i];
  }

  gpio_put(csn_pin, false);
  busy_wait_us(1);

  spi_write_blocking(spi_inst, write_data, write_len);

  gpio_put(csn_pin, true);
  return true;
}

void BMP280::read_spi_bytes(const uint8_t start_reg_addr, uint8_t* data, const size_t len) const
{
  assert(!is_i2c_interface);

  gpio_put(csn_pin, false);
  busy_wait_us(1);

  const auto read_addr = static_cast<uint8_t>(start_reg_addr | 0x80);
  spi_write_blocking(spi_inst, &read_addr, 1);
  spi_read_blocking(spi_inst, 0x00, data, len);

  gpio_put(csn_pin, true);
}
