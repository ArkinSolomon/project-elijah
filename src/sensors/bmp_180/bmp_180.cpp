#include "hardware/i2c.h"
#include "pico/float.h"
#include <cstdio>

#include "../../pin_outs.h"
#include "../i2c/i2c_util.h"
#include "bmp_180.h"

#include <format>

#include "src/usb_communication.h"

/**
 * Returns true if the device is detected and the device id reads 0x55.
 */
bool bmp_180::check_device_id()
{
  uint8_t read_id;
  const bool success = i2c_util::read_ubyte(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_DEVICE_ID, read_id);
  return success && read_id == BMP_180_DEVICE_ID;
}

/**
 * Read the calibration data from the device.
 */
bool bmp_180::read_calibration_data()
{
  const bool success = i2c_util::read_short(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_CALIB_AC1, bmp_180_calib_data.AC1) &&
    i2c_util::read_short(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_CALIB_AC2, bmp_180_calib_data.AC2) &&
    i2c_util::read_short(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_CALIB_AC3, bmp_180_calib_data.AC3) &&
    i2c_util::read_ushort(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_CALIB_AC4, bmp_180_calib_data.AC4) &&
    i2c_util::read_ushort(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_CALIB_AC5, bmp_180_calib_data.AC5) &&
    i2c_util::read_ushort(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_CALIB_AC6, bmp_180_calib_data.AC6) &&
    i2c_util::read_short(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_CALIB_B1, bmp_180_calib_data.B1) &&
    i2c_util::read_short(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_CALIB_B2, bmp_180_calib_data.B2) &&
    i2c_util::read_short(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_CALIB_MB, bmp_180_calib_data.MB) &&
    i2c_util::read_short(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_CALIB_MC, bmp_180_calib_data.MC) &&
    i2c_util::read_short(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_CALIB_MD, bmp_180_calib_data.MD);
  return success;
}

/**
 * Read true temperature and true pressure.
 *
 * Has about a 9-30ms delay. Returns false if read fails. See datasheet for
 * implementation details. Temperature is in Celsius, pressure is in pascals.
 * Altitude is in meters.
 */
bool bmp_180::read_press_temp_alt(const oss_setting oss_setting, double& temperature,
                                  int32_t& pressure, double& altitude)
{
  constexpr uint8_t write_data[2] = {_reg_defs::REG_CTRL_MEAS, 0x2E};
  bool success = i2c_write_blocking_until(I2C_BUS, BMP_180_ADDR, write_data, 2, false,
                                          delayed_by_ms(get_absolute_time(), 100));
  if (!success)
  {
    return false;
  }

  sleep_us(4500);

  int16_t data;
  success = i2c_util::read_short(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_OUT_MSB, data);
  if (!success)
  {
    return false;
  }
  const int32_t uncompensated_temp = data;

  uint32_t sleep_time = 1000;
  switch (oss_setting)
  {
  case ULTRA_LOW:
    sleep_time = 4500;
    break;
  case STANDARD:
    sleep_time = 7500;
    break;
  case HIGH:
    sleep_time = 13500;
    break;
  case ULTRA_HIGH:
    sleep_time = 25500;
    break;
  }
  sleep_us(sleep_time);

  bool is_complete;
  do
  {
    success = is_conversion_complete(is_complete);

    if (!success)
    {
      return false;
    }

    if (!is_complete)
    {
      sleep_us(100);
    }
  }
  while (!is_complete);

  uint8_t uncomp_press_raw[3];
  success = i2c_util::read_bytes(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_OUT_MSB, uncomp_press_raw, 3);
  if (!success)
  {
    return false;
  }

  const int32_t uncomp_press = ((uncomp_press_raw[0] << 16) + (uncomp_press_raw[1] << 8) + uncomp_press_raw[2]) >> (8 -
    oss_setting);

  int32_t x1 = (uncompensated_temp - bmp_180_calib_data.AC6) * bmp_180_calib_data.AC5 / 0x8000;
  int32_t x2 = bmp_180_calib_data.MC * 0x800 / (x1 + bmp_180_calib_data.MD);
  const int32_t b5 = x1 + x2;
  const int32_t temperature_dc = (b5 + 8) / 0x10; // in dC°
  temperature = static_cast<double>(temperature_dc) / 10; // temperature to C°

  // Because we operate so close to integer limits, we must be very careful not to go over the limit.
  // ReSharper disable CppRedundantParentheses
  const int32_t b6 = b5 - 4000;
  x1 = (bmp_180_calib_data.B2 * (b6 * b6 / 0x1000)) / 0x800;
  x2 = bmp_180_calib_data.AC2 * b6 / 0x800;
  int32_t x3 = x1 + x2;
  const int32_t b3 = (((bmp_180_calib_data.AC1 * 4 + x3) << oss_setting) + 2) / 4;
  x1 = bmp_180_calib_data.AC3 * b6 / 0x2000;
  x2 = (bmp_180_calib_data.B1 * (b6 * b6 / 0x1000)) / 0x10000;
  x3 = ((x1 + x2) + 2) / 4;
  const uint32_t b4 = bmp_180_calib_data.AC4 * static_cast<uint32_t>(x3 + 0x8000) / 0x8000;
  const uint32_t b7 = (static_cast<uint32_t>(uncomp_press) - b3) * (static_cast<int32_t>(5e4) >> oss_setting);
  pressure = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2; // NOLINT(*-narrowing-conversions)
  x1 = (pressure / 0x100) * (pressure / 0x100);
  x1 = (x1 * 3038) / 0x10000;
  x2 = (-7357 * pressure) / 0x10000;
  pressure = pressure + (x1 + x2 + 3791) / 0x10; // pressure in pA
  // ReSharper restore CppRedundantParentheses

  altitude = static_cast<double>(44330) * (1 - pow(pressure / SEA_LEVEL_PRESS, 1.0 / 5.255));

  return true;
}

/**
 * Soft-reset the device to startup state.
 *
 * Returns true if successful.
 */
bool bmp_180::soft_reset()
{
  const uint8_t data[2] = {_reg_defs::REG_SOFT_RESET, BMP_180_RESET_VALUE};
  return i2c_write_blocking_until(I2C_BUS, BMP_180_ADDR, data, 2, false, delayed_by_ms(get_absolute_time(), 100)) == 2;
}

/**
 * Check if conversion is complete. Note that the return value IS NOT indicative of a complete conversion.
 *
 * @param is_complete The output to determine if conversion is complete.
 * @return True if the device was read from successfully.
 */
bool bmp_180::is_conversion_complete(bool& is_complete)
{
  uint8_t ctrl;
  const bool success = i2c_util::read_ubyte(I2C_BUS, BMP_180_ADDR, _reg_defs::REG_CTRL_MEAS, ctrl);
  if (!success)
  {
    is_complete = false;
    return false;
  }

  is_complete = (ctrl & 0x20) == 0;
  return true;
}

/**
 * Send calibration data.
 */
void bmp_180::send_calib_data()
{
  uint8_t packet_data[23];

  packet_data[0] = bmp_180_calib_data.AC1 >= 0 << 7 |
    bmp_180_calib_data.AC2 >= 0 << 6 |
    bmp_180_calib_data.AC3 >= 0 << 5 |
    bmp_180_calib_data.B1 >= 0 << 4 |
    bmp_180_calib_data.B2 >= 0 << 3 |
    bmp_180_calib_data.MB >= 0 << 2 |
    bmp_180_calib_data.MC >= 0 << 1 |
    bmp_180_calib_data.MD >= 0;

  const uint16_t AC1 = abs(bmp_180_calib_data.AC1);
  const uint16_t AC2 = abs(bmp_180_calib_data.AC2);
  const uint16_t AC3 = abs(bmp_180_calib_data.AC3);
  const uint16_t B1 = abs(bmp_180_calib_data.B1);
  const uint16_t B2 = abs(bmp_180_calib_data.B2);
  const uint16_t MB = abs(bmp_180_calib_data.MB);
  const uint16_t MC = abs(bmp_180_calib_data.MC);
  const uint16_t MD = abs(bmp_180_calib_data.MD);

  packet_data[1] = AC1 >> 8;
  packet_data[2] = AC1;
  packet_data[3] = AC2 >> 8;
  packet_data[4] = AC2;
  packet_data[5] = AC3 >> 8;
  packet_data[6] = AC3;
  packet_data[7] = bmp_180_calib_data.AC4 >> 8;
  packet_data[8] = bmp_180_calib_data.AC4;
  packet_data[9] = bmp_180_calib_data.AC5 >> 8;
  packet_data[10] = bmp_180_calib_data.AC5;
  packet_data[11] = bmp_180_calib_data.AC6 >> 8;
  packet_data[12] = bmp_180_calib_data.AC6;
  packet_data[13] = B1 >> 8;
  packet_data[14] = B1;
  packet_data[15] = B2 >> 8;
  packet_data[16] = B2;
  packet_data[17] = MB >> 8;
  packet_data[18] = MB;
  packet_data[19] = MC >> 8;
  packet_data[20] = MC;
  packet_data[21] = MD >> 8;
  packet_data[22] = MD;

  send_packet(usb_communication::CALIBRATION_DATA, packet_data);
}
