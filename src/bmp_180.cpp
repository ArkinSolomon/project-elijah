#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/float.h"

#include "pin_outs.h"
#include "i2c_util.h"
#include "bmp_180.h"

/**
 * Returns true if the device is detected and the device id reads 0x55.
 */
bool bmp_180::check_device_id()
{
  uint8_t read_id;
  const bool success = i2c_util::read_ubyte(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_DEVICEID, read_id);

  return success && read_id == BMP_180_DEVICEID;
}

/**
 * Read the calibration data from the device.
 */
bool bmp_180::read_calibration_data(CalibrationData &calib_data)
{
  bool success = i2c_util::read_short(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_CALIB_AC1, calib_data.AC1) &&
                 i2c_util::read_short(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_CALIB_AC2, calib_data.AC2) &&
                 i2c_util::read_short(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_CALIB_AC3, calib_data.AC3) &&
                 i2c_util::read_ushort(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_CALIB_AC4, calib_data.AC4) &&
                 i2c_util::read_ushort(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_CALIB_AC5, calib_data.AC5) &&
                 i2c_util::read_ushort(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_CALIB_AC6, calib_data.AC6) &&
                 i2c_util::read_short(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_CALIB_B1, calib_data.B1) &&
                 i2c_util::read_short(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_CALIB_B2, calib_data.B2) &&
                 i2c_util::read_short(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_CALIB_MB, calib_data.MB) &&
                 i2c_util::read_short(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_CALIB_MC, calib_data.MC) &&
                 i2c_util::read_short(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_CALIB_MD, calib_data.MD);
  return success;
}

/**
 * Read true temperature and true pressure.
 *
 * Has about a 9-30ms delay. Returns false if read fails. See datasheet for
 * implementation details. Temperature is in celcius, pressure is in pascals.
 * Altitude is in meters.
 */
bool bmp_180::read_press_temp_alt(oss_setting oss_setting, const CalibrationData &calib_data, double &temperature, int32_t &pressure, double &altitude)
{
  uint8_t write_data[2] = {__reg_defs::REG_CTRL_MEAS, 0x2E};
  bool success = i2c_write_blocking(I2C_BUS, BMP_180_ADDR, write_data, 2, true);
  if (!success)
  {
    return false;
  }

  sleep_us(4500);

  int16_t data;
  success = i2c_util::read_short(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_OUT_MSB, data);
  if (!success)
  {
    return false;
  }
  int32_t uncompensated_temp = data;

  uint32_t sleep_time;
  switch (oss_setting)
  {
  case bmp_180::oss_setting::ULTRA_LOW:
    sleep_time = 4500;
    break;
  case bmp_180::oss_setting::STANDARD:
    sleep_time = 7500;
    break;
  case bmp_180::oss_setting::HIGH:
    sleep_time = 13500;
    break;
  case bmp_180::oss_setting::ULTRA_HIGH:
    sleep_time = 25500;
    break;
  }
  sleep_us(sleep_time);

  while (!is_conversion_complete())
  {
    sleep_us(500);
  }

  uint8_t uncomp_press_raw[3];
  success = i2c_util::read_bytes(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_OUT_MSB, uncomp_press_raw, 3);
  if (!success)
  {
    return false;
  }

  int32_t uncomp_press = ((uncomp_press_raw[0] << 16) + (uncomp_press_raw[1] << 8) + uncomp_press_raw[2]) >> (8 - oss_setting);

  int32_t x1 = (uncompensated_temp - calib_data.AC6) * calib_data.AC5 / 0x8000;
  int32_t x2 = calib_data.MC * 0x800 / (x1 + calib_data.MD);
  int32_t b5 = x1 + x2;
  int32_t temperature_dc = (b5 + 8) / 0x10;       // in dC°
  temperature = ((double)temperature_dc) / 10; // temperature to C°

  int32_t b6 = b5 - 4000;
  x1 = (calib_data.B2 * (b6 * b6 / 0x1000)) / 0x800;
  x2 = calib_data.AC2 * b6 / 0x800;
  int32_t x3 = x1 + x2;
  int32_t b3 = (((calib_data.AC1 * 4 + x3) << oss_setting) + 2) / 4;
  x1 = calib_data.AC3 * b6 / 0x2000;
  x2 = (calib_data.B1 * (b6 * b6 / 0x1000)) / 0x10000;
  x3 = ((x1 + x2) + 2) / 4;
  uint32_t b4 = calib_data.AC4 * (uint32_t)(x3 + 0x8000) / 0x8000;
  uint32_t b7 = ((uint32_t)uncomp_press - b3) * (((int32_t)5e4) >> oss_setting);
  pressure = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (pressure / 0x100) * (pressure / 0x100);
  x1 = (x1 * 3038) / 0x10000;
  x2 = (-7357 * pressure) / 0x10000;
  pressure = pressure + (x1 + x2 + 3791) / 0x10; // pressure in pA

  altitude = ((double)44330) * (1 - pow(pressure / SEA_LEVEL_PRESS, 1.0 / 5.255));

  return true;
}

/**
 * Soft-reset the device to startup state.
 */
void bmp_180::soft_reset()
{
  uint8_t data[2] = {__reg_defs::REG_SOFT_RESET, BMP_180_RESET_VALUE};
  i2c_write_blocking(I2C_BUS, BMP_180_ADDR, data, 2, false);
}

bool bmp_180::is_conversion_complete()
{
  uint8_t ctrl;
  const bool success = i2c_util::read_ubyte(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_CTRL_MEAS, ctrl);
  if (!success)
  {
    return false;
  }

  return (ctrl & 0x20) == 0;
}

/**
 * Print calibration data out.
 */
void bmp_180::print_calib_data(const CalibrationData &calib_data)
{
  printf("--- Calibration Data ---\n");
  printf("AC1: %d\n", calib_data.AC1);
  printf("AC2: %d\n", calib_data.AC2);
  printf("AC3: %d\n", calib_data.AC3);
  printf("AC4: %d\n", calib_data.AC4);
  printf("AC5: %d\n", calib_data.AC5);
  printf("AC5: %d\n", calib_data.AC6);
  printf("B1: %d\n", calib_data.B1);
  printf("B2: %d\n", calib_data.B2);
  printf("MB: %d\n", calib_data.MB);
  printf("MC: %d\n", calib_data.MC);
  printf("MD: %d\n", calib_data.MD);
  printf("------\n");
}