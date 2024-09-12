#include "hardware/i2c.h"

#include "i2c_config.h"
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
 * Has about a 9-30ms delay. Returns false if read fails. See datasheet for implementation details.
 */
bool bmp_180::read_press_temp(oss_setting oss_setting, const CalibrationData &calib_data, long &temperature, long &pressure)
{
  bool success = i2c_util::write_byte(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_CTRL_MEAS, 0x2E);
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
  long uncompensated_temp = data;

  uint8_t uncomp_press_raw[3];
  success = i2c_util::read_bytes(I2C_BUS, BMP_180_ADDR, __reg_defs::REG_OUT_MSB, uncomp_press_raw, 3);
  if (!success)
  {
    return false;
  }

  uint64_t sleep_time;
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

  long uncomp_press = ((uncomp_press_raw[0] << 16) + (uncomp_press_raw[1] << 8) + uncomp_press_raw[2]) >> (8 - oss_setting);

  long x1 = (uncompensated_temp - calib_data.AC6) * calib_data.AC5 / 0x8000;
  long x2 = calib_data.MC * 0x800 / (x1 + calib_data.MD);
  long b5 = x1 + x2;
  long temperature_dc = (b5 + 8) / 0x10; // in dC°

  long b6 = b5 - 4000;
  x1 = (calib_data.B2 * (b6 * b6 / 0x1000)) / 0x800;
  x2 = calib_data.AC2 * b6 / 0x800;
  long x3 = x1 + x2;
  long b3 = (((calib_data.AC4 * 4 + x3) << oss_setting) + 2) / 4;
  long b4 = calib_data.AC4 * (unsigned long)(x3 + 0x8000) / 0x8000;
  long b7 = ((unsigned long)uncomp_press - b3) * (((int)5e4) >> oss_setting);
  pressure = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (pressure / 0x100) * (pressure / 0x100);
  x1 *= 3038 / 0x10000;
  x2 = (-7357 * pressure) / 0x10000;
  pressure += (x1 + x2 + 3791) / 0x10; // pressure in pA

  // temperature = temperature_dc / 10; // temperature to C°

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