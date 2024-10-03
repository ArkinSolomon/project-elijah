#include "bmp_280.h"

#include <format>
#include <ranges>
#include <cmath>

#include "src/main.h"
#include "src/pin_outs.h"
#include "src/status_manager.h"
#include "src/usb_communication.h"
#include "src/sensors/i2c/i2c_util.h"

/**
 * Check if the chip is detected.
 *
 * @return True if the device is detected and the device id reads 0x58.
 */
bool bmp_280::check_chip_id()
{
  uint8_t read_id;
  const bool success = i2c_util::read_ubyte(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_CHIP_ID, read_id);
  usb_communication::send_string(std::format("Chip ID: 0b{:08b}", read_id));
  return success && read_id == BMP_280_CHIP_ID;
}

/**
 * Soft-reset the device to startup state.
 *
 * @return True if successful.
 */
bool bmp_280::soft_reset()
{
  constexpr uint8_t data[2] = {_reg_defs::REG_SOFT_RESET, BMP_280_RESET_VALUE};
  return i2c_write_blocking_until(I2C_BUS, BMP_280_ADDR, data, 2, false, delayed_by_ms(get_absolute_time(), 100)) == 2;
}

/**
 * Check the device's current status.
 * 
 * @param measuring Set to true if the device is currently measuring.
 * @param updating Set to true if the device is currently updating.
 * @return True if the status was retrieved successfully.
 */
bool bmp_280::check_status(bool& measuring, bool& updating)
{
  uint8_t read_status;
  const bool success = i2c_util::read_ubyte(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_STATUS, read_status);
  if (!success)
  {
    return false;
  }

  measuring = read_status & 0b1000 == 0;
  updating = read_status & 0b1 == 0;
  return true;
}

bool bmp_280::change_settings(const device_mode mode, const standby_time_setting standby_time,
                              const filter_coefficient_setting filter_setting,
                              const oss_setting_pressure pressure_oss, const oss_setting_temperature temperature_oss)
{
  const uint8_t ctrl_meas = temperature_oss << 5 | pressure_oss << 2 | mode;
  const uint8_t config_data = standby_time << 5 | filter_setting << 2;

  const uint8_t write_data[3] = {_reg_defs::REG_CTRL_MEAS, ctrl_meas, config_data};
  const bool success = i2c_write_blocking_until(I2C_BUS, BMP_280_ADDR, write_data, 3, false,
                                                delayed_by_ms(get_absolute_time(), 50)) == 3;
  return success;
}

/**
 * Read device calibration data.
 *
 * @return True if successful.
 */
bool bmp_280::read_calibration_data()
{
  const bool success = i2c_util::read_ushort_reversed(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_DIG_T1,
                                                      bmp_280_calib_data.dig_T1) &&
    i2c_util::read_short_reversed(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_DIG_T2, bmp_280_calib_data.dig_T2) &&
    i2c_util::read_short_reversed(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_DIG_T3, bmp_280_calib_data.dig_T3) &&
    i2c_util::read_ushort_reversed(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_DIG_P1, bmp_280_calib_data.dig_P1) &&
    i2c_util::read_short_reversed(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_DIG_P2, bmp_280_calib_data.dig_P2) &&
    i2c_util::read_short_reversed(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_DIG_P3, bmp_280_calib_data.dig_P3) &&
    i2c_util::read_short_reversed(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_DIG_P4, bmp_280_calib_data.dig_P4) &&
    i2c_util::read_short_reversed(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_DIG_P5, bmp_280_calib_data.dig_P5) &&
    i2c_util::read_short_reversed(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_DIG_P6, bmp_280_calib_data.dig_P6) &&
    i2c_util::read_short_reversed(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_DIG_P7, bmp_280_calib_data.dig_P7) &&
    i2c_util::read_short_reversed(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_DIG_P8, bmp_280_calib_data.dig_P8) &&
    i2c_util::read_short_reversed(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_DIG_P9, bmp_280_calib_data.dig_P9);

  // bmp_280_calib_data.dig_T1 = 27504;
  // bmp_280_calib_data.dig_T2 = 26435;
  // bmp_280_calib_data.dig_T3 = -1000;
  // bmp_280_calib_data.dig_P1 = 36477;
  // bmp_280_calib_data.dig_P2 = -10685;
  // bmp_280_calib_data.dig_P3 = 3024;
  // bmp_280_calib_data.dig_P4 = 2855;
  // bmp_280_calib_data.dig_P5 = 140;
  // bmp_280_calib_data.dig_P6 = -7;
  // bmp_280_calib_data.dig_P7 = 15500;
  // bmp_280_calib_data.dig_P8 = -14600;
  // bmp_280_calib_data.dig_P9 = 6000;
  return success;
}

/**
 * Send calibration data over USB.
 */
void bmp_280::send_calibration_data()
{
  uint8_t packet_data[26];

  packet_data[0] = (bmp_280_calib_data.dig_T2 < 0) << 7 |
    (bmp_280_calib_data.dig_T3 < 0) << 6 |
    (bmp_280_calib_data.dig_P2 < 0) << 5 |
    (bmp_280_calib_data.dig_P3 < 0) << 4 |
    (bmp_280_calib_data.dig_P4 < 0) << 3 |
    (bmp_280_calib_data.dig_P5 < 0) << 2 |
    (bmp_280_calib_data.dig_P6 < 0) << 1 |
    bmp_280_calib_data.dig_P7 < 0;

  packet_data[1] = (bmp_280_calib_data.dig_P8 < 0) << 1 | bmp_280_calib_data.dig_P9 < 0;

  packet_data[2] = (bmp_280_calib_data.dig_T1 & 0xFF00) >> 8;
  packet_data[3] = bmp_280_calib_data.dig_T1 & 0xFF;
  packet_data[4] = (abs(bmp_280_calib_data.dig_T2) & 0xFF00) >> 8;
  packet_data[5] = abs(bmp_280_calib_data.dig_T2) & 0xFF;
  packet_data[6] = (abs(bmp_280_calib_data.dig_T3) & 0xFF00) >> 8;
  packet_data[7] = abs(bmp_280_calib_data.dig_T3) & 0xFF;
  packet_data[8] = (bmp_280_calib_data.dig_P1 & 0xFF00) >> 8;
  packet_data[9] = bmp_280_calib_data.dig_P1 & 0xFF;
  packet_data[10] = (abs(bmp_280_calib_data.dig_P2) & 0xFF00) >> 8;
  packet_data[11] = abs(bmp_280_calib_data.dig_P2) & 0xFF;
  packet_data[12] = (abs(bmp_280_calib_data.dig_P3) & 0xFF00) >> 8;
  packet_data[13] = abs(bmp_280_calib_data.dig_P3) & 0xFF;
  packet_data[14] = (abs(bmp_280_calib_data.dig_P4) & 0xFF00) >> 8;
  packet_data[15] = abs(bmp_280_calib_data.dig_P4) & 0xFF;
  packet_data[16] = (abs(bmp_280_calib_data.dig_P5) & 0xFF00) >> 8;
  packet_data[17] = abs(bmp_280_calib_data.dig_P5) & 0xFF;
  packet_data[18] = (abs(bmp_280_calib_data.dig_P6) & 0xFF00) >> 8;
  packet_data[19] = abs(bmp_280_calib_data.dig_P6) & 0xFF;
  packet_data[20] = (abs(bmp_280_calib_data.dig_P7) & 0xFF00) >> 8;
  packet_data[21] = abs(bmp_280_calib_data.dig_P7) & 0xFF;
  packet_data[22] = (abs(bmp_280_calib_data.dig_P8) & 0xFF00) >> 8;
  packet_data[23] = abs(bmp_280_calib_data.dig_P8) & 0xFF;
  packet_data[24] = (abs(bmp_280_calib_data.dig_P9) & 0xFF00) >> 8;
  packet_data[25] = abs(bmp_280_calib_data.dig_P9) & 0xFF;

  send_packet(usb_communication::CALIBRATION_DATA_BMP_280, packet_data);
}

/**
 * Retrieve and calculate pressure, temperature, and altitude.
 *
 * @return True if the device successfully reads and/or calculates pressure, temperature, and altitude.
 */
bool bmp_280::read_press_temp_alt(int32_t& pressure, double& temperature, double& altitude)
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
  success = i2c_util::read_bytes(I2C_BUS, BMP_280_ADDR, _reg_defs::REG_PRESS_MSB, raw_data, 6);
  if (!success)
  {
    return false;
  }

  // Do not touch or try to simplify... otherwise you'll have weird overflow things
  // ReSharper disable All
  const int32_t pressure_adc = raw_data[0] << 12 | raw_data[1] << 4 | raw_data[2] >> 4;
  const int32_t temperature_adc = raw_data[3] << 12 | raw_data[4] << 4 | raw_data[5] >> 4;
  double var1 = (((double)temperature_adc) / 16384.0 - ((double)bmp_280_calib_data.dig_T1) / 1024.0) * ((double)
    bmp_280_calib_data.dig_T2);
  double var2 = ((((double)temperature_adc) / 131072.0 - ((double)bmp_280_calib_data.dig_T1) / 8192.0) * (((double)
    temperature_adc) / 131072.0 - ((double)bmp_280_calib_data.dig_T1) / 8192.0)) * ((double)bmp_280_calib_data.dig_T3);
  const auto t_fine = static_cast<int32_t>(var1 + var2);
  temperature = (var1 + var2) / 5120.0;

  var1 = ((double) t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)bmp_280_calib_data.dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)bmp_280_calib_data.dig_P5) * 2.0;
  var2 = (var2 / 4.0) + (((double)bmp_280_calib_data.dig_P4) * 65536.0);
  var1 = (((double)bmp_280_calib_data.dig_P3) * var1 * var1 / 524288.0 + ((double)bmp_280_calib_data.dig_P2) * var1) /
    524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)bmp_280_calib_data.dig_P1);
  double p = 1048576.0 - (double)pressure_adc;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double)bmp_280_calib_data.dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double)bmp_280_calib_data.dig_P8) / 32768.0;
  p = p + (var1 + var2 + ((double)bmp_280_calib_data.dig_P7)) / 16.0;

  pressure = static_cast<int32_t>(std::round(p));
  altitude = 44330.0 * (1 - std::pow(p / SEA_LEVEL_PRESS, 1 / 5.255));
  // ReSharper restore All

  return true;
}

/**
 * Loop function for the BMP 280.
 *
 * @param collection_data Where to put the data.
 */
void bmp_280::data_collection_loop(CollectionData& collection_data)
{
  static bool calibration_data_received = false;

  if (!calibration_data_received)
  {
    const bool device_detected = check_chip_id();
    if (!device_detected || !read_calibration_data())
    {
      usb_communication::send_string("Fault: BMP 280, did not detect and/or can not receive calibration data.");
      set_fault(status_manager::fault_id::DEVICE_BMP_280, true);
      collection_data.pressure = -1;
      collection_data.temperature = collection_data.altitude = -1;
      calibration_data_received = false;
      return;
    }

    if (!soft_reset() || !change_settings(NORMAL_MODE, STANDBY_500us, FILTER_X4, PRESSURE_OSS8, TEMPERATURE_OSS16))
    {
      usb_communication::send_string("Fault: BMP 280, was not able to soft reset and/or change settings");
      set_fault(status_manager::fault_id::DEVICE_BMP_280, true);
    }

    calibration_data_received = true;
  }

  const bool success = read_press_temp_alt(collection_data.pressure, collection_data.temperature,
                                           collection_data.altitude);
  if (!success)
  {
    usb_communication::send_string("Fault: BMP 280, failed to read information");
    set_fault(status_manager::fault_id::DEVICE_BMP_280, true);
    calibration_data_received = false;
    collection_data.pressure = -1;
    collection_data.temperature = collection_data.altitude = -1;
    return;
  }

  set_fault(status_manager::fault_id::DEVICE_BMP_280, false);
}
