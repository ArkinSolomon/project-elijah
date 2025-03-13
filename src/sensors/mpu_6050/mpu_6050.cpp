#include "mpu_6050.h"

#include <format>
#include <hardware/flash.h>
#include <hardware/watchdog.h>
#include <pico/flash.h>

#include "byte_util.h"
#include "lock_nums.h"
#include "main.h"
#include "pin_outs.h"
#include "status_manager.h"
#include "usb_communication.h"
#include "sensors/i2c/i2c_util.h"

void mpu_6050::init()
{
  if (!critical_section_is_initialized(&mpu_6050_cs))
  {
    critical_section_init_with_lock_num(&mpu_6050_cs, CS_LOCK_NUM_MPU_SENS_DATA);
  }

  critical_section_enter_blocking(&mpu_6050_cs);
  configure_default();
  critical_section_exit(&mpu_6050_cs);

  if (!read_calibration_data())
  {
    usb_communication::send_string("Could not read calibration data for MPU 6050");
  }
}

/**
 * Check if the chip is detected.
 *
 * @return True if the device is detected and the device id reads 0x68.
 */
bool mpu_6050::check_chip_id()
{
  uint8_t read_id = 0x55;
  const bool success = i2c_util::read_ubyte(I2C_BUS1, MPU_6050_ADDR, _reg_defs::REG_WHO_AM_I, read_id);
  usb_communication::send_string(std::format("Chip ID: 0x{:02X}", read_id));

  return success && read_id == MPU_6050_DEVICE_ID;
}

bool mpu_6050::configure(const uint8_t dlpf_cfg, const gyro_full_scale_range gyro_range,
                         const accel_full_scale_range accel_range,
                         const bool self_test_en, const bool enable_ints)
{
  const uint8_t config_reg_data = dlpf_cfg & 0x07;
  uint8_t gyro_config_reg_data = static_cast<uint8_t>(gyro_range) << 3 & 0x18;
  uint8_t accel_config_reg_dat = static_cast<uint8_t>(accel_range) << 3 & 0x18;

  if (self_test_en)
  {
    gyro_config_reg_data |= 0xE0;
    accel_config_reg_dat |= 0xE0;
  }

  const uint8_t write_cfg_data[4] = {
    _reg_defs::REG_CONFIG, config_reg_data, gyro_config_reg_data, accel_config_reg_dat
  };

  int bytes_written = i2c_write_blocking_until(I2C_BUS1, MPU_6050_ADDR, write_cfg_data, 4, false,
                                               delayed_by_ms(get_absolute_time(), 32));
  if (bytes_written != 4)
  {
    return false;
  }

  calibration_data.accel_scale = get_accel_scale(accel_range);
  calibration_data.gyro_scale = get_gyro_scale(gyro_range);

  const uint8_t int_en_data[2] = {_reg_defs::REG_INT_ENABLE, static_cast<uint8_t>(enable_ints ? 0x01 : 0x00)};
  bytes_written = i2c_write_blocking_until(I2C_BUS1, MPU_6050_ADDR, int_en_data, 2, false,
                                           delayed_by_ms(get_absolute_time(), 32));
  if (bytes_written != 2)
  {
    return false;
  }

  constexpr uint8_t power_mgmt_data[2] = {_reg_defs::REG_PWR_MGMT_1, 0x08};
  bytes_written = i2c_write_blocking_until(I2C_BUS1, MPU_6050_ADDR, power_mgmt_data, 2, false,
                                           delayed_by_ms(get_absolute_time(), 32));
  return bytes_written == 2;
}

double mpu_6050::get_accel_scale(const accel_full_scale_range accel_range)
{
  switch (accel_range)
  {
  case accel_full_scale_range::RANGE_2g:
    return GRAVITY_CONSTANT / 16384.0;
  case accel_full_scale_range::RANGE_4g:
    return GRAVITY_CONSTANT / 8192.0;
  case accel_full_scale_range::RANGE_8g:
    return GRAVITY_CONSTANT / 4096.0;
  case accel_full_scale_range::RANGE_16g:
    return GRAVITY_CONSTANT / 2048.0;
  }
  return 0;
}

double mpu_6050::get_gyro_scale(const gyro_full_scale_range gyro_range)
{
  switch (gyro_range)
  {
  case gyro_full_scale_range::RANGE_250:
    return 1.0 / 131.0;
  case gyro_full_scale_range::RANGE_500:
    return 1.0 / 65.5;
  case gyro_full_scale_range::RANGE_1000:
    return 1.0 / 32.8;
  case gyro_full_scale_range::RANGE_2000:
    return 1.0 / 16.4;
  }
  return 0;
}

bool mpu_6050::configure_default()
{
  usb_communication::send_string("configuring mpu6050");
  return configure(CONFIG_MPU_6050_DLPF_CFG, gyro_full_scale_range::RANGE_250,
                   accel_full_scale_range::RANGE_2g, false, false);
}

void mpu_6050::write_calibration_data(void*)
{
  uint8_t encoded_calibration_data[256];

  constexpr uint64_t flash_check = MPU_6050_CALIBRATION_CHECK;
  *reinterpret_cast<uint64_t*>(encoded_calibration_data) = flash_check;
  encode_calibration_data(encoded_calibration_data + sizeof(flash_check));

  constexpr uint32_t flash_offset = MPU_6050_CALIB_FLASH_SECTOR_NUM * 4096;
  flash_range_erase(flash_offset, 256);
  flash_range_program(flash_offset, encoded_calibration_data, 256);
}

bool mpu_6050::read_calibration_data()
{
  // ReSharper disable once CppLocalVariableMayBeConst
  bool flash_data_exists = false;
  const int status = flash_safe_execute([](void* data_exists_v_ptr)
  {
    const auto data_exists_ptr = static_cast<bool*>(data_exists_v_ptr);

    constexpr uint32_t flash_offset = MPU_6050_CALIB_FLASH_SECTOR_NUM * 4096;
    const uint8_t* flash_contents = reinterpret_cast<uint8_t*>(XIP_BASE + flash_offset);

    constexpr uint64_t flash_check = MPU_6050_CALIBRATION_CHECK;
    const uint64_t read_flash_check = *reinterpret_cast<const uint64_t*>(flash_contents);
    if (read_flash_check != flash_check)
    {
      *data_exists_ptr = false;
      return;
    }

    calibration_data.diff_xa = byte_util::decode_double(flash_contents + sizeof(flash_check));
    calibration_data.diff_ya = byte_util::decode_double(flash_contents + sizeof(flash_check) + 8);
    calibration_data.diff_za = byte_util::decode_double(flash_contents + sizeof(flash_check) + 16);
    calibration_data.diff_xg = byte_util::decode_double(flash_contents + sizeof(flash_check) + 24);
    calibration_data.diff_yg = byte_util::decode_double(flash_contents + sizeof(flash_check) + 32);
    calibration_data.diff_zg = byte_util::decode_double(flash_contents + sizeof(flash_check) + 40);
    *data_exists_ptr = true;
  }, &flash_data_exists, MPU_6050_FLASH_TIMEOUT_MS);
  return status == PICO_OK && flash_data_exists;
}

void mpu_6050::encode_calibration_data(uint8_t* encoded_data)
{
  byte_util::encode_double(calibration_data.diff_xa, encoded_data);
  byte_util::encode_double(calibration_data.diff_ya, encoded_data + 8);
  byte_util::encode_double(calibration_data.diff_za, encoded_data + 16);

  byte_util::encode_double(calibration_data.diff_xg, encoded_data + 24);
  byte_util::encode_double(calibration_data.diff_yg, encoded_data + 32);
  byte_util::encode_double(calibration_data.diff_zg, encoded_data + 40);
}

void mpu_6050::send_calibration_data()
{
  uint8_t encoded_calibration_data[MPU_6050_CALIBRATION_SIZE];
  encode_calibration_data(encoded_calibration_data);
  send_packet(usb_communication::CALIBRATION_DATA_MPU_6050, encoded_calibration_data);
}

void mpu_6050::calibrate()
{
  critical_section_enter_blocking(&mpu_6050_cs);
  usb_communication::send_string(std::format("Calibrating MPU 6050 with {} cycles", MPU_6050_CALIBRATION_CYCLES));

  bool configure_success = configure(CONFIG_MPU_6050_DLPF_CFG, gyro_full_scale_range::RANGE_250,
                                     accel_full_scale_range::RANGE_2g, false, false);

  if (!configure_success)
  {
    usb_communication::send_string("Failed to configure MPU 6050 for calibration");
    critical_section_exit(&mpu_6050_cs);
    return;
  }

  constexpr double expected_xa = 0, expected_ya = -GRAVITY_CONSTANT, expected_za = 0;
  constexpr double expected_xg = 0, expected_yg = 0, expected_zg = 0;

  double total_xa = 0, total_ya = 0, total_za = 0;
  double total_xg = 0, total_yg = 0, total_zg = 0;
  for (int i = 1; i <= MPU_6050_CALIBRATION_CYCLES; i++)
  {
    watchdog_update();

    double new_xa, new_ya, new_za;
    double new_xg, new_yg, new_zg;
    get_uncompensated_data(new_xa, new_ya, new_za, new_xg, new_yg, new_zg);

    total_xa += new_xa;
    total_ya += new_ya;
    total_za += new_za;

    total_xg += new_xg;
    total_yg += new_yg;
    total_zg += new_zg;

    sleep_ms(MS_BETWEEN_CALIBRATION_CYCLES);
  }

  const double avg_xa = total_xa / MPU_6050_CALIBRATION_CYCLES
               , avg_ya = total_ya / MPU_6050_CALIBRATION_CYCLES
               , avg_za = total_za / MPU_6050_CALIBRATION_CYCLES;

  const double avg_xg = total_xg / MPU_6050_CALIBRATION_CYCLES
               , avg_yg = total_yg / MPU_6050_CALIBRATION_CYCLES
               , avg_zg = total_zg / MPU_6050_CALIBRATION_CYCLES;

  calibration_data.diff_xa = expected_xa - avg_xa;
  calibration_data.diff_ya = expected_ya - avg_ya;
  calibration_data.diff_za = expected_za - avg_za;

  calibration_data.diff_xg = expected_xg - avg_xg;
  calibration_data.diff_yg = expected_yg - avg_yg;
  calibration_data.diff_zg = expected_zg - avg_zg;

  usb_communication::send_string(std::format(
    "MPU 6050 calibrated: xa = {:.03f}, ya = {:.03f}, za = {:.03f}, xg = {:.03f}, yg = {:.03f}, zg = {:.03f}",
    calibration_data.diff_xa, calibration_data.diff_ya,
    calibration_data.diff_za, calibration_data.diff_xg,
    calibration_data.diff_yg, calibration_data.diff_zg
  ));

  configure_success = configure_default();
  critical_section_exit(&mpu_6050_cs);

  if (!configure_success)
  {
    usb_communication::send_string("Failed to re-configure MPU 6050 to default settings after calibration");
    return;
  }

  send_calibration_data();
  const int write_status = flash_safe_execute(write_calibration_data, nullptr, MPU_6050_FLASH_TIMEOUT_MS);
  if (write_status != PICO_OK)
  {
    usb_communication::send_string(std::format("Failed to write calibration data to flash, code {}", write_status));
  }
}

bool mpu_6050::get_raw_data(int16_t& raw_xa, int16_t& raw_ya, int16_t& raw_za, int16_t& raw_xg, int16_t& raw_yg,
                            int16_t& raw_zg)
{
  uint8_t read_data[14];
  const bool success = i2c_util::read_bytes(I2C_BUS1, MPU_6050_ADDR, _reg_defs::REG_ACCEL_XOUT, read_data, 14);
  if (!success)
  {
    return false;
  }

  raw_xa = static_cast<int16_t>(read_data[0] << 8 | read_data[1]);
  raw_ya = static_cast<int16_t>(read_data[2] << 8 | read_data[3]);
  raw_za = static_cast<int16_t>(read_data[4] << 8 | read_data[5]);
  // Two bytes skipped for unused temperature
  raw_xg = static_cast<int16_t>(read_data[8] << 8 | read_data[9]);
  raw_yg = static_cast<int16_t>(read_data[10] << 8 | read_data[11]);
  raw_zg = static_cast<int16_t>(read_data[12] << 8 | read_data[13]);
  return true;
}

bool mpu_6050::get_uncompensated_data(double& uncomp_xa, double& uncomp_ya, double& uncomp_za, double& uncomp_xg,
                                      double& uncomp_yg, double& uncomp_zg)
{
  int16_t raw_xa, raw_ya, raw_za, raw_xg, raw_yg, raw_zg;
  if (!get_raw_data(raw_xa, raw_ya, raw_za, raw_xg, raw_yg, raw_zg))
  {
    return false;
  }

  uncomp_xa = raw_xa * calibration_data.accel_scale;
  uncomp_ya = raw_ya * calibration_data.accel_scale;
  uncomp_za = raw_za * calibration_data.accel_scale;

  uncomp_xg = raw_xg * calibration_data.gyro_scale;
  uncomp_yg = raw_yg * calibration_data.gyro_scale;
  uncomp_zg = raw_zg * calibration_data.gyro_scale;
  return true;
}

void mpu_6050::data_loop(CollectionData& collection_data)
{
  static bool device_detected = false;
  critical_section_enter_blocking(&mpu_6050_cs);

  if (!device_detected)
  {
    if (!check_chip_id())
    {
      usb_communication::send_string("Fault: MPU 6050, device is not detected");
      set_fault(status_manager::DEVICE_MPU_6050, true);
      critical_section_exit(&mpu_6050_cs);
      return;
    }

    device_detected = configure_default();
    if (!device_detected)
    {
      usb_communication::send_string("Fault: MPU 6050, device was detected, but failed to configure");
      set_fault(status_manager::DEVICE_MPU_6050, true);
      critical_section_exit(&mpu_6050_cs);
      return;
    }
  }

  device_detected = get_uncompensated_data(collection_data.accel_x, collection_data.accel_y, collection_data.accel_z,
                                           collection_data.gyro_x, collection_data.gyro_y, collection_data.gyro_z);
  if (!device_detected)
  {
    usb_communication::send_string("Fault: MPU 6050, failed to get values from device");
    set_fault(status_manager::DEVICE_MPU_6050, true);
    critical_section_exit(&mpu_6050_cs);
    return;
  }

  collection_data.accel_x += calibration_data.diff_xa;
  collection_data.accel_y += calibration_data.diff_ya;
  collection_data.accel_z += calibration_data.diff_za;

  collection_data.gyro_x += calibration_data.diff_xg;
  collection_data.gyro_y += calibration_data.diff_yg;
  collection_data.gyro_z += calibration_data.diff_zg;

  read_calibration_data();

  set_fault(status_manager::DEVICE_MPU_6050, false);
  critical_section_exit(&mpu_6050_cs);
}
