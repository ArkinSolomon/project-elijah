#include "mpu_6050.h"

#include <cmath>
#include <format>

#include "pico/rand.h"
#include "src/main.h"
#include "src/pin_outs.h"
#include "src/status_manager.h"
#include "src/usb_communication.h"
#include "src/sensors/i2c/i2c_util.h"

/**
 * Check if the chip is detected.
 *
 * @return True if the device is detected and the device id reads 0x68.
 */
bool mpu_6050::check_chip_id()
{
  uint8_t read_id;
  const bool success = i2c_util::read_ubyte(I2C_BUS1, MPU_6050_ADDR, _reg_defs::REG_WHO_AM_I, read_id);
  return success && read_id == MPU_6050_DEVICE_ID;
}

bool mpu_6050::configure(const uint8_t dlpf_cfg, const gyro_full_scale_range gyro_range,
                         const accel_full_scale_range accel_range, const bool
                         int_enable)
{
  const uint8_t config_reg_data = dlpf_cfg & 0x07;
  const uint8_t gyro_config_reg_data = static_cast<uint8_t>(gyro_range) << 3 & 0x18;
  const uint8_t accel_config_reg_dat = static_cast<uint8_t>(accel_range) << 2 & 0x18;
  const uint8_t write_cfg_data[4] = {
    _reg_defs::REG_CONFIG, config_reg_data, gyro_config_reg_data, accel_config_reg_dat
  };

  int bytes_written = i2c_write_blocking_until(I2C_BUS1, MPU_6050_ADDR, write_cfg_data, 4, false,
                                               delayed_by_ms(get_absolute_time(), 100));
  if (bytes_written != 4)
  {
    return false;
  }

  accel_scale = get_accel_scale(accel_range);

  const uint8_t int_en_data[2] = {_reg_defs::REG_INT_ENABLE, static_cast<uint8_t>(int_enable ? 0x01 : 0x00)};
  bytes_written = i2c_write_blocking_until(I2C_BUS1, MPU_6050_ADDR, int_en_data, 2, false,
                                           delayed_by_ms(get_absolute_time(), 100));
  if (bytes_written != 2)
  {
    return false;
  }

  constexpr uint8_t sleep_disable_data[3] = {_reg_defs::REG_PWR_MGMT_1, 0x18, 0xC7};
  bytes_written = i2c_write_blocking_until(I2C_BUS1, MPU_6050_ADDR, sleep_disable_data, 3, false,
                                           delayed_by_ms(get_absolute_time(), 100));
  return bytes_written == 3;
}

bool mpu_6050::configure_default()
{
  return configure(CONFIG_MPU_6050_DLPF_CFG, gyro_full_scale_range::RANGE_250,
                   accel_full_scale_range::RANGE_2g, true);
}

double mpu_6050::get_accel_scale(const accel_full_scale_range accel_range)
{
  switch (accel_range)
  {
  case accel_full_scale_range::RANGE_2g:
    return 0.000061 * GRAVITY_CONSTANT;
  case accel_full_scale_range::RANGE_4g:
    return 0.000122 * GRAVITY_CONSTANT;
  case accel_full_scale_range::RANGE_8g:
    return 0.000244 * GRAVITY_CONSTANT;
  case accel_full_scale_range::RANGE_16g:
    return 0.0004882 * GRAVITY_CONSTANT;
  }
  return 0;
}

bool mpu_6050::self_test()
{
  uint8_t write_data[5];
  write_data[0] = _reg_defs::REG_SELF_TEST_X;

  // Write a bunch of random values to self test registers x, y, z, and a at once
  const auto rand_loc = reinterpret_cast<uint32_t*>(&write_data[1]);
  *rand_loc = get_rand_32();
  write_data[4] &= 0x3F;

  bool success = configure(CONFIG_MPU_6050_DLPF_CFG, gyro_full_scale_range::RANGE_250,
                           accel_full_scale_range::RANGE_8g, true);
  if (!success)
  {
    return false;
  }

  const uint8_t xa_test = write_data[1] >> 4 | write_data[4] & 0x30
                , ya_test = write_data[2] >> 4 | write_data[4] & 0x0C
                , za_test = write_data[3] >> 4 | write_data[4] & 0x03;

  double st_d_accel_x, st_d_accel_y, st_d_accel_z;
  success = get_data(st_d_accel_x, st_d_accel_y, st_d_accel_z);
  if (!success)
  {
    return false;
  }

  int bytes_written = i2c_write_blocking_until(I2C_BUS1, MPU_6050_ADDR, write_data, 5, false,
                                               delayed_by_ms(get_absolute_time(), 100));
  if (bytes_written != 5)
  {
    return false;
  }

  double st_e_accel_x, st_e_accel_y, st_e_accel_z;
  success = get_data(st_e_accel_x, st_e_accel_y, st_e_accel_z);
  if (!success)
  {
    return false;
  }

  const double str_accel_x = st_e_accel_x - st_d_accel_x
               , str_accel_y = st_e_accel_y - st_d_accel_y
               , str_accel_z = st_e_accel_z - st_d_accel_z;

  *rand_loc = 0;
  bytes_written = i2c_write_blocking_until(I2C_BUS1, MPU_6050_ADDR, write_data, 5, false,
                                           delayed_by_ms(get_absolute_time(), 100));
  if (bytes_written != 5)
  {
    return false;
  }

  mpu_6050_factory_trim_data.ft_xa = factory_trim_accel(xa_test);
  mpu_6050_factory_trim_data.ft_ya = factory_trim_accel(ya_test);
  mpu_6050_factory_trim_data.ft_za = factory_trim_accel(za_test);

  mpu_6050_factory_trim_data.ft_xa_change = calculate_self_test_change(str_accel_x, mpu_6050_factory_trim_data.ft_xa);
  mpu_6050_factory_trim_data.ft_ya_change = calculate_self_test_change(str_accel_y, mpu_6050_factory_trim_data.ft_ya);
  mpu_6050_factory_trim_data.ft_za_change = calculate_self_test_change(str_accel_z, mpu_6050_factory_trim_data.ft_za);

  return configure_default();
}

double mpu_6050::factory_trim_accel(const double test_value)
{
  if (test_value == 0)
  {
    return 0;
  }

  return 4096 * 0.34 * std::pow(0.92 / 0.34, (test_value - 1) / 30);
}

double mpu_6050::calculate_self_test_change(const double str, const double ft)
{
  return (str - ft) / ft;
}

bool mpu_6050::get_data(double& accel_x, double& accel_y, double& accel_z)
{
  uint8_t read_data[6];
  const bool success = i2c_util::read_bytes(I2C_BUS1, MPU_6050_ADDR, _reg_defs::REG_ACCEL_XOUT, read_data, 6);
  if (!success)
  {
    return false;
  }

  accel_x = static_cast<int16_t>(read_data[0] << 8 | read_data[1]) * accel_scale + mpu_6050_factory_trim_data.
    ft_xa_change;
  accel_y = static_cast<int16_t>(read_data[2] << 8 | read_data[3]) * accel_scale + mpu_6050_factory_trim_data.
    ft_ya_change;
  accel_z = static_cast<int16_t>(read_data[4] << 8 | read_data[5]) * accel_scale + mpu_6050_factory_trim_data.
    ft_za_change;

  return true;
}

void mpu_6050::accel_loop(CollectionData& collection_data)
{
  static bool device_detected = false;
  static uint8_t no_detect_cycles = 0;

  if (!device_detected)
  {
    no_detect_cycles = 0;
    device_detected = check_chip_id();
    if (!device_detected)
    {
      set_fault(status_manager::DEVICE_MPU_6050, true);
      usb_communication::send_string("Fault: MPU 6050, device not detected");
      return;
    }

    const bool success = self_test();
    if (!success)
    {
      device_detected = false;
      set_fault(status_manager::DEVICE_MPU_6050, true);
      usb_communication::send_string("Fault: MPU 6050, failed to configure/self-test");
      return;
    }
    usb_communication::send_string("MPU 6050 configured");
  }

  if (!gpio_get(MPU_6050_INT_PIN))
  {
    if (no_detect_cycles == MAX_MPU_6050_NOT_READY_CYCLES)
    {
      usb_communication::send_string(std::format("MPU 6050 not interrupted for {} cycles",
                                                 MAX_MPU_6050_NOT_READY_CYCLES));
      no_detect_cycles = 0;
      device_detected = check_chip_id();
      if (!device_detected)
      {
        set_fault(status_manager::DEVICE_MPU_6050, true);
        usb_communication::send_string("Fault: MPU 6050, device not detected, ready pin not set");
      }
    }
    else
    {
      no_detect_cycles++;
    }
    return;
  }

  no_detect_cycles = 0;
  const bool success = get_data(collection_data.accel_x, collection_data.accel_y, collection_data.accel_z);
  if (!success)
  {
    usb_communication::send_string("Fault: MPU 6050, failed to read outputs");
    set_fault(status_manager::DEVICE_MPU_6050, true);
  }

  double accel_magnitude = sqrt(
    pow(collection_data.accel_x, 2) + pow(collection_data.accel_y, 2) + pow(collection_data.accel_z, 2));
  usb_communication::send_string(std::format("magnitude {}", accel_magnitude));

  set_fault(status_manager::DEVICE_MPU_6050, false);
}
