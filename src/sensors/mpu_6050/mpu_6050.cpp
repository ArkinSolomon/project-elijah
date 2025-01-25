#include "mpu_6050.h"

#include <cmath>
#include <format>

#include "pico/rand.h"
#include "lock_nums.h"
#include "main.h"
#include "pin_outs.h"
#include "status_manager.h"
#include "usb_communication.h"
#include "sensors/i2c/i2c_util.h"

mpu_6050::ReadSensorData::ReadSensorData()
{
  accel_x = accel_y = accel_z = -1;
  update_time = 0;
}

mpu_6050::ReadSensorData::ReadSensorData(volatile const ReadSensorData& read_sensor_data)
{
  accel_x = read_sensor_data.accel_x;
  accel_y = read_sensor_data.accel_y;
  accel_z = read_sensor_data.accel_z;
  update_time = read_sensor_data.update_time;
}

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
                         const accel_full_scale_range accel_range, lp_wake_ctrl wake_ctrl,
                         const bool self_test)
{
  switch (wake_ctrl)
  {
  case lp_wake_ctrl::FREQ_1250mHz:
    max_time_since_irq = 1500;
    break;
  case lp_wake_ctrl::FREQ_5Hz:
    max_time_since_irq = 500;
    break;
  case lp_wake_ctrl::FREQ_20Hz:
    max_time_since_irq = 110;
    break;
  case lp_wake_ctrl::FREQ_40Hz:
    max_time_since_irq = 75;
    break;
  }

  const uint8_t config_reg_data = dlpf_cfg & 0x07;
  uint8_t gyro_config_reg_data = static_cast<uint8_t>(gyro_range) << 3 & 0x18;
  uint8_t accel_config_reg_dat = static_cast<uint8_t>(accel_range) << 2 & 0x18;

  if (self_test)
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

  accel_scale = get_accel_scale(accel_range);

  constexpr uint8_t int_en_data[2] = {_reg_defs::REG_INT_ENABLE, 0x01};
  bytes_written = i2c_write_blocking_until(I2C_BUS1, MPU_6050_ADDR, int_en_data, 2, false,
                                           delayed_by_ms(get_absolute_time(), 32));
  if (bytes_written != 2)
  {
    return false;
  }

  const uint8_t wake_ctrl_reg_val = static_cast<uint8_t>(wake_ctrl) << 6 | 0x07;
  const uint8_t sleep_disable_data[3] = {_reg_defs::REG_PWR_MGMT_1, 0x28, wake_ctrl_reg_val};
  bytes_written = i2c_write_blocking_until(I2C_BUS1, MPU_6050_ADDR, sleep_disable_data, 3, false,
                                           delayed_by_ms(get_absolute_time(), 32));
  return bytes_written == 3;
}

bool mpu_6050::configure_default()
{
  return configure(CONFIG_MPU_6050_DLPF_CFG, gyro_full_scale_range::RANGE_250,
                   accel_full_scale_range::RANGE_2g, lp_wake_ctrl::FREQ_20Hz, false);
}

bool mpu_6050::configure_default_with_lock()
{
  critical_section_enter_blocking(&mpu_6050_cs);
  const bool ret_val = configure_default();
  critical_section_exit(&mpu_6050_cs);
  return ret_val;
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

void mpu_6050::init_crit_section()
{
  if (!critical_section_is_initialized(&mpu_6050_cs))
  {
    critical_section_init_with_lock_num(&mpu_6050_cs, CS_LOCK_NUM_MPU_SENS_DATA);
  }
}

bool mpu_6050::self_test()
{
  critical_section_enter_blocking(&mpu_6050_cs);
  gpio_set_irq_enabled(MPU_6050_INT_PIN, GPIO_IRQ_EDGE_RISE, false);
  bool success = configure(CONFIG_MPU_6050_DLPF_CFG, gyro_full_scale_range::RANGE_250,
                           accel_full_scale_range::RANGE_8g, lp_wake_ctrl::FREQ_40Hz, true);
  if (!success)
  {
    return false;
  }

  uint8_t self_test_registers[5];
  success = i2c_util::read_bytes(I2C_BUS1, MPU_6050_ADDR, _reg_defs::REG_SELF_TEST_X, self_test_registers, 5);
  if (!success)
  {
    return false;
  }

  const uint8_t xa_test = self_test_registers[1] >> 5 << 2 | self_test_registers[4] & 0x30
                , ya_test = self_test_registers[2] >> 5 << 2 | self_test_registers[4] & 0x0C
                , za_test = self_test_registers[3] >> 5 << 2 | self_test_registers[4] & 0x03;

  mpu_6050_factory_trim_data.ft_xa = factory_trim_accel(xa_test);
  mpu_6050_factory_trim_data.ft_ya = factory_trim_accel(ya_test);
  mpu_6050_factory_trim_data.ft_za = factory_trim_accel(za_test);

  while (gpio_get(MPU_6050_INT_PIN))
  {
  }
  while (!gpio_get(MPU_6050_INT_PIN))
  {
  }

  double xa_st_en, ya_st_en, za_st_en;
  success = get_data(xa_st_en, ya_st_en, za_st_en);
  if (!success)
  {
    return false;
  }

  success = configure(CONFIG_MPU_6050_DLPF_CFG, gyro_full_scale_range::RANGE_250,
                      accel_full_scale_range::RANGE_8g, lp_wake_ctrl::FREQ_40Hz, false);
  if (!success)
  {
    return false;
  }

  while (gpio_get(MPU_6050_INT_PIN))
  {
  }
  while (!gpio_get(MPU_6050_INT_PIN))
  {
  }

  double xa_st_dis, ya_st_dis, za_st_dis;
  success = get_data(xa_st_dis, ya_st_dis, za_st_dis);
  if (!success)
  {
    return false;
  }

  const double str_xa = xa_st_en - xa_st_dis
               , str_ya = ya_st_dis - ya_st_en
               , str_za = za_st_dis - za_st_en;

  usb_communication::send_string(std::format("{} - {}, {} - {}, {} - {}", xa_st_en, xa_st_dis, ya_st_en, ya_st_dis,
                                             za_st_en, za_st_dis));

  mpu_6050_factory_trim_data.ft_xa_change_percent = calculate_self_test_change_percent(
    str_xa, mpu_6050_factory_trim_data.ft_xa);
  mpu_6050_factory_trim_data.ft_ya_change_percent = calculate_self_test_change_percent(
    str_ya, mpu_6050_factory_trim_data.ft_ya);
  mpu_6050_factory_trim_data.ft_za_change_percent = calculate_self_test_change_percent(
    str_za, mpu_6050_factory_trim_data.ft_za);

  const bool ret_val = configure_default();
  gpio_set_irq_enabled(MPU_6050_INT_PIN, GPIO_IRQ_EDGE_RISE, true);
  while (gpio_get(MPU_6050_INT_PIN))
  {
  }

  critical_section_exit(&mpu_6050_cs);
  return ret_val;
}

double mpu_6050::factory_trim_accel(const double test_value)
{
  if (test_value == 0)
  {
    return 0;
  }

  return 4096 * 0.34 * std::pow(0.92 / 0.34, (test_value - 1) / 30);
}

double mpu_6050::calculate_self_test_change_percent(const double str, const double ft)
{
  if (ft == 0)
  {
    return -9999999;
  }

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

  accel_x = static_cast<int16_t>(read_data[0] << 8 | read_data[1]) * accel_scale; // * mpu_6050_factory_trim_data.
  // ft_xa_change;
  accel_y = static_cast<int16_t>(read_data[2] << 8 | read_data[3]) * accel_scale; // * mpu_6050_factory_trim_data.
  // ft_ya_change;
  accel_z = static_cast<int16_t>(read_data[4] << 8 | read_data[5]) * accel_scale; // * mpu_6050_factory_trim_data.
  // ft_za_change;

  return true;
}

void mpu_6050::data_int(uint gpio, uint32_t event_mask)
{
  critical_section_enter_blocking(&mpu_6050_cs);

  ReadSensorData data;
  const bool success = get_data(data.accel_x, data.accel_y, data.accel_z);
  if (!success)
  {
    usb_communication::send_string("MPU 6050 failed to get data on interrupt");
    set_fault(status_manager::DEVICE_MPU_6050, true);
    return;
  }

  set_fault(status_manager::DEVICE_MPU_6050, false);

  data.update_time = get_absolute_time();

  irq_sens_data.accel_x = data.accel_x;
  irq_sens_data.accel_y = data.accel_y;
  irq_sens_data.accel_z = data.accel_z;
  irq_sens_data.update_time = data.update_time;
  critical_section_exit(&mpu_6050_cs);
}

void mpu_6050::accel_loop(CollectionData& collection_data)
{
  critical_section_enter_blocking(&mpu_6050_cs);
  const ReadSensorData last_sensor_data = irq_sens_data;

  if (absolute_time_diff_us(last_sensor_data.update_time, get_absolute_time()) > MAX_CYCLE_DELAY_DIFF_MS * 1000)
  {
    if (!check_chip_id())
    {
      // usb_communication::send_string(std::format(
      //     "Fault: MPU 6050, it's been more than {}ms since a sensor update (updated at {}), and the device is not detected",
      //     MAX_CYCLE_DELAY_DIFF_MS, last_sensor_data.update_time)
      // );
    }
    else
    {
      usb_communication::send_string(std::format(
          "Fault: MPU 6050, it's been more than {}ms since a sensor update (updated at {}), but the device is still detected",
          MAX_CYCLE_DELAY_DIFF_MS, last_sensor_data.update_time)
      );
      configure_default();
    }

    set_fault(status_manager::DEVICE_MPU_6050, true);
    critical_section_exit(&mpu_6050_cs);
    return;
  }

  collection_data.accel_x = last_sensor_data.accel_x;
  collection_data.accel_y = last_sensor_data.accel_y;
  collection_data.accel_z = last_sensor_data.accel_z;

  set_fault(status_manager::DEVICE_MPU_6050, false);
  critical_section_exit(&mpu_6050_cs);
}
