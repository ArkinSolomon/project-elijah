#include "mpu_6050.h"

#include <hardware/watchdog.h>
#include <cmath>
#include <hardware/gpio.h>

#include "elijah_state_framework.h"
#include "i2c_util.h"

MPU6050::MPU6050(i2c_inst_t* i2c_inst, const uint8_t i2c_addr, const GyroFullScaleRange default_gyro_range,
                 const AccelFullScaleRange default_accel_range) : i2c_inst(i2c_inst), i2c_addr(i2c_addr),
                                                                  default_gyro_range(default_gyro_range),
                                                                  default_accel_range(default_accel_range)
{
}

double MPU6050::get_accel_scale(const AccelFullScaleRange accel_range)
{
  switch (accel_range)
  {
  case AccelFullScaleRange::Range2g:
    return GRAVITY_CONSTANT * 1.0 / 16384.0;
  case AccelFullScaleRange::Range4g:
    return GRAVITY_CONSTANT * 2.0 / 16384.0;
  case AccelFullScaleRange::Range8g:
    return GRAVITY_CONSTANT * 4.0 / 16384.0;
  case AccelFullScaleRange::Range16g:
    return GRAVITY_CONSTANT * 8.0 / 16384.0;
  }
  return 0;
}

double MPU6050::get_gyro_scale(const GyroFullScaleRange gyro_range)
{
  switch (gyro_range)
  {
  // No clue where these numbers come from but everyone else uses them so idk
  case GyroFullScaleRange::Range250:
    return 0.007633;
  case GyroFullScaleRange::Range500:
    return 0.015267;
  case GyroFullScaleRange::Range1000:
    return 0.030487;
  case GyroFullScaleRange::Range2000:
    return 0.060975f;
  }
  return 0;
}

double MPU6050::get_magnitude(const double x, const double y, const double z)
{
  return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

bool MPU6050::check_chip_id() const
{
  uint8_t read_id;
  const bool success = i2c_util::read_ubyte(i2c_inst, i2c_addr, REG_WHO_AM_I, read_id);
  return success && read_id == MPU_6050_DEVICE_ID;
}

bool MPU6050::configure(const uint8_t dlpf_cfg, const GyroFullScaleRange gyro_range,
                        const AccelFullScaleRange accel_range,
                        const bool self_test_en, const bool enable_ints)
{
  const uint8_t config_reg_data = dlpf_cfg & 0x07;
  uint8_t gyro_config_reg_data = (static_cast<uint8_t>(gyro_range) << 3) & 0x18;
  uint8_t accel_config_reg_data = (static_cast<uint8_t>(accel_range) << 3) & 0x18;

  if (self_test_en)
  {
    gyro_config_reg_data |= 0xE0;
    accel_config_reg_data |= 0xE0;
  }

  const uint8_t write_cfg_data[4] = {
    REG_CONFIG, config_reg_data, gyro_config_reg_data, accel_config_reg_data
  };

  int bytes_written = i2c_write_blocking_until(i2c_inst, i2c_addr, write_cfg_data, 4, false,
                                               delayed_by_ms(get_absolute_time(), 64));
  if (bytes_written != 4)
  {
    return false;
  }

  calibration_data.accel_scale = get_accel_scale(accel_range);
  calibration_data.gyro_scale = get_gyro_scale(gyro_range);

  const uint8_t int_en_data[2] = {REG_INT_ENABLE, static_cast<uint8_t>(enable_ints ? 0x01 : 0x00)};
  bytes_written = i2c_write_blocking_until(i2c_inst, i2c_addr, int_en_data, 2, false,
                                           delayed_by_ms(get_absolute_time(), 32));
  if (bytes_written != 2)
  {
    return false;
  }

  constexpr uint8_t power_mgmt_data[2] = {REG_PWR_MGMT_1, 0x08};
  bytes_written = i2c_write_blocking_until(i2c_inst, i2c_addr, power_mgmt_data, 2, false,
                                           delayed_by_ms(get_absolute_time(), 32));
  if (bytes_written != 2)
  {
    return false;
  }

  uint8_t output[3];
  const bool read_success = i2c_util::read_bytes(i2c_inst, i2c_addr, REG_CONFIG, output, 3);
  if (!read_success || output[0] != config_reg_data || output[1] != gyro_config_reg_data || output[2] !=
    accel_config_reg_data)
  {
    // elijah_state_framework::log_serial_message(std::format(
    //     "Read success: {}, output[0]: {}, expected: {}, output[1]: {}, expected: {}, output[2]: {}, expected: {}",
    //     read_success, output[0], config_reg_data, output[1], gyro_config_reg_data, output[2], accel_config_reg_data
    //   )
    // );
    return false;
  }
  return true;
}

bool MPU6050::configure_default()
{
  return configure(CONFIG_MPU_6050_DLPF_CFG, default_gyro_range,
                   default_accel_range, false, false);
}

bool MPU6050::calibrate(const uint calibration_cycles, const double expected_xa, const double expected_ya,
                        const double expected_za, const double expected_xg, const double expected_yg,
                        const double expected_zg)
{
  bool configure_success = configure(CONFIG_MPU_6050_DLPF_CFG, GyroFullScaleRange::Range250,
                                     AccelFullScaleRange::Range2g, false, false);

  if (!configure_success)
  {
    return false;
  }

  double total_xa = 0, total_ya = 0, total_za = 0;
  double total_xg = 0, total_yg = 0, total_zg = 0;
  for (int i = 1; i <= calibration_cycles; i++)
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

  configure_success = configure_default();

  if (!configure_success)
  {
    return false;
  }

  return true;
}

void MPU6050::load_calibration_data(const double diff_xa, const double diff_ya, const double diff_za,
                                    const double diff_xg, const double diff_yg,
                                    const double diff_zg)
{
  calibration_data.diff_xa = diff_xa;
  calibration_data.diff_ya = diff_ya;
  calibration_data.diff_za = diff_za;

  calibration_data.diff_xg = diff_xg;
  calibration_data.diff_yg = diff_yg;
  calibration_data.diff_zg = diff_zg;
}

const MPU6050::CalibrationData& MPU6050::get_calibration_data() const
{
  return calibration_data;
}

bool MPU6050::get_raw_data(int16_t& raw_xa, int16_t& raw_ya, int16_t& raw_za, int16_t& raw_xg, int16_t& raw_yg,
                           int16_t& raw_zg)
{
  uint8_t read_data[14];
  const bool success = i2c_util::read_bytes(i2c_inst, i2c_addr, REG_ACCEL_XOUT, read_data, 14);
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

bool MPU6050::get_uncompensated_data(double& uncomp_xa, double& uncomp_ya, double& uncomp_za, double& uncomp_xg,
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

bool MPU6050::get_data(double& xa, double& ya, double& za, double& xg, double& yg, double& zg)
{
  double uncomp_xa, uncomp_ya, uncomp_za, uncomp_xg, uncomp_yg, uncomp_zg;
  const bool success = get_uncompensated_data(uncomp_xa, uncomp_ya, uncomp_za, uncomp_xg, uncomp_yg, uncomp_zg);
  if (!success)
  {
    return false;
  }

  xa = uncomp_xa + calibration_data.diff_xa;
  ya = uncomp_ya + calibration_data.diff_ya;
  za = uncomp_za + calibration_data.diff_za;

  xg = uncomp_xg + calibration_data.diff_xg;
  yg = uncomp_yg + calibration_data.diff_yg;
  zg = uncomp_zg + calibration_data.diff_zg;

  return true;
}
