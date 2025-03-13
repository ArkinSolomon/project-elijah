#pragma once

#include <cstdint>
#include <hardware/i2c.h>

#define MPU_6050_ADDR 0x68
#define MPU_6050_DEVICE_ID 0x68

#define CONFIG_MPU_6050_DLPF_CFG 3 // See datasheet

#define GRAVITY_CONSTANT 9.80665
#define MAX_CYCLE_DELAY_DIFF_MS 300

#define MPU_6050_CALIBRATION_SIZE 48
#define MPU_6050_FLASH_TIMEOUT_MS 100
#define MPU_6050_CALIBRATION_CHECK 0xB04374211578AE55
#define MPU_6050_CALIB_FLASH_SECTOR_NUM 499
#define MPU_6050_CALIBRATION_CYCLES 100
#define MS_BETWEEN_CALIBRATION_CYCLES 10

class MPU6050
{
public:
  enum class GyroFullScaleRange : uint8_t
  {
    Range250 = 0b00,
    Range500 = 0b01,
    Range1000 = 0b10,
    Range2000 = 0b11
  };

  enum class AccelFullScaleRange : uint8_t
  {
    Range2g = 0b00,
    Range4g = 0b01,
    Range8g = 0b10,
    Range16g = 0b11
  };

  struct CalibrationData
  {
    double diff_xa, diff_ya, diff_za;
    double diff_xg, diff_yg, diff_zg;
    double accel_scale, gyro_scale;
  };

  MPU6050(i2c_inst_t* i2c_inst, uint8_t i2c_addr, GyroFullScaleRange default_gyro_range,
          AccelFullScaleRange default_accel_range);

  static double get_accel_scale(AccelFullScaleRange accel_range);
  static double get_gyro_scale(GyroFullScaleRange gyro_range);
  static double get_magnitude(double x, double y, double z);

  [[nodiscard]] bool check_chip_id() const;
  bool configure(uint8_t dlpf_cfg, GyroFullScaleRange gyro_range, AccelFullScaleRange accel_range,
                 bool self_test_en, bool enable_ints);
  bool configure_default();

  bool calibrate(uint calibration_cycles, double expected_xa, double expected_ya, double expected_za, double expected_xg, double
                 expected_yg, double expected_zg);
  void load_calibration_data(double diff_xa, double diff_ya, double diff_za, double diff_xg, double diff_yg, double diff_zg);
  [[nodiscard]] const CalibrationData& get_calibration_data() const;

  bool get_raw_data(int16_t& raw_xa, int16_t& raw_ya, int16_t& raw_za, int16_t& raw_xg, int16_t& raw_yg,
                    int16_t& raw_zg);
  bool get_uncompensated_data(double& uncomp_xa, double& uncomp_ya, double& uncomp_za, double& uncomp_xg,
                              double& uncomp_yg, double& uncomp_zg);
  bool get_data(double& xa, double& ya, double& za, double& xg,
                double& yg, double& zg);

private:
  static constexpr uint8_t REG_SELF_TEST_X = 0x0D;
  static constexpr uint8_t REG_SELF_TEST_Y = 0x0E;
  static constexpr uint8_t REG_SELF_TEST_Z = 0x0F;
  static constexpr uint8_t REG_SELF_TEST_A = 0x10;

  static constexpr uint8_t REG_CONFIG = 0x1A;
  static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
  static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;

  static constexpr uint8_t REG_INT_ENABLE = 0x38;

  static constexpr uint8_t REG_ACCEL_XOUT = 0x3B;
  static constexpr uint8_t REG_ACCEL_YOUT = 0x3D;
  static constexpr uint8_t REG_ACCEL_ZOUT = 0x3F;

  static constexpr uint8_t REG_TEMP_OUT = 0x41;

  static constexpr uint8_t REG_GYRO_XOUT = 0x43;
  static constexpr uint8_t REG_GYRO_YOUT = 0x45;
  static constexpr uint8_t REG_GYRO_ZOUT = 0x47;

  static constexpr uint8_t REG_USER_CTRL = 0x6A;
  static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
  static constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
  static constexpr uint8_t REG_WHO_AM_I = 0x75;

  CalibrationData calibration_data{};

  i2c_inst_t* i2c_inst;
  uint8_t i2c_addr;

  GyroFullScaleRange default_gyro_range;
  AccelFullScaleRange default_accel_range;
};