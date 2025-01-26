#pragma once

#include <cstdint>
#include <pico/critical_section.h>
#include <pico/mutex.h>

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

struct CollectionData;

namespace mpu_6050
{
  namespace _reg_defs
  {
    constexpr uint8_t REG_SELF_TEST_X = 0x0D;
    constexpr uint8_t REG_SELF_TEST_Y = 0x0E;
    constexpr uint8_t REG_SELF_TEST_Z = 0x0F;
    constexpr uint8_t REG_SELF_TEST_A = 0x10;

    constexpr uint8_t REG_CONFIG = 0x1A;
    constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
    constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;

    constexpr uint8_t REG_INT_ENABLE = 0x38;

    constexpr uint8_t REG_ACCEL_XOUT = 0x3B;
    constexpr uint8_t REG_ACCEL_YOUT = 0x3D;
    constexpr uint8_t REG_ACCEL_ZOUT = 0x3F;

    constexpr uint8_t REG_TEMP_OUT = 0x41;

    constexpr uint8_t REG_GYRO_XOUT = 0x43;
    constexpr uint8_t REG_GYRO_YOUT = 0x45;
    constexpr uint8_t REG_GYRO_ZOUT = 0x47;

    constexpr uint8_t REG_USER_CTRL = 0x6A;
    constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
    constexpr uint8_t REG_WHO_AM_I = 0x75;
  }

  enum class gyro_full_scale_range
  {
    RANGE_250 = 0b00,
    RANGE_500 = 0b01,
    RANGE_1000 = 0b10,
    RANGE_2000 = 0b11
  };

  enum class accel_full_scale_range
  {
    RANGE_2g = 0b00,
    RANGE_4g = 0b01,
    RANGE_8g = 0b10,
    RANGE_16g = 0b11
  };

  struct CalibrationData
  {
    double diff_xa, diff_ya, diff_za;
    double diff_xg, diff_yg, diff_zg;
    double accel_scale, gyro_scale;
  };

  inline CalibrationData calibration_data{};
  inline critical_section_t mpu_6050_cs;

  void init();
  bool check_chip_id();

  bool configure(uint8_t dlpf_cfg, gyro_full_scale_range gyro_range, accel_full_scale_range accel_range,
                 bool self_test_en, bool enable_ints);
  double get_accel_scale(accel_full_scale_range accel_range);
  double get_gyro_scale(gyro_full_scale_range gyro_range);

  bool configure_default();

  void write_calibration_data(void*);
  bool read_calibration_data();
  void encode_calibration_data(uint8_t* encoded_data);
  void send_calibration_data();
  void calibrate();

  bool get_raw_data(int16_t& raw_xa, int16_t& raw_ya, int16_t& raw_za, int16_t& raw_xg, int16_t& raw_yg, int16_t& raw_zg);
  bool get_uncompensated_data(double& uncomp_xa, double& uncomp_ya, double& uncomp_za, double& uncomp_xg, double& uncomp_yg, double& uncomp_zg);

  void data_loop(CollectionData& collection_data);
}
