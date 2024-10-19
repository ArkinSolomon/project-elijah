#pragma once

#include <cstdint>

#define MPU_6050_ADDR 0x68
#define MPU_6050_DEVICE_ID 3

#define CONFIG_MPU_6050_DLPF_CFG 0b010 // See datasheet

#define MAX_MPU_6050_NOT_READY_CYCLES 150

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

  struct FactoryTrimData
  {
    double ft_xg, ft_yg, ft_zg;
    double ft_xa, ft_ya, ft_za;
    double ft_xg_change, ft_yg_change, ft_zg_change;
    double ft_xa_change, ft_ya_change, ft_za_change;
  };

  inline FactoryTrimData mpu_6050_factory_trim_data{};

  bool check_chip_id();
  bool configure(uint8_t dlpf_cfg, gyro_full_scale_range gyro_range, accel_full_scale_range accel_range, bool int_enable);
  bool configure_default();

  bool self_test();
  double factory_trim_gyro(double test_value);
  double factory_trim_accel(double test_value);
  double calculate_self_test_change(double str, double ft);

  bool get_data(int16_t& accel_x, int16_t& accel_y, int16_t& accel_z, int16_t& temp, int16_t& gyro_x, int16_t& gyro_y, int16_t& gyro_z);
  void accel_loop(CollectionData& collection_data);
}
