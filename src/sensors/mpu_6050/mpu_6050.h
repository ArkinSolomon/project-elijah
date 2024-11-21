#pragma once

#include <cstdint>
#include <pico/critical_section.h>
#include <pico/mutex.h>

#define MPU_6050_ADDR 0x68
#define MPU_6050_DEVICE_ID 0x68

#define CONFIG_MPU_6050_DLPF_CFG 0b010 // See datasheet

#define GRAVITY_CONSTANT 9.80665

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

  enum class lp_wake_ctrl
  {
    FREQ_1250mHz = 0b00,
    FREQ_5Hz = 0b01,
    FREQ_20Hz = 0b10,
    FREQ_40Hz = 0b11
  };

  struct FactoryTrimData
  {
    double ft_xa, ft_ya, ft_za;
    double ft_xa_change, ft_ya_change, ft_za_change;
  };

  struct ReadSensorData
  {
    double accel_x, accel_y, accel_z;
    absolute_time_t last_update_time;
  };

  inline FactoryTrimData mpu_6050_factory_trim_data{};
  inline double accel_scale;

  inline volatile ReadSensorData irq_sens_data{
    -1, -1, -1, 0
  };
  inline critical_section_t irq_sens_data_cs;

  inline mutex_t mpu_6050_lock;

  bool check_chip_id();
  bool configure(uint8_t dlpf_cfg, gyro_full_scale_range gyro_range, accel_full_scale_range accel_range,
                 lp_wake_ctrl wake_ctrl);
  bool configure_default();
  double get_accel_scale(accel_full_scale_range accel_range);

  bool self_test();
  double factory_trim_gyro(double test_value);
  double factory_trim_accel(double test_value);
  double calculate_self_test_change(double str, double ft);

  bool get_data(double& accel_x, double& accel_y, double& accel_z);
  void accel_loop(CollectionData& collection_data);
}
