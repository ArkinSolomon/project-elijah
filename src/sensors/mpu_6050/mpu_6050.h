#pragma once

#include <cstdint>

#define MPU_6050_ADDR 0x68
#define MPU_6050_DEVICE_ID 0b01101000

#define CONFIG_MPU_6050_INT_EN 0x01
#define CONFIG_MPU_6050_DLPF_CFG 0b010 // See datasheet

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

  bool check_chip_id();
  void accel_loop(CollectionData& collection_data);
}
