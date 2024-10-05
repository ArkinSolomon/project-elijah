#pragma once

#define BMP_280_ADDR 0b1110110

#define BMP_280_RESET_VALUE 0xB6
#define BMP_280_CHIP_ID 0x58
#include <cstdint>

struct CollectionData;

namespace bmp_280
{
  namespace _reg_defs
  {
    // 2-byte registers
    constexpr uint8_t REG_DIG_T1 = 0x88;
    constexpr uint8_t REG_DIG_T2 = 0x8A;
    constexpr uint8_t REG_DIG_T3 = 0x8C;
    constexpr uint8_t REG_DIG_P1 = 0x8E;
    constexpr uint8_t REG_DIG_P2 = 0x90;
    constexpr uint8_t REG_DIG_P3 = 0x92;
    constexpr uint8_t REG_DIG_P4 = 0x94;
    constexpr uint8_t REG_DIG_P5 = 0x96;
    constexpr uint8_t REG_DIG_P6 = 0x98;
    constexpr uint8_t REG_DIG_P7 = 0x9A;
    constexpr uint8_t REG_DIG_P8 = 0x9C;
    constexpr uint8_t REG_DIG_P9 = 0x9E;

    constexpr uint8_t REG_CHIP_ID = 0xD0;
    constexpr uint8_t REG_SOFT_RESET = 0xE0;
    constexpr uint8_t REG_STATUS = 0xF3;
    constexpr uint8_t REG_CTRL_MEAS = 0xF4;
    constexpr uint8_t REG_CONFIG = 0xF5;

    constexpr uint8_t REG_PRESS_MSB = 0xF7;
    constexpr uint8_t REG_PRESS_LSB = 0xF8;
    constexpr uint8_t REG_PRESS_XLSB = 0xF9;

    constexpr uint8_t REG_TEMP_MSB = 0xFA;
    constexpr uint8_t REG_TEMP_LSB = 0xFB;
    constexpr uint8_t REG_TEMP_XLSB = 0xFC;
  }

  enum device_mode
  {
    SLEEP_MODE = 0b00,
    FORCED_MODE = 0b01,
    NORMAL_MODE = 0b11
  };

  enum standby_time_setting
  {
    STANDBY_500us = 0b000,
    STANDBY_62500us = 0b001,
    STANDBY_125ms = 0b010,
    STANDBY_250ms = 0b011,
    STANDBY_500ms = 0b100,
    STANDBY_1s = 0b101,
    STANDBY_2s = 0b110,
    STANDBY_4s = 0b111
  };

  enum oss_setting_pressure
  {
    PRESSURE_OSS_SKIPPED = 0b000,
    PRESSURE_OSS1 = 0b001,
    PRESSURE_OSS2 = 0b010,
    PRESSURE_OSS4 = 0b011,
    PRESSURE_OSS8 = 0b100,
    PRESSURE_OSS16 = 0b101
  };

  enum oss_setting_temperature
  {
    TEMPERATURE_OSS_SKIPPED = 0b000,
    TEMPERATURE_OSS1 = 0b001,
    TEMPERATURE_OSS2 = 0b010,
    TEMPERATURE_OSS4 = 0b011,
    TEMPERATURE_OSS8 = 0b100,
    TEMPERATURE_OSS16 = 0b101
  };

  enum filter_coefficient_setting
  {
    FILTER_OFF = 0x00,
    FILTER_X2 = 0x01,
    FILTER_X4 = 0x02,
    FILTER_X8 = 0x03,
    FILTER_X16 = 0x04
  };

  struct CalibrationData
  {
    uint16_t dig_T1 = 0;
    int16_t dig_T2 = -1 , dig_T3 = -1;
    uint16_t dig_P1 = 0;
    int16_t dig_P2 = -1, dig_P3 = -1, dig_P4 = -1, dig_P5 = -1, dig_P6 = -1, dig_P7 = -1, dig_P8 = -1, dig_P9 = -1;
    double sea_level_pressure = 101325; // Pascals
  };

  inline CalibrationData bmp_280_calib_data{};

  bool check_chip_id();
  bool soft_reset();

  bool update_sea_level_pressure();
  bool update_sea_level_pressure(double pressure, bool write = true);
  bool read_stored_sea_level_pressure(double& pressure);

  bool check_status(bool& measuring, bool& updating);
  bool change_settings(device_mode mode, standby_time_setting standby_time, filter_coefficient_setting filter_setting,
                       oss_setting_pressure pressure_oss, oss_setting_temperature temperature_oss);
  bool read_calibration_data();
  void send_calibration_data();
  bool read_press_temp_alt(int32_t& pressure, double& temperature, double& altitude);



  void data_collection_loop(CollectionData& collection_data);
}
