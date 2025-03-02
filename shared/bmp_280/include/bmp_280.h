#pragma once

#include <hardware/i2c.h>
#include <hardware/spi.h>

#define BMP_280_CHIP_ID 0x58
#define BMP_280_RESET_VALUE 0xB6

class BMP280
{
public:
  enum class DeviceMode : uint8_t
  {
    SleepMode = 0b00,
    ForcedMode = 0b01,
    NormalMode = 0b11
  };

  enum class StandbyTimeSetting : uint8_t
  {
    Standby500us = 0b000,
    Standby62500us = 0b001,
    Standby125ms = 0b010,
    Standby250ms = 0b011,
    Standby500ms = 0b100,
    Standby1s = 0b101,
    Standby2s = 0b110,
    Standby4s = 0b111
  };

  enum class OssSettingPressure : uint8_t
  {
    PressureOssSkipped = 0b000,
    PressureOss1 = 0b001,
    PressureOss2 = 0b010,
    PressureOss4 = 0b011,
    PressureOss8 = 0b100,
    PressureOss16 = 0b101
  };

  enum class OssSettingTemperature : uint8_t
  {
    TemperatureOssSkipped = 0b000,
    TemperatureOss1 = 0b001,
    TemperatureOss2 = 0b010,
    TemperatureOss4 = 0b011,
    TemperatureOss8 = 0b100,
    TemperatureOss16 = 0b101
  };

  enum class FilterCoefficientSetting : uint8_t
  {
    FilterOff = 0x00,
    Filter2x = 0x01,
    Filter4x = 0x02,
    Filter8x = 0x03,
    Filter16x = 0x04
  };

  struct CalibrationData
  {
    uint16_t dig_T1 = 0;
    int16_t dig_T2 = -1, dig_T3 = -1;
    uint16_t dig_P1 = 0;
    int16_t dig_P2 = -1, dig_P3 = -1, dig_P4 = -1, dig_P5 = -1, dig_P6 = -1, dig_P7 = -1, dig_P8 = -1, dig_P9 = -1;
    double baro_pressure = 101325; // Pascals
  };

  BMP280(i2c_inst_t* i2c, uint8_t addr);
  BMP280(spi_inst_t* spi, uint8_t csn_gpio);

  [[nodiscard]] const CalibrationData& get_calibration_data() const;

  [[nodiscard]] bool check_chip_id() const;
  bool soft_reset() const; // NOLINT(*-use-nodiscard)

  bool check_status(bool& is_measuring, bool& is_updating) const;
  bool change_settings(DeviceMode mode, StandbyTimeSetting standby_time, FilterCoefficientSetting filter_setting,
                       OssSettingPressure pressure_oss, OssSettingTemperature temperature_oss);

  bool read_calibration_data();
  bool read_press_temp_alt(int32_t& pressure, double& temperature, double& altitude) const;
  // void data_collection_loop(CollectionData& collection_data);

private:
  // 2-byte registers
  static constexpr uint8_t REG_DIG_T1 = 0x88;
  static constexpr uint8_t REG_DIG_T2 = 0x8A;
  static constexpr uint8_t REG_DIG_T3 = 0x8C;
  static constexpr uint8_t REG_DIG_P1 = 0x8E;
  static constexpr uint8_t REG_DIG_P2 = 0x90;
  static constexpr uint8_t REG_DIG_P3 = 0x92;
  static constexpr uint8_t REG_DIG_P4 = 0x94;
  static constexpr uint8_t REG_DIG_P5 = 0x96;
  static constexpr uint8_t REG_DIG_P6 = 0x98;
  static constexpr uint8_t REG_DIG_P7 = 0x9A;
  static constexpr uint8_t REG_DIG_P8 = 0x9C;
  static constexpr uint8_t REG_DIG_P9 = 0x9E;

  static constexpr uint8_t REG_CHIP_ID = 0xD0;
  static constexpr uint8_t REG_SOFT_RESET = 0xE0;
  static constexpr uint8_t REG_STATUS = 0xF3;
  static constexpr uint8_t REG_CTRL_MEAS = 0xF4;
  static constexpr uint8_t REG_CONFIG = 0xF5;

  static constexpr uint8_t REG_PRESS_MSB = 0xF7;
  static constexpr uint8_t REG_PRESS_LSB = 0xF8;
  static constexpr uint8_t REG_PRESS_XLSB = 0xF9;

  static constexpr uint8_t REG_TEMP_MSB = 0xFA;
  static constexpr uint8_t REG_TEMP_LSB = 0xFB;
  static constexpr uint8_t REG_TEMP_XLSB = 0xFC;

  bool is_i2c_interface;

  i2c_inst_t* i2c_inst = nullptr;
  uint8_t i2c_addr = 0xFF;

  spi_inst_t* spi_inst = nullptr;
  uint8_t csn_pin = 0xFF;

  CalibrationData calibration_data{};

  bool read_byte(uint8_t reg_addr, uint8_t& value) const;
  bool read_short(uint8_t reg_addr, int16_t& value) const;
  bool read_ushort(uint8_t reg_addr, uint16_t& value) const;
  bool read_bytes(uint8_t reg_addr, uint8_t* data, size_t len) const;
  bool write_bytes_to_device(uint8_t start_reg_addr, const uint8_t* data, size_t len) const;
  void read_spi_bytes(uint8_t start_reg_addr, uint8_t* data, size_t len) const;
};
