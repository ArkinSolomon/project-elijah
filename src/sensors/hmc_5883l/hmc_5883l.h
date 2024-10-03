#pragma once
#include <cstdint>

#define HMC_5883L_ADDR 0b111100
#define HMC_5883L_DEVICE_ID 0x483433

struct CollectionData;

namespace hmc_5883l
{
  namespace _reg_defs
  {
    constexpr uint8_t REG_CONFIG_A = 0x00;
    constexpr uint8_t REG_CONFIG_B = 0x01;
    constexpr uint8_t REG_MODE = 0x02;
    constexpr uint8_t REG_OUT_X = 0x03;
    constexpr uint8_t REG_OUT_Z = 0x05;
    constexpr uint8_t REG_OUT_Y = 0x07;
    constexpr uint8_t REG_STATUS = 0x08;
    constexpr uint8_t REG_IDENT_A = 0x09;
    constexpr uint8_t REG_IDENT_B = 0x0A;
    constexpr uint8_t REG_IDENT_C = 0x0B;
  }

  enum class samples_averaged
  {
    ONE_SAMPLE = 0b00,
    TWO_SAMPLES = 0b01,
    FOUR_SAMPLES = 0b10,
    EIGHT_SAMPLES = 0b11
  };

  enum class data_output_rate
  {
    OUTPUT_RATE_750mHz = 0b000,
    OUTPUT_RATE_1500mHz = 0b001,
    OUTPUT_RATE_3000mHz = 0b010,
    OUTPUT_RATE_7500mHz = 0b011,
    OUTPUT_RATE_15Hz = 0b100,
    OUTPUT_RATE_30Hz = 0b101,
    OUTPUT_RATE_75Hz = 0b110,
  };

  enum class measurement_mode
  {
    NORMAL = 0b00,
    POSITIVE = 0b01,
    NEGATIVE = 0b10
  };

  enum class gain_setting
  {
    GAIN_1370_GAUSS = 0b000,
    GAIN_1090_GAUSS = 0b001,
    GAIN_820_GAUSS = 0b010,
    GAIN_660_GAUSS = 0b011,
    GAIN_440_GAUSS = 0b100,
    GAIN_390_GAUSS = 0b101,
    GAIN_330_GAUSS = 0b110,
    GAIN_230_GAUSS = 0b111
  };

  enum class operating_mode
  {
    CONTINUOUS_MEASUREMENT = 0b00,
    SINGLE_MEASUREMENT = 0b01,
    IDLE = 0b10
  };

  bool check_device_id();
  bool configure(samples_averaged samples_averaged, data_output_rate data_output_rate,
                 measurement_mode measurement_mode, gain_setting gain_configuration, bool isHighSpeedI2c,
                 operating_mode
                 operating_mode);
  void accel_loop(CollectionData& collection_data);
}
