#pragma once

#include <functional>

#include "hardware/i2c.h"

#define I2C_TIMEOUT_MS 50

namespace i2c_util
{
  inline uint baud_rate_bus0, baud_rate_bus1;

  void i2c_bus_init(i2c_inst_t* i2c, uint sda_pin, uint scl_pin, uint baud_rate);
  void recover_i2c(i2c_inst_t* i2c, uint8_t sda_pin, uint8_t scl_pin);

  bool read_byte(i2c_inst_t* i2c, uint8_t dev_addr, uint8_t reg_addr, int8_t& output);
  bool read_ubyte(i2c_inst_t* i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t& output);
  bool read_short(i2c_inst_t* i2c, uint8_t dev_addr, uint8_t reg_addr, int16_t& output);
  bool read_ushort(i2c_inst_t* i2c, uint8_t dev_addr, uint8_t reg_addr, uint16_t& output);
  bool read_short_reversed(i2c_inst_t* i2c, uint8_t dev_addr, uint8_t reg_addr, int16_t& output);
  bool read_ushort_reversed(i2c_inst_t* i2c, uint8_t dev_addr, uint8_t reg_addr, uint16_t& output);
  bool read_bytes(i2c_inst_t* i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t output[], uint8_t len);

  uint8_t scan_for_devices(i2c_inst_t* i2c, const std::function<void(uint8_t)>& on_device_found);
}
