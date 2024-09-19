#pragma once

#include "hardware/i2c.h"

namespace i2c_util
{
  bool read_byte(i2c_inst_t *i2c, uint8_t dev_addr, uint8_t reg_addr, int8_t &output);
  bool read_ubyte(i2c_inst_t *i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t &output);
  bool read_short(i2c_inst_t *i2c, uint8_t dev_addr, uint8_t reg_addr, int16_t &output);
  bool read_ushort(i2c_inst_t *i2c, uint8_t dev_addr, uint8_t reg_addr, uint16_t &output);
  bool read_bytes(i2c_inst_t *i2c, uint8_t dev_addr, uint8_t reg_addr, uint8_t output[], uint8_t len);
}