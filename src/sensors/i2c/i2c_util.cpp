#include "i2c_util.h"
#include "../../pin_outs.h"
#include <cstdio>

void i2c_util::i2c_init(i2c_inst_t* i2c, const uint sda_pin, const uint scl_pin)
{
  // i2c@400kHz
  i2c_init(i2c, 400 * 1000);

  gpio_set_function(sda_pin, GPIO_FUNC_I2C);
  gpio_set_function(scl_pin, GPIO_FUNC_I2C);
  gpio_pull_up(sda_pin);
  gpio_pull_up(scl_pin);
}

void i2c_util::recover_i2c(i2c_inst_t* i2c, const uint8_t sda_pin, const uint8_t scl_pin)
{
  i2c_deinit(i2c);
  gpio_deinit(sda_pin);
  gpio_deinit(scl_pin);

  gpio_init(scl_pin);
  gpio_set_dir(scl_pin, true);

  // Send 9 clock cycles for release as specified in I2C 3.1.16 ~400kHz
  for (uint8_t i = 1; i <= 19; ++i)
  {
    gpio_put(scl_pin, i % 2);
    sleep_us(2);
  }

  gpio_deinit(scl_pin);
  i2c_init(i2c, sda_pin, scl_pin);
}


bool i2c_util::read_byte(i2c_inst_t *i2c, const uint8_t dev_addr, const uint8_t reg_addr, int8_t &output)
{
  uint8_t read_byte;
  const bool success = read_ubyte(i2c, dev_addr, reg_addr, read_byte);
  output = read_byte;
  return success;
}

bool i2c_util::read_ubyte(i2c_inst_t *i2c, const uint8_t dev_addr, const uint8_t reg_addr, uint8_t &output)
{
  return read_bytes(i2c, dev_addr, reg_addr, &output, 1);
}

bool i2c_util::read_short(i2c_inst_t *i2c, const uint8_t dev_addr, const uint8_t reg_addr, int16_t &output)
{
  uint16_t read_ushort;
  const bool success = i2c_util::read_ushort(i2c, dev_addr, reg_addr, read_ushort);
  if (!success)
  {
    return false;
  }

  output = read_ushort; // NOLINT(*-narrowing-conversions)
  return true;
}

bool i2c_util::read_ushort(i2c_inst_t *i2c, const uint8_t dev_addr, const uint8_t reg_addr, uint16_t &output)
{
  uint8_t read_data[2];
  const bool success = read_bytes(i2c, dev_addr, reg_addr, read_data, 2);
  if (!success)
  {
    return false;
  }

  output = (read_data[0] << 8) | read_data[1];
  return success;
}

/**
 * Read len bytes into output[].
 *
 * Returns false on failure, or true otherwise
 */
bool i2c_util::read_bytes(i2c_inst_t *i2c, const uint8_t dev_addr, const uint8_t reg_addr, uint8_t output[], const uint8_t len)
{

  const int bytes_written = i2c_write_blocking_until(i2c, dev_addr, &reg_addr, 1, true, delayed_by_ms(get_absolute_time(), 100));
  if (bytes_written != 1)
  {
    return false;
  }

  const int bytes_read = i2c_read_blocking_until(I2C_BUS, dev_addr, output, len, false, delayed_by_ms(get_absolute_time(), 100));
  if (bytes_read != len)
  {
    return false;
  }

  return true;
}