#include "i2c_util.h"
#include "../../pin_outs.h"
#include <format>
#include <sys/unistd.h>

#include "src/usb_communication.h"

void i2c_util::i2c_bus_init(i2c_inst_t* i2c, const uint sda_pin, const uint scl_pin, const uint baud_rate)
{
  // i2c@400kHz
  i2c_init(i2c, baud_rate);

  (i2c == I2C_BUS0 ? baud_rate_bus0 : baud_rate_bus1) = baud_rate;

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
  i2c_bus_init(i2c, sda_pin, scl_pin, i2c == I2C_BUS0 ? baud_rate_bus0 : baud_rate_bus1);
}


bool i2c_util::read_byte(i2c_inst_t* i2c, const uint8_t dev_addr, const uint8_t reg_addr, int8_t& output)
{
  uint8_t read_byte;
  const bool success = read_ubyte(i2c, dev_addr, reg_addr, read_byte);
  output = static_cast<int8_t>(read_byte);
  return success;
}

bool i2c_util::read_ubyte(i2c_inst_t* i2c, const uint8_t dev_addr, const uint8_t reg_addr, uint8_t& output)
{
  return read_bytes(i2c, dev_addr, reg_addr, &output, 1);
}

bool i2c_util::read_short(i2c_inst_t* i2c, const uint8_t dev_addr, const uint8_t reg_addr, int16_t& output)
{
  uint16_t read_value;
  const bool success = read_ushort(i2c, dev_addr, reg_addr, read_value);
  if (!success)
  {
    return false;
  }

  output = static_cast<int16_t>(read_value);
  return true;
}

bool i2c_util::read_ushort(i2c_inst_t* i2c, const uint8_t dev_addr, const uint8_t reg_addr, uint16_t& output)
{
  uint8_t read_data[2];
  const bool success = read_bytes(i2c, dev_addr, reg_addr, read_data, 2);
  if (!success)
  {
    return false;
  }

  output = read_data[0] << 8 | read_data[1];
  return success;
}

bool i2c_util::read_short_reversed(i2c_inst_t* i2c, const uint8_t dev_addr, const uint8_t reg_addr, int16_t& output)
{
  uint16_t read_value;
  const bool success = read_ushort_reversed(i2c, dev_addr, reg_addr, read_value);
  if (!success)
  {
    return false;
  }

  output = static_cast<int16_t>(read_value);
  return true;
}

bool i2c_util::read_ushort_reversed(i2c_inst_t* i2c, const uint8_t dev_addr, const uint8_t reg_addr, uint16_t& output)
{
  uint8_t read_data[2];
  const bool success = read_bytes(i2c, dev_addr, reg_addr, read_data, 2);
  if (!success)
  {
    return false;
  }

  output = read_data[1] << 8 | read_data[0];
  return success;
}

/**
 * Read len bytes into output[].
 *
 * Returns false on failure, or true otherwise
 */
bool i2c_util::read_bytes(i2c_inst_t* i2c, const uint8_t dev_addr, const uint8_t reg_addr, uint8_t output[],
                          const uint8_t len)
{
  const int bytes_written = i2c_write_blocking_until(i2c, dev_addr, &reg_addr, 1, true,
                                                     delayed_by_ms(get_absolute_time(), 100));
  if (bytes_written != 1)
  {
    return false;
  }

  const int bytes_read = i2c_read_blocking_until(I2C_BUS0, dev_addr, output, len, false,
                                                 delayed_by_ms(get_absolute_time(), 100));
  if (bytes_read != len)
  {
    return false;
  }

  return true;
}

// See https://github.com/raspberrypi/pico-examples/blob/master/i2c/bus_scan/bus_scan.c
void i2c_util::scan_for_devices(i2c_inst_t* i2c)
{
  const uint8_t bus_num = i2c == I2C_BUS0 ? 0 : 1;
  uint8_t found_count = 0;
  for (int addr = 0; addr < 1 << 7; ++addr)
  {
    uint8_t data;
    if ((addr & 0x78) == 0 || (addr & 0x78) == 0x78)
      continue;

    const int read_bytes = i2c_read_blocking_until(i2c, addr, &data, 1, false, delayed_by_ms(get_absolute_time(), 5));
    if (read_bytes == 1)
    {
      found_count++;
      usb_communication::send_string(std::format("Found I2C device at 0x{:02x} on bus {}", addr, bus_num));
    }
  }

  if (found_count == 0)
  {
    usb_communication::send_string(std::format("Scan complete, no I2C devices found on bus {}", bus_num));
  }
  else
  {
    usb_communication::send_string(std::format("Scan complete! Found {} I2C devices on bus {}", found_count, bus_num));
  }
}
