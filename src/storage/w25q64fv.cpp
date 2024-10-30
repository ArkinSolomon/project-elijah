#include "w25q64fv.h"

#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <pico/time.h>

#include "src/pin_outs.h"

bool w25q64fv::init()
{
  gpio_put(SPI0_CSN_PIN, false);
  constexpr uint8_t manufacturer_id_write_data[4] = {_command_defs::COMMAND_MANUFACTURER_ID, 0x00, 0x00, 0x00};
  int bytes_written = spi_write_blocking(spi0, manufacturer_id_write_data, 4);
  if (bytes_written != 4)
  {
    gpio_put(SPI0_CSN_PIN, true);
    return false;
  }

  uint8_t device_id_read[2];
  int bytes_read = spi_read_blocking(spi0, 0x00, device_id_read, 2);
  if (bytes_read != 2)
  {
    gpio_put(SPI0_CSN_PIN, true);
    return false;
  }

  manufacturer_id = device_id_read[0];
  manufacturer_device_id = device_id_read[1];

  gpio_put(SPI0_CSN_PIN, true);
  return true;
  // sleep_ms(3);
  // gpio_put(SPI0_CSN_PIN, false);
  //
  // bytes_written = spi_write_blocking(spi0, &_command_defs::COMMAND_JEDEC_ID, 1);
  // if (bytes_written != 1)
  // {
  //   gpio_put(SPI0_CSN_PIN, true);
  //   return false;
  // }
  //
  // uint8_t read_jedec_id[3];
  // bytes_read = spi_read_blocking(spi0, 0x00, read_jedec_id, 3);
  // if (bytes_read != 3)
  // {
  //   gpio_put(SPI0_CSN_PIN, true);
  //   return false;
  // }
  //
  // memory_type = read_jedec_id[1];
  // capacity = read_jedec_id[2];
  //
  // bytes_written = spi_write_blocking(spi0, &_command_defs::COMMAND_READ_UNIQUE_ID, 1);
  // if (bytes_written != 1)
  // {
  //   gpio_put(SPI0_CSN_PIN, true);
  //   return false;
  // }
  //
  // constexpr uint8_t dummy_clocks[4] = {};
  // bytes_written = spi_write_blocking(spi0, dummy_clocks, 4);
  // if (bytes_written != 4)
  // {
  //   gpio_put(SPI0_CSN_PIN, true);
  //   return false;
  // }
  //
  // const int shorts_read = spi_read16_blocking(spi0, 0x0000, reinterpret_cast<uint16_t*>(&unique_id), 4);
  // if (shorts_read != 4)
  // {
  //   gpio_put(SPI0_CSN_PIN, true);
  //   return false;
  // }
  //
  // bytes_written = spi_write_blocking(spi0, &_command_defs::COMMAND_WRITE_ENABLE, 1);
  // gpio_put(SPI0_CSN_PIN, true);
  // return bytes_written == 1;
}
