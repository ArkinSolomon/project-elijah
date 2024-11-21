#include "w25q64fv.h"

#include <format>
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <pico/time.h>

#include "src/pin_outs.h"
#include "src/status_manager.h"
#include "src/usb_communication.h"

bool w25q64fv::init()
{
  gpio_put(SPI1_CSN_PIN, false);
  constexpr uint8_t manufacturer_id_write_data[4] = {_command_defs::COMMAND_MANUFACTURER_ID, 0x00, 0x00, 0x00};
  int bytes_written = spi_write_blocking(spi1, manufacturer_id_write_data, 4);

  if (bytes_written != 4)
  {
    gpio_put(SPI1_CSN_PIN, true);
    return false;
  }

  uint8_t device_id_read[2];
  int bytes_read = spi_read_blocking(spi1, 0x00, device_id_read, 2);
  gpio_put(SPI1_CSN_PIN, true);
  if (bytes_read != 2)
  {
    return false;
  }

  manufacturer_id = device_id_read[0];
  manufacturer_device_id = device_id_read[1];
  gpio_put(SPI1_CSN_PIN, false);

  bytes_written = spi_write_blocking(spi1, &_command_defs::COMMAND_JEDEC_ID, 1);
  if (bytes_written != 1)
  {
    gpio_put(SPI1_CSN_PIN, true);
    return false;
  }

  uint8_t read_jedec_id[3];
  bytes_read = spi_read_blocking(spi1, 0x00, read_jedec_id, 3);
  gpio_put(SPI1_CSN_PIN, true);
  if (bytes_read != 3)
  {
    return false;
  }

  memory_type = read_jedec_id[1];
  capacity = read_jedec_id[2];

  gpio_put(SPI1_CSN_PIN, false);
  bytes_written = spi_write_blocking(spi1, &_command_defs::COMMAND_READ_UNIQUE_ID, 1);
  if (bytes_written != 1)
  {
    gpio_put(SPI1_CSN_PIN, true);
    return false;
  }

  constexpr uint8_t dummy_clocks[4] = {};
  bytes_written = spi_write_blocking(spi1, dummy_clocks, 4);
  if (bytes_written != 4)
  {
    gpio_put(SPI1_CSN_PIN, true);
    return false;
  }

  const int shorts_read = spi_read16_blocking(spi1, 0x0000, reinterpret_cast<uint16_t*>(&unique_id), 4);
  gpio_put(SPI1_CSN_PIN, true);
  if (shorts_read != 4)
  {
    return false;
  }

  gpio_put(SPI1_CSN_PIN, false);
  bytes_written = spi_write_blocking(spi1, &_command_defs::COMMAND_WRITE_ENABLE, 1);
  gpio_put(SPI1_CSN_PIN, true);
  return bytes_written == 1;
}

void w25q64fv::print_device_info()
{
  usb_communication::send_string(std::format(
    "W25Q64FV initialized! Manufacturer ID: 0x{:02X}\nDevice ID: 0x{:02X}\nMemory Type: 0x{:02X}\nCapacity ID: 0x{:02X}\nUnique ID: 0x{:016X}",
    manufacturer_id,
    manufacturer_device_id, memory_type, capacity, unique_id));
}

bool w25q64fv::write_data(uint32_t page_addr, uint8_t* data, uint16_t len)
{
}

bool w25q64fv::chip_erase()
{
  wait_for_not_busy();

  gpio_put(SPI1_CSN_PIN, false);
  const int bytes_written = spi_write_blocking(spi1, &_command_defs::COMMAND_CHIP_ERASE, 1);
  gpio_put(SPI1_CSN_PIN, true);

  return bytes_written == 1;
}

void w25q64fv::wait_for_not_busy()
{
  gpio_put(SPI1_CSN_PIN, false);
  const int bytes_written = spi_write_blocking(spi1, &_command_defs::COMMAND_READ_STATUS_REG1, 1);
  if (bytes_written != 1)
  {
    usb_communication::send_string("W25Q64FV can not write write status register 1 read command");
    set_fault(status_manager::DEVICE_W25Q64FV, true);
    gpio_put(SPI1_CSN_PIN, true);
    return;
  }

  while (true)
  {
    uint8_t status_reg;
    const int bytes_read = spi_read_blocking(spi1, 0x00, &status_reg, 1);
    if (bytes_read != 1)
    {
      usb_communication::send_string("W25Q64FV can not read status register 1");
      set_fault(status_manager::DEVICE_W25Q64FV, true);
      break;
    }

    set_fault(status_manager::DEVICE_W25Q64FV, false);
    usb_communication::send_string(std::format("{:08b}", status_reg));
    if (!(status_reg & 0x01))
    {
      break;
    }
  }
  gpio_put(SPI1_CSN_PIN, true);
}
