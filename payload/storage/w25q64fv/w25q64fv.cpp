#include <cstring>
#include <format>
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <sys/unistd.h>

#include "w25q64fv.h"
#include "pin_outs.h"
#include "elijah_state_framework.h"
#include "payload_state_manager.h"
#include "status_manager.h"

#ifndef W25Q64FV_FLASH_SIM

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

  if (manufacturer_id != WINBOND_MANUFACTURER_ID)
  {
    return false;
  }

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
  return shorts_read == 4;
}

bool w25q64fv::write_enable()
{
  gpio_put(SPI1_CSN_PIN, false);
  const int bytes_written = spi_write_blocking(spi1, &_command_defs::COMMAND_WRITE_ENABLE, 1);
  gpio_put(SPI1_CSN_PIN, true);
  if (bytes_written != 1)
  {
    PayloadStateManager::log_message("Fault: W25Q64FV, failed to enable writing");
    set_fault(status_manager::DEVICE_W25Q64FV, true);
    return false;
  }

  if (!wait_for_wel(true))
  {
    return false;
  }

  set_fault(status_manager::DEVICE_W25Q64FV, false);
  return true;
}

bool w25q64fv::write_disable()
{
  gpio_put(SPI1_CSN_PIN, false);
  const int bytes_written = spi_write_blocking(spi1, &_command_defs::COMMAND_WRITE_DISABLE, 1);
  gpio_put(SPI1_CSN_PIN, true);
  if (bytes_written != 1)
  {
    PayloadStateManager::log_message("Fault: W25Q64FV, failed to disable writing");
    set_fault(status_manager::DEVICE_W25Q64FV, true);
    return false;
  }

  if (!wait_for_wel(true))
  {
    return false;
  }

  set_fault(status_manager::DEVICE_W25Q64FV, false);
  return true;
}

bool w25q64fv::write_sector(const uint32_t sector_addr, const uint8_t* data, const uint16_t data_len)
{
  if (sector_addr > LARGEST_SECTOR_ADDR)
  {
    PayloadStateManager::log_message(std::format("Sector address too large, can not write to 0x{:06X}", sector_addr));
    return false;
  }

  wait_for_not_busy();
  if (!write_enable())
  {
    return false;
  }

  gpio_put(SPI1_CSN_PIN, false);
  const uint8_t erase_data[4] = {
    _command_defs::COMMAND_SECTOR_ERASE, static_cast<uint8_t>(sector_addr >> 16 & 0xFF),
    static_cast<uint8_t>(sector_addr >> 8 & 0xFF), static_cast<uint8_t>(sector_addr & 0xFF)
  };
  const int bytes_written = spi_write_blocking(spi1, erase_data, 4);
  gpio_put(SPI1_CSN_PIN, true);
  if (bytes_written != 4)
  {
    PayloadStateManager::log_message(std::format("Fault: W25Q64FV, failed to erase sector 0x{:012X}", sector_addr));
    set_fault(status_manager::DEVICE_W25Q64FV, true);
    return false;
  }

  PayloadStateManager::log_message(std::format("Sector {:012X} erased", sector_addr));

  size_t remaining_len = data_len;
  size_t offset = 0;
  while (remaining_len > 0)
  {
    size_t write_size = 0;
    if (remaining_len >= 256)
    {
      remaining_len -= 256;
      write_size = 256;
    }
    else
    {
      write_size = remaining_len;
      remaining_len = 0;
    }

    const bool success = page_program(sector_addr + offset, data + offset, write_size);
    if (!success)
    {
      return false;
    }
    offset += write_size;
  }

  set_fault(status_manager::DEVICE_W25Q64FV, false);
  return true;
}

bool w25q64fv::page_program(const uint32_t page_addr, const uint8_t* data, const size_t data_len)
{
  const size_t write_len = 4 + data_len;
  uint8_t write_buff[write_len];
  write_buff[0] = _command_defs::COMMAND_PAGE_PROGRAM;
  write_buff[1] = static_cast<uint8_t>(page_addr >> 16 & 0xFF);
  write_buff[2] = static_cast<uint8_t>(page_addr >> 8 & 0xFF);
  write_buff[3] = static_cast<uint8_t>(page_addr & 0xFF);
  memcpy(write_buff + 4, data, data_len);

  wait_for_not_busy();
  if (!write_enable())
  {
    return false;
  }

  std::string write_data;
  for (int i = 0; i < write_len; i++)
  {
    write_data += std::format("{:02X}", write_buff[i]);
  }

  // PayloadStateManager::log_message(std::format("writing page {:012X} {} bytes ({} byte command), cmd: 0x{}", page_addr, data_len, write_len, write_data));
  gpio_put(SPI1_CSN_PIN, false);
  const int bytes_written = spi_write_blocking(spi1, write_buff, write_len);
  gpio_put(SPI1_CSN_PIN, true);

  if (bytes_written != write_len)
  {
    PayloadStateManager::log_message(std::format("Fault: W25Q64FV, failed to program page 0x{:012X} with {} bytes",
                                               page_addr, data_len));
    set_fault(status_manager::DEVICE_W25Q64FV, true);
    return false;
  }

  set_fault(status_manager::DEVICE_W25Q64FV, false);
  return true;
}

bool w25q64fv::read_data(const uint32_t data_addr, uint8_t* data_buff, const size_t data_len)
{
  const uint32_t end_addr = data_addr + data_len;
  if (end_addr > LARGEST_READABLE_ADDR)
  {
    PayloadStateManager::log_message(std::format("Can not read {} bytes from 0x{:06X}, would overflow flash", data_len,
                                               data_addr));
    return false;
  }

  wait_for_not_busy();

  const uint8_t read_command[5] = {
    _command_defs::COMMAND_FAST_READ, static_cast<uint8_t>(data_addr >> 16 & 0xFF),
    static_cast<uint8_t>(data_addr >> 8 & 0xFF), static_cast<uint8_t>(data_addr & 0xFF), 0x00
  };

  gpio_put(SPI1_CSN_PIN, false);
  const int bytes_written = spi_write_blocking(spi1, read_command, 5);
  PayloadStateManager::log_message(std::format("sent read data command to read {} bytes from 0x{:012X}", data_len,
                                             data_addr));

  if (bytes_written != 5)
  {
    gpio_put(SPI1_CSN_PIN, true);
    PayloadStateManager::log_message(std::format(
      "Fault: W25Q64FV, failed to send read data command to read {} bytes from 0x{:012X}", data_len, data_addr));
    set_fault(status_manager::DEVICE_W25Q64FV, true);
    return false;
  }

  const int bytes_read = spi_read_blocking(spi1, 0x00, data_buff, data_len);
  gpio_put(SPI1_CSN_PIN, true);
  if (bytes_read != data_len)
  {
    PayloadStateManager::log_message(std::format(
      "Fault: W25Q64FV, failed to read {} bytes from 0x{:012X}, only read {} bytes", data_len, data_addr, bytes_read));
    set_fault(status_manager::DEVICE_W25Q64FV, true);
    return false;
  }
  return true;
}

bool w25q64fv::chip_erase()
{
  wait_for_not_busy();

  gpio_put(SPI1_CSN_PIN, false);
  const int bytes_written = spi_write_blocking(spi1, &_command_defs::COMMAND_CHIP_ERASE, 1);
  gpio_put(SPI1_CSN_PIN, true);

  if (bytes_written != 1)
  {
    PayloadStateManager::log_message("Fault: W25Q64FV, failed to send chip erase command");
    set_fault(status_manager::DEVICE_W25Q64FV, true);
    return false;
  }

  set_fault(status_manager::DEVICE_W25Q64FV, false);
  return true;
}

/**
 * Read status register one by pulling CSN low then leaving it low to read continuously.
 *
 * @return True if the read command was sent successfully.
 */
bool w25q64fv::open_status_reg_1()
{
  gpio_put(SPI1_CSN_PIN, false);
  const int bytes_written = spi_write_blocking(spi1, &_command_defs::COMMAND_READ_STATUS_REG1, 1);
  if (bytes_written != 1)
  {
    PayloadStateManager::log_message("Fault: W25Q64FV can not write write status register 1 read command");
    set_fault(status_manager::DEVICE_W25Q64FV, true);
    gpio_put(SPI1_CSN_PIN, true);
    return false;
  }

  return true;
}

bool w25q64fv::wait_for_wel(const bool can_write)
{
  if (!open_status_reg_1())
  {
    return false;
  }

  while (true)
  {
    uint8_t status_reg;
    const int bytes_read = spi_read_blocking(spi1, 0x00, &status_reg, 1);
    if (bytes_read != 1)
    {
      PayloadStateManager::log_message("Fault: W25Q64FV can not read status register 1");
      set_fault(status_manager::DEVICE_W25Q64FV, true);
      gpio_put(SPI1_CSN_PIN, true);
      return false;
    }

    set_fault(status_manager::DEVICE_W25Q64FV, false);

    const bool wel_en = (status_reg & 0x02) > 0;
    if (wel_en == can_write && !(status_reg & 0x01))
    {
      gpio_put(SPI1_CSN_PIN, true);
      return true;
    }
  }
}


bool w25q64fv::wait_for_not_busy()
{
  if (!open_status_reg_1())
  {
    return false;
  }

  while (true)
  {
    uint8_t status_reg;
    const int bytes_read = spi_read_blocking(spi1, 0x00, &status_reg, 1);
    if (bytes_read != 1)
    {
      PayloadStateManager::log_message("Fault: W25Q64FV can not read status register 1");
      set_fault(status_manager::DEVICE_W25Q64FV, true);
      gpio_put(SPI1_CSN_PIN, true);
      return false;
    }

    set_fault(status_manager::DEVICE_W25Q64FV, false);

    if (!(status_reg & 0x01))
    {
      gpio_put(SPI1_CSN_PIN, true);
      return true;
    }
  }
}

#endif

void w25q64fv::print_device_info()
{
  PayloadStateManager::log_message(std::format(
    "W25Q64FV initialized!\nManufacturer ID: 0x{:02X}\nDevice ID: 0x{:02X}\nMemory Type: 0x{:02X}\nCapacity ID: 0x{:02X}\nUnique ID: 0x{:016X}",
    manufacturer_id,
    manufacturer_device_id, memory_type, capacity, unique_id));
}
