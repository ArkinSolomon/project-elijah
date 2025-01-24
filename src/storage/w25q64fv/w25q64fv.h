#pragma once

#define WINBOND_MANUFACTURER_ID 0xEF

#define SECTOR_SIZE 4096

#include <cstdint>
#include <cstddef>

namespace w25q64fv
{
  namespace _command_defs
  {
    constexpr uint8_t COMMAND_WRITE_ENABLE = 0x06;
    constexpr uint8_t COMMAND_WRITE_DISABLE = 0x04;

    constexpr uint8_t COMMAND_MANUFACTURER_ID = 0x90;
    constexpr uint8_t COMMAND_JEDEC_ID = 0x9F;
    constexpr uint8_t COMMAND_READ_UNIQUE_ID = 0x4B;

    constexpr uint8_t COMMAND_READ_STATUS_REG1 = 0x05;
    constexpr uint8_t COMMAND_READ_STATUS_REG2 = 0x35;

    constexpr uint8_t COMMAND_PAGE_PROGRAM = 0x4E;
    constexpr uint8_t COMMAND_CHIP_ERASE = 0xC7;
    constexpr uint8_t COMMAND_SECTOR_ERASE = 0x20;

    constexpr uint8_t COMMAND_READ_DATA = 0x03;
    constexpr uint8_t COMMAND_FAST_READ = 0x0B;
  }

  inline uint8_t manufacturer_id, manufacturer_device_id, memory_type, capacity;
  inline uint64_t unique_id;

  bool init();
  void print_device_info();

  bool write_enable();
  bool write_disable();
  bool write_sector(uint32_t sector_addr, const uint8_t* data, uint16_t data_len);
  bool page_program(uint32_t page_addr, const uint8_t* data, size_t data_len);

  bool read_data(uint32_t data_addr, uint8_t* data_buff, size_t data_len);

  bool chip_erase();

  bool open_status_reg_1();

  bool wait_for_wel(bool can_write);
  bool wait_for_not_busy();
}
