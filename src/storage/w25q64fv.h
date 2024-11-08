#pragma once
#include <cstdint>

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
  }

  inline uint8_t manufacturer_id, manufacturer_device_id, memory_type, capacity;
  inline uint64_t unique_id;

  bool init();
  void print_device_info();
  bool write_data(uint32_t page_addr, uint8_t* data, uint16_t len);
  bool chip_erase();
  bool is_busy(bool& is_flash_busy);
  void wait_for_not_busy();
}
