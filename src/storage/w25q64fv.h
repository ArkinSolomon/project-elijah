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
  }

  inline uint8_t manufacturer_id, manufacturer_device_id, memory_type, capacity;
  inline uint64_t unique_id;

  bool init();
}
