#pragma once

#include <cstdint>
#include <ctime>

#define DS_1307_ADDR 0b1101000

struct CollectionData;

namespace ds_1307
{

  namespace _reg_defs
  {
    constexpr uint8_t REG_SECONDS = 0x00;
    constexpr uint8_t REG_MINUTES = 0x01;
    constexpr uint8_t REG_HOURS = 0x02;
    constexpr uint8_t REG_DAY = 0x03;
    constexpr uint8_t REG_DATE = 0x04;
    constexpr uint8_t REG_MONTH = 0x05;
    constexpr uint8_t REG_YEAR = 0x06;
    constexpr uint8_t REG_CTRL = 0x07;
  }

  enum class custom_register
  {};

  bool check_clock(bool& clock_set);
  bool set_clock(const tm& time_inst);
  bool functional_check(const tm& reset_inst);

  void handle_time_set_packet(const uint8_t* packet_data);
  bool read_clock(tm &time_inst);
  bool check_and_read_clock(tm& time_inst);

  bool read_custom_register(custom_register addr, uint8_t* output, uint8_t size);
  bool write_custom_register(custom_register addr, const uint8_t* data, uint8_t size);
  void reg_dump();
  void erase_data();
}