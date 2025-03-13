#pragma once

#include <cstdint>
#include <memory>
#include <pico/critical_section.h>

#include "log_level.h"

namespace elijah_state_framework
{
  void log_serial_message(const std::string& message);

  namespace internal
  {
    inline critical_section_t usb_cs;

    void init_usb_comm();

    void write_to_serial(const uint8_t* write_data, size_t write_len);
    void write_to_serial(const uint8_t* packet_data, size_t packet_len, bool flush);

    std::unique_ptr<uint8_t[]> encode_log_message(const std::string& message,
                                                         LogLevel log_level,
                                                         size_t& encoded_len);

    void encode_time(uint8_t* dest, const tm& time_inst);
    tm decode_time(const uint8_t* encoded_time_inst);
  }
}
