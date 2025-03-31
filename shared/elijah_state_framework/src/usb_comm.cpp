#include "usb_comm.h"

#include <cstring>
#include <ctime>
#include <format>
#include <memory>
#include <hardware/gpio.h>
#include <pico/critical_section.h>
#include <pico/stdio.h>
#include <pico/stdio_usb.h>

#include "output_packet.h"

void elijah_state_framework::init_usb_comm()
{
  static bool did_init = false;
  if (did_init)
  {
    return;
  }

  if (!critical_section_is_initialized(&internal::usb_cs))
  {
    critical_section_init(&internal::usb_cs);
  }
  did_init = stdio_usb_init();
}

void elijah_state_framework::log_serial_message(const std::string& message)
{
  if (!stdio_usb_connected())
  {
    return;
  }

  size_t encoded_len;
  const std::unique_ptr<uint8_t[]> encoded_message = internal::encode_log_message(
   message, LogLevel::SerialOnly, encoded_len);

  critical_section_enter_blocking(&internal::usb_cs);
  internal::write_to_serial(encoded_message.get(), encoded_len);
  critical_section_exit(&internal::usb_cs);
}

void elijah_state_framework::internal::write_to_serial(
  const uint8_t* write_data, const size_t write_len)
{
  write_to_serial(write_data, write_len, true);
}

void elijah_state_framework::internal::write_to_serial(
  const uint8_t* packet_data,
  const size_t packet_len, const bool flush)
{
  if (!stdio_usb_connected())
  {
    return;
  }

  stdio_put_string(reinterpret_cast<const char*>(packet_data), static_cast<int>(packet_len), false, false);
  if (flush)
  {
    stdio_flush();
  }
}

std::unique_ptr<uint8_t[]> elijah_state_framework::internal::encode_log_message(
  const std::string& message, LogLevel log_level, size_t& encoded_len)
{
  std::string send_message = message;

  // TODO, do not hardcode, should be less than write buff len for logger
  if (message.size() > 1000 && log_level != LogLevel::SerialOnly && log_level != LogLevel::Debug)
  {
    send_message = "Message too large... ";
    send_message += message.substr(0, 1000 - send_message.size());
  }

  encoded_len = sizeof(uint8_t) /* packet id */ + sizeof(uint8_t) /* log level */ + sizeof(uint16_t)
    /* message length */ + send_message.size();

  std::unique_ptr<uint8_t[]> encoded_message(new uint8_t[encoded_len]{
    static_cast<uint8_t>(OutputPacket::LogMessage), static_cast<uint8_t>(log_level)
  });

  *reinterpret_cast<uint16_t*>(encoded_message.get() + 2) = static_cast<uint16_t>(send_message.size());
  memcpy(encoded_message.get() + (2 * sizeof(uint8_t) + sizeof(uint16_t)), send_message.c_str(), send_message.size());

  return encoded_message;
}

void elijah_state_framework::internal::encode_time(uint8_t* dest, const tm& time_inst)
{
  dest[0] = time_inst.tm_sec;
  dest[1] = time_inst.tm_min;
  dest[2] = time_inst.tm_hour;
  dest[3] = time_inst.tm_wday;
  dest[4] = time_inst.tm_mday;
  dest[5] = time_inst.tm_mon;

  const uint16_t full_year = time_inst.tm_year + 1980;
  memcpy(dest + 6, &full_year, sizeof(full_year));
}

tm elijah_state_framework::internal::decode_time(const uint8_t* encoded_time_inst)
{
  const uint8_t seconds = encoded_time_inst[0]
                , minutes = encoded_time_inst[1]
                , hours = encoded_time_inst[2]
                , day = encoded_time_inst[3]
                , date = encoded_time_inst[4]
                , month = encoded_time_inst[5];
  const int year = *reinterpret_cast<const uint16_t*>(encoded_time_inst + 6);

  return tm{
    seconds, minutes, hours, date, month, year - 1900, day
  };
}
