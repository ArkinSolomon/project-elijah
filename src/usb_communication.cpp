#include "usb_communication.h"

#include <cstdio>
#include <cstring>
#include <format>
#include <string>
#include <pico/stdio.h>
#include <pico/time.h>

#include "main.h"
#include "status_manager.h"
#include "sensors/bmp_180/bmp_180.h"
#include "sensors/bmp_280/bmp_280.h"
#include "sensors/ds_1307/ds_1307.h"

void usb_communication::init_usb_com()
{
  stdio_init_all();
}

void usb_communication::scan_for_packets()
{
  char packet_type = 0xFF;
  int num_read = stdio_get_until(&packet_type, 1, delayed_by_ms(get_absolute_time(), 100));
  if (num_read != 1)
  {
    return;
  }

  const auto it = packet_type_lens.find(static_cast<packet_type_id>(packet_type));
  if (it == packet_type_lens.end())
  {
    return;
  }
  const uint8_t data_len = it->second;

  const auto packet_data = new char[data_len];
  if (data_len > 0)
  {
    num_read = stdio_get_until(packet_data, data_len, delayed_by_ms(get_absolute_time(), 750));
    if (num_read != data_len)
    {
      return;
    }
  }

  handle_usb_packet(static_cast<packet_type_id>(packet_type), reinterpret_cast<unsigned char*>(packet_data));
}

void usb_communication::send_packet(const packet_type_id type_id)
{
  send_packet(type_id, nullptr);
}

void usb_communication::send_packet(const packet_type_id type_id, const uint8_t packet_data[])
{
  stdio_putchar_raw(type_id);

  const uint8_t write_len = packet_type_lens.at(type_id);
  if (write_len > 0)
  {
    stdio_put_string(reinterpret_cast<const char*>(packet_data), write_len, false, false);
  }
}

void usb_communication::send_string(const std::string& str)
{
  if (!stdio_usb_connected())
  {
    return;
  }

  const uint16_t str_len = str.size();
  const uint8_t meta_data[3] = {
    STRING, static_cast<uint8_t>(str_len >> 8), static_cast<uint8_t>(str_len & 0xFF)
  };

  stdio_put_string(reinterpret_cast<const char*>(meta_data), 3, false, false);
  stdio_put_string(str.c_str(), str_len, false, false);
}

void usb_communication::handle_usb_packet(const packet_type_id packet_type_id, const uint8_t* packet_data)
{
  switch (packet_type_id)
  {
  case TIME_SET:
    ds_1307::handle_time_set_packet(packet_data);
    break;
  case REQ_CALIBRATION_DATA:
    bmp_280::send_calibration_data();
    break;
  case HELLO:
    say_hello();
    break;
  default: ;
  }

  delete [] packet_data;
}

void usb_communication::send_collection_data(const CollectionData& collection_data)
{
  uint8_t serialized_data[28];

  serialized_data[0] = collection_data.time_inst.seconds;
  serialized_data[1] = collection_data.time_inst.minutes;
  serialized_data[2] = collection_data.time_inst.hours;
  serialized_data[3] = collection_data.time_inst.day;
  serialized_data[4] = collection_data.time_inst.date;
  serialized_data[5] = collection_data.time_inst.month;
  serialized_data[6] = collection_data.time_inst.year >> 8;
  serialized_data[7] = collection_data.time_inst.year & 0xFF;

  for (size_t i = 0; i < 4; i++)
  {
    serialized_data[8 + i] = collection_data.pressure >> 32 - 8 * (i + 1) & 0xFF;
  }

  memcpy(&serialized_data[12], &collection_data.temperature, sizeof(collection_data.temperature));
  memcpy(&serialized_data[20], &collection_data.altitude, sizeof(collection_data.altitude));

  send_packet(COLLECTION_DATA, serialized_data);
}

void usb_communication::say_hello()
{
  send_string("Hello, computer \u263a");
}
