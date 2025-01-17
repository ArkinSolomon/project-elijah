#include "usb_communication.h"

#include <cstdio>
#include <cstring>
#include <format>
#include <string>
#include <hardware/watchdog.h>
#include <pico/critical_section.h>
#include <pico/stdio.h>
#include <pico/time.h>

#include "byte_util.h"
#include "cs_lock_num.h"
#include "main.h"
#include "pin_outs.h"
#include "status_manager.h"
#include "sensors/bmp_280/bmp_280.h"
#include "sensors/ds_1307/ds_1307.h"
#include "sensors/i2c/i2c_util.h"
#include "sensors/mpu_6050/mpu_6050.h"
#include "storage/w25q64fv.h"

void usb_communication::init_usb_com()
{
  stdio_init_all();
  critical_section_init_with_lock_num(&usb_cs, CS_LOCK_NUM_USB);
}

void usb_communication::scan_for_packets()
{
  char packet_type = 0xFF;
  int num_read = stdio_get_until(&packet_type, 1, delayed_by_ms(get_absolute_time(), 30));
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
  if (!stdio_usb_connected())
  {
    return;
  }

  const uint8_t write_len = packet_type_lens.at(type_id);
  const int total_len = write_len + 1;

  uint8_t write_data[total_len];
  write_data[0] = type_id;

  if (write_len > 0)
  {
    memcpy(write_data + 1, packet_data, write_len);
  }

  write_packet(write_data, total_len);
}

void usb_communication::send_string(const std::string& str)
{
  if (!stdio_usb_connected())
  {
    return;
  }

  const uint16_t str_len = str.size();
  const uint16_t total_len = str_len + 3;

  uint8_t write_data[total_len];
  for (int i = 0; i < total_len; i++)
  {
    write_data[i] = 'A';
  }

  write_data[0] = STRING;
  write_data[1] = str_len >> 8;
  write_data[2] = str_len & 0xFF;

  if (str_len > 0)
  {
    memcpy(write_data + 3, str.c_str(), str_len);
  }

  write_packet(write_data, total_len);
}

void usb_communication::say_hello()
{
  send_string("Hello, computer \u263a");
}

void usb_communication::send_collection_data(const CollectionData& collection_data)
{
  if (!stdio_usb_connected())
  {
    return;
  }

  uint8_t serialized_data[52];

  serialized_data[0] = collection_data.time_inst.seconds;
  serialized_data[1] = collection_data.time_inst.minutes;
  serialized_data[2] = collection_data.time_inst.hours;
  serialized_data[3] = collection_data.time_inst.day;
  serialized_data[4] = collection_data.time_inst.date;
  serialized_data[5] = collection_data.time_inst.month;

  byte_util::encode_uint16(collection_data.time_inst.year, &serialized_data[6]);
  byte_util::encode_uint32(collection_data.pressure, &serialized_data[8]);

  byte_util::encode_double(collection_data.temperature, &serialized_data[12]);
  byte_util::encode_double(collection_data.altitude, &serialized_data[20]);

  byte_util::encode_double(collection_data.accel_x, &serialized_data[28]);
  byte_util::encode_double(collection_data.accel_y, &serialized_data[36]);
  byte_util::encode_double(collection_data.accel_z, &serialized_data[44]);

  send_packet(COLLECTION_DATA, serialized_data);
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
  case I2C_SCAN_0:
    i2c_util::scan_for_devices(I2C_BUS0);
    break;
  case I2C_SCAN_1:
    i2c_util::scan_for_devices(I2C_BUS1);
    break;
  case SET_SEA_LEVEL_PRESS:
    {
      const double pressure = byte_util::decode_double(packet_data);
      if (bmp_280::update_sea_level_pressure(pressure))
      {
        send_packet(SEA_LEVEL_PRESS_ACK_SUCCESS);
      }
      else
      {
        send_packet(SEA_LEVEL_PRESS_ACK_FAIL);
      }
      break;
    }
  case DS_1307_REG_DUMP:
    ds_1307::reg_dump();
    break;
  case DS_1307_ERASE:
    ds_1307::erase_data();
    break;
  case GET_BUILD_INFO:
    send_string(std::format("Elijah Payload compiled {} at {}", __DATE__, __TIME__));
    break;
  case MPU_6050_ST:
    {
      const bool success = mpu_6050::self_test();
      if (!success)
      {
        send_string("MPU 6050 self-test failed to execute");
        break;
      }

      send_string(std::format(
        "Change from factory trim (x, y, z): {:.7}%, {:.7}%, {:.7}%",
        mpu_6050::mpu_6050_factory_trim_data.ft_xa_change_percent,
        mpu_6050::mpu_6050_factory_trim_data.ft_ya_change_percent,
        mpu_6050::mpu_6050_factory_trim_data.ft_za_change_percent
      ));
      break;
    }
  case W25Q64FV_DEV_INFO:
    w25q64fv::print_device_info();
    break;
  case RESTART:
    send_string("Restarting...");
    watchdog_enable(100, false);

  // ReSharper disable once CppPossiblyErroneousEmptyStatements CppDFAEndlessLoop
    while (true);
  default: ;
  }

  delete [] packet_data;
}

void usb_communication::write_packet(const uint8_t* packet_data, const size_t packet_len)
{
  if (!stdio_usb_connected())
  {
    return;
  }

  critical_section_enter_blocking(&usb_cs);
  size_t remaining_len = packet_len;
  size_t offset = 0;
  while (remaining_len > 0)
  {
    size_t write_size = 0;
    if (remaining_len >= MAX_RAW_STR_WRITE_BYTES)
    {
      remaining_len -= MAX_RAW_STR_WRITE_BYTES;
      write_size = MAX_RAW_STR_WRITE_BYTES;
    }
    else
    {
      write_size = remaining_len;
      remaining_len = 0;
    }

    stdio_put_string(reinterpret_cast<const char*>(packet_data + offset), static_cast<int>(write_size), false, false);
    stdio_flush();
    offset += write_size;
  }

  critical_section_exit(&usb_cs);
}
