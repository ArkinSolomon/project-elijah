#pragma once
#include <cstdint>
#include <functional>
#include <map>
#include <bits/basic_string.h>
#include <pico/mutex.h>

struct CollectionData;

namespace usb_communication
{
  enum packet_type_id
  {
    TIME_SET = 0x01,
    TIME_SET_SUCCESS = 0x02,
    TIME_SET_FAIL = 0x03,
    COLLECTION_DATA = 0x04,
    STRING = 0x05,
    REQ_CALIBRATION_DATA = 0x06,
    CALIBRATION_DATA_BMP_180 = 0x07,
    HELLO = 0x08,
    CALIBRATION_DATA_BMP_280 = 0x09,
    LOOP_TIME = 0x0A,
    I2C_SCAN_0 = 0x0B,
    I2C_SCAN_1 = 0x0C,
    SET_SEA_LEVEL_PRESS = 0x0D,
    DS_1307_REG_DUMP = 0x0E,
    DS_1307_ERASE = 0x0F,
    SEA_LEVEL_PRESS_ACK_SUCCESS = 0x10,
    SEA_LEVEL_PRESS_ACK_FAIL = 0x11,
    GET_BUILD_INFO = 0x12,
    MPU_6050_ST = 0x13
  };

  inline const std::map<packet_type_id, uint8_t> packet_type_lens = {
    {
      // (MSB->LSB) 8-bit seconds, 8-bit minutes, 8-bit hours, 8-bit day-of-week, 8-bit date, 8-bit month, 16-bit year
      TIME_SET, 8

    },
    {
      TIME_SET_SUCCESS, 0
    },
    {
      TIME_SET_FAIL, 0
    },
    {
      COLLECTION_DATA, 28
    },
    {
      STRING, 0,
    },
    {
      REQ_CALIBRATION_DATA, 0
    },
    {
      CALIBRATION_DATA_BMP_180, 23
    },
    {
      HELLO, 0
    },
    {
      CALIBRATION_DATA_BMP_280, 34
    },
    {
      LOOP_TIME, 4
    },
    {
      I2C_SCAN_0, 0
    },
    {
      I2C_SCAN_1, 0
    },
    {
      SET_SEA_LEVEL_PRESS, 8
    },
    {
      DS_1307_REG_DUMP, 0
    },
    {
      DS_1307_ERASE, 0
    },
    {
      SEA_LEVEL_PRESS_ACK_SUCCESS, 0
    },
    {
      SEA_LEVEL_PRESS_ACK_FAIL, 0
    },
    {
      GET_BUILD_INFO, 0
    },
    {
      MPU_6050_ST, 0
    }
  };

  inline mutex_t usb_comm_mtx;

  void init_usb_com();

  void scan_for_packets();
  void send_packet(packet_type_id type_id);
  void send_packet(packet_type_id type_id, const uint8_t* packet_data);
  void send_string(const std::string& str);

  void handle_usb_packet(packet_type_id packet_type_id, const uint8_t* packet_data);
  void send_collection_data(const CollectionData& collection_data);
  void say_hello();
}
