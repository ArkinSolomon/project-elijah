#pragma once
#include <cstdint>
#include <map>
#include <memory>
#include <pico/critical_section.h>

#include "storage/payload_data_manager/payload_data_manager.h"

#define MAX_RAW_STR_WRITE_BYTES 128

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
    SET_BARO_PRESS = 0x0D,
    DS_1307_REG_DUMP = 0x0E,
    DS_1307_ERASE = 0x0F,
    BARO_PRESS_ACK_SUCCESS = 0x10,
    BARO_PRESS_ACK_FAIL = 0x11,
    GET_BUILD_INFO = 0x12,
    MPU_6050_ST = 0x13,
    W25Q64FV_DEV_INFO = 0x14,
    FAULT_DATA = 0x15,
    RESTART = 0x16,
    NEW_LAUNCH = 0x17,
    LAUNCH_DATA = 0x18,
    FLUSH_TO_SD_CARD = 0x19
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
      COLLECTION_DATA, 68
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
      LOOP_TIME, 24
    },
    {
      I2C_SCAN_0, 0
    },
    {
      I2C_SCAN_1, 0
    },
    {
      SET_BARO_PRESS, 8
    },
    {
      DS_1307_REG_DUMP, 0
    },
    {
      DS_1307_ERASE, 0
    },
    {
      BARO_PRESS_ACK_SUCCESS, 0
    },
    {
      BARO_PRESS_ACK_FAIL, 0
    },
    {
      GET_BUILD_INFO, 0
    },
    {
      MPU_6050_ST, 0
    },
    {
      W25Q64FV_DEV_INFO, 0
    },
    {
      FAULT_DATA, 6
    },
    {
      RESTART, 0
    },
    {
      NEW_LAUNCH, MAX_LAUNCH_NAME_SIZE
    },
    {
      LAUNCH_DATA, ENCODED_LAUNCH_DATA_SIZE
    },
     {
     FLUSH_TO_SD_CARD, 0}
  };

  inline critical_section_t usb_cs;

  void init_usb_com();
  void scan_for_packets();

  void send_packet(packet_type_id type_id);
  void send_packet(packet_type_id type_id, const uint8_t* packet_data);
  void send_string(const std::string& str);
  void say_hello();
  void send_collection_data(const CollectionData& collection_data);

  void handle_usb_packet(packet_type_id packet_type_id, const uint8_t* packet_data);
  void write_packet(const uint8_t* packet_data, size_t packet_len);
}
