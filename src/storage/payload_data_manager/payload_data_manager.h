#pragma once
#include <cmath>
#include <cstdint>
#include <format>
#include <string>
#include <pico/mutex.h>
#include <pico/rand.h>

#include "storage/w25q64fv/w25q64fv.h"

#define METADATA_SECTOR 1
#define START_SECTOR 2

#define DATA_BUFF_SIZE 128

#define LAUNCH_DATA_PRESENT_CHECK 0xA64B39FC
#define SECTOR_DATA_PRESENT_CHECK 0xC8A7E312

#define LAUNCH_DATA_SIZE 76
#define ENCODED_DATA_INSTANCE_SIZE 81

struct CollectionData;

namespace payload_data_manager
{
  struct LaunchData
  {
    char launch_name[64];
    uint32_t start_sector;
    uint32_t next_sector;

    explicit LaunchData(const std::string& new_launch_name);
  };

  enum class DataInstanceEvent : uint8_t
  {
    NONE = 0x00,
    LAUNCH = 0x01,
    LANDING = 0x02,
    _neg_press = 0x80
  };

  constexpr DataInstanceEvent operator|(DataInstanceEvent a, DataInstanceEvent b)
  {
    return static_cast<DataInstanceEvent>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
  }

  inline uint32_t curr_seq = 0;

  struct DataInstance
  {
    time_t collected_time;

    // If this ever decreases, we know there was a power loss
    uint32_t packet_seq;
    uint64_t us_since_last_data;

    int32_t pressure;
    double temperature;
    double altitude;
    double accel_x, accel_y, accel_z;
    double bat_voltage, bat_percent;

    DataInstanceEvent events;

    DataInstance();
    DataInstance(const CollectionData& collection_data, uint64_t us_since_last_data);
    DataInstance(const CollectionData& collection_data, uint64_t us_since_last_data, DataInstanceEvent events);
  };

  constexpr uint8_t instances_per_sector = std::floor(
    (SECTOR_SIZE - 4 /* (bytes) present check (not shown) */ - 1 /* num_instances */ - 2 /* for dropped packets */
      - 8  /* For CRC */) / ENCODED_DATA_INSTANCE_SIZE);

  struct SectorData
  {
    uint8_t num_instances = 0;
    uint16_t dropped_instances = 0;
    uint64_t crc = 0;
    DataInstance data[instances_per_sector];
  };

  inline mutex_t launch_data_mtx;
  inline mutex_t active_sector_mtx;

  inline SectorData* active_sector_buff = nullptr;

  inline auto current_launch_data = LaunchData(std::format("Unknown launch {:016x}", get_rand_64()));

  void init_data_manager();
  void init_launch_data();
  void init_active_sector();

  void encode_launch_data(const LaunchData& launch_data, uint8_t* data_buff);
  void encode_data_instance(const DataInstance& data_inst,uint8_t* sector_buff);
  void encode_sector_data(const SectorData& sector_data, uint8_t* sector_buff);

  bool new_launch(const std::string& new_launch_name);
  bool write_current_launch_data();
  bool load_stored_launch_data();

  void buffer_data(const DataInstance& data_inst);
  bool try_write_active_data(bool only_full_buff = true);
}
