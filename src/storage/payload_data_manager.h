#pragma once
#include <cstdint>
#include <format>
#include <string>
#include <pico/rand.h>

#define METADATA_SECTOR 0
#define START_SECTOR 1

#define LAUNCH_DATA_PRESENT_CHECK 0xA64B39FC

namespace payload_data_manager
{
  struct LaunchData
  {
    char launch_name[64];
    uint32_t start_sector;
    uint32_t next_sector;
    uint32_t present_check;

    explicit LaunchData(const std::string& new_launch_name);
  };

  inline auto current_launch_data = LaunchData(std::format("Unknown launch {:8X}", get_rand_32()));

  void init_launch_data();

  bool new_launch(const std::string& new_launch_name);
  bool write_current_launch_data();
  bool load_stored_launch_data();
}
