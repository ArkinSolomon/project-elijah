#include "payload_data_manager.h"

#include <cstring>

#include "w25q64fv.h"
#include "src/status_manager.h"
#include "src/usb_communication.h"

payload_data_manager::LaunchData::LaunchData(const std::string& new_launch_name) // NOLINT(*-pro-type-member-init)
{
  strcpy(launch_name, new_launch_name.c_str());
  start_sector = START_SECTOR;
  next_sector = START_SECTOR;
  present_check = LAUNCH_DATA_PRESENT_CHECK;
}

void payload_data_manager::init_launch_data()
{
  const bool did_load_launch_data = load_stored_launch_data();
  if (!did_load_launch_data)
  {
    usb_communication::send_string(std::format(
      "Writing launch data for launch \"{}\" starting at sector {} (next sector {})",
      std::string(current_launch_data.launch_name), current_launch_data.start_sector, current_launch_data.next_sector));

    write_current_launch_data();
  }
  else
  {
    usb_communication::send_string(std::format(
      "Loaded launch data for launch \"{}\" starting at sector {} (next sector {})",
      std::string(current_launch_data.launch_name), current_launch_data.start_sector,
      current_launch_data.next_sector));
  }
}

bool payload_data_manager::new_launch(const std::string& new_launch_name)
{
  current_launch_data = LaunchData(new_launch_name);
  return write_current_launch_data();
}

bool payload_data_manager::write_current_launch_data()
{
  return w25q64fv::write_sector(METADATA_SECTOR * 4096, reinterpret_cast<uint8_t*>(&current_launch_data),
                                sizeof(LaunchData));
}

bool payload_data_manager::load_stored_launch_data()
{
  auto read_data = LaunchData("tmp");
  const bool success =
    w25q64fv::read_data(METADATA_SECTOR * 4096, reinterpret_cast<uint8_t*>(&read_data), sizeof(LaunchData));
  if (!success)
  {
    return false;
  }

  usb_communication::send_string(std::format(
  "tried reading launch data for launch \"{}\" starting at sector {} (next sector {}) 0x{:08X}",
  std::string(read_data.launch_name), read_data.start_sector, read_data.next_sector, read_data.present_check));
  if (read_data.present_check != LAUNCH_DATA_PRESENT_CHECK)
  {
    usb_communication::send_string(std::format("present check failed {} != {}", read_data.present_check, LAUNCH_DATA_PRESENT_CHECK));
    return false;
  }

  memcpy(current_launch_data.launch_name, read_data.launch_name, sizeof(LaunchData::launch_name));
  current_launch_data.start_sector = read_data.start_sector;
  current_launch_data.next_sector = read_data.next_sector;

  return true;
}
