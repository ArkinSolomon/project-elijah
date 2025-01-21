#include "payload_data_manager.h"

#include <cstring>
#include <ff.h>
#include <f_util.h>
#include <sd_card.h>

#include "main.h"
#include "pin_outs.h"
#include "usb_communication.h"
#include "storage/w25q64fv/w25q64fv.h"

payload_data_manager::LaunchData::LaunchData(const std::string& new_launch_name) // NOLINT(*-pro-type-member-init)
{
  strcpy(launch_name, new_launch_name.c_str());
  start_sector = START_SECTOR;
  next_sector = START_SECTOR;
  present_check = LAUNCH_DATA_PRESENT_CHECK;
}

payload_data_manager::DataInstance::DataInstance()
{
  collected_time = 0;
  us_since_last_data = 0;
  events = DataInstanceEvent::NONE;
  packet_seq = UINT32_MAX;

  pressure = -1;
  temperature = -1;
  altitude = -1;
  accel_x = -1;
  accel_y = -1;
  accel_z = -1;

  bat_voltage = -1;
  bat_percent = -1;
}

payload_data_manager::DataInstance::DataInstance(const CollectionData& collection_data, uint64_t us_since_last_data) :
  DataInstance(collection_data, us_since_last_data, DataInstanceEvent::NONE)
{
}

payload_data_manager::DataInstance::DataInstance(const CollectionData& collection_data,
                                                 const uint64_t us_since_last_data,
                                                 const DataInstanceEvent events) : us_since_last_data(
  us_since_last_data), events(events)
{
  tm time_inst = collection_data.time_inst;
  collected_time = mktime(&time_inst);
  packet_seq = curr_seq++;

  pressure = collection_data.pressure;
  temperature = collection_data.temperature;
  altitude = collection_data.altitude;
  accel_x = collection_data.accel_x;
  accel_y = collection_data.accel_y;
  accel_z = collection_data.accel_z;

  bat_voltage = collection_data.bat_voltage;
  bat_percent = collection_data.bat_percent;
}

void payload_data_manager::init_data_manager()
{
  mutex_init(&launch_data_mtx);
  mutex_init(&active_sector_mtx);
}

void payload_data_manager::init_launch_data()
{
  // Expect call from core 1 init, so don't worry about locking mutexes
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

  init_active_sector();
}

void payload_data_manager::init_active_sector()
{
  active_sector_buff = new LaunchSectorData;
}

bool payload_data_manager::new_launch(const std::string& new_launch_name)
{
  mutex_enter_blocking(&active_sector_mtx);
  mutex_enter_blocking(&launch_data_mtx);

  current_launch_data = LaunchData(new_launch_name);
  const bool success = write_current_launch_data();

  delete active_sector_buff;
  active_sector_buff = new LaunchSectorData;

  mutex_exit(&active_sector_mtx);
  mutex_exit(&launch_data_mtx);
  return success;
}

bool payload_data_manager::write_current_launch_data()
{
  FATFS fs;
  FRESULT fr = f_mount(&fs, "0:", 1);
  if (fr != FR_OK)
  {
    usb_communication::send_string(std::format("ERROR: Could not mount filesystem ({})", static_cast<uint8_t>(fr)));
    return false;
  }

  std::string filename = std::format("{}.flight-data", current_launch_data.launch_name);
  FIL fil;
  fr = f_open(&fil, filename.c_str(), FA_OPEN_APPEND | FA_WRITE);
  if (FR_OK != fr && FR_EXIST != fr)
  {
    usb_communication::send_string(std::format("f_open({}) error: {} ({})\n", filename, FRESULT_str(fr),
                                               static_cast<uint8_t>(fr)));
    return false;
  }
  fr = f_lseek(&fil, 0);

  if (FR_OK != fr)
  {
    usb_communication::send_string(std::format("f_lseek error: {} ({})\n", FRESULT_str(fr),
                                               static_cast<uint8_t>(fr)));
    return false;
  }

  uint bytes_written = 0;
  uint8_t data[4096];
  memcpy(data, &current_launch_data, sizeof(LaunchData));
  fr = f_write(&fil, data, 4096, &bytes_written);
  if (FR_OK != fr)
  {
    usb_communication::send_string(
      std::format("f_write failed error: {} {}", FRESULT_str(fr), static_cast<uint8_t>(fr)));
    return false;
  }

  usb_communication::send_string(std::format("Wrote {} bytes, expected {} bytes [LaunchData]", bytes_written,
                                             4096 /*sizeof(LaunchData)*/));

  // Close the file
  fr = f_close(&fil);
  if (FR_OK != fr)
  {
    usb_communication::send_string(std::format("f_close error: {} ({})", FRESULT_str(fr), static_cast<uint8_t>(fr)));
    return false;
  }

  f_unmount("");
  // return w25q64fv::write_sector(METADATA_SECTOR * 4096, reinterpret_cast<uint8_t*>(&current_launch_data),
  //                               sizeof(LaunchData));
  return true;
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
    "Tried reading launch data for launch \"{}\" starting at sector {} (next sector {}) 0x{:08X}",
    std::string(read_data.launch_name), read_data.start_sector, read_data.next_sector, read_data.present_check));
  if (read_data.present_check != LAUNCH_DATA_PRESENT_CHECK)
  {
    usb_communication::send_string(std::format("present check failed {} != {}", read_data.present_check,
                                               LAUNCH_DATA_PRESENT_CHECK));
    return false;
  }

  memcpy(current_launch_data.launch_name, read_data.launch_name, sizeof(LaunchData::launch_name));
  current_launch_data.start_sector = read_data.start_sector;
  current_launch_data.next_sector = read_data.next_sector;

  return true;
}


void payload_data_manager::buffer_data(const DataInstance& data_inst)
{
  mutex_enter_blocking(&active_sector_mtx);
  if (active_sector_buff == nullptr)
  {
    init_active_sector();
  }

  if (active_sector_buff->num_instances >= instances_per_sector)
  {
    if (active_sector_buff->dropped_instances < UINT16_MAX)
    {
      active_sector_buff->dropped_instances++;
      usb_communication::send_string(std::format("Dropped data instance (total {})!",
                                                 active_sector_buff->dropped_instances));
    }
  }
  else
  {
    active_sector_buff->data[active_sector_buff->num_instances] = data_inst;
    active_sector_buff->num_instances++;
  }

  mutex_exit(&active_sector_mtx);
}

bool payload_data_manager::try_write_active_data(const bool only_full_buff)
{
  const bool launch_data_mtx_claimed = mutex_try_enter(&launch_data_mtx, nullptr);
  if (!launch_data_mtx_claimed)
  {
    return false;
  }
  mutex_enter_blocking(&active_sector_mtx);

  if (active_sector_buff == nullptr)
  {
    init_active_sector();
    mutex_exit(&launch_data_mtx);
    mutex_exit(&active_sector_mtx);
    return false;
  }

  if (only_full_buff && active_sector_buff->num_instances < instances_per_sector)
  {
    mutex_exit(&launch_data_mtx);
    mutex_exit(&active_sector_mtx);
    return false;
  }

  const LaunchSectorData* write_data = active_sector_buff;
  init_active_sector();
  mutex_exit(&active_sector_mtx);

  const size_t empty_data_slots = instances_per_sector - write_data->num_instances;
  const size_t write_size = sizeof(LaunchSectorData) + 4 - empty_data_slots * sizeof(DataInstance);
  // uint8_t write_bytes[write_size];
  uint8_t write_bytes[4096];
  *reinterpret_cast<uint32_t*>(&write_bytes) = SECTOR_DATA_PRESENT_CHECK;
  write_bytes[4] = write_data->num_instances - 1;
  *reinterpret_cast<typeof(LaunchSectorData::dropped_instances)*>(write_bytes + 5) = write_data->dropped_instances;
  *reinterpret_cast<typeof(LaunchSectorData::crc)*>(write_bytes + 9) = write_data->crc;
  memcpy(write_bytes + 17, write_data->data, write_size);
  delete write_data;

  // bool write_success = w25q64fv::write_sector(current_launch_data.next_sector * 4096, write_bytes, write_size);

  FATFS fs;
  FRESULT fr = f_mount(&fs, "0:", 1);
  if (fr != FR_OK)
  {
    usb_communication::send_string(std::format("ERROR: Could not mount filesystem [1] ({})", static_cast<uint8_t>(fr)));
    mutex_exit(&launch_data_mtx);
    return false;
  }

  std::string filename = std::format("{}.flight-data", current_launch_data.launch_name);
  FIL fil;
  fr = f_open(&fil, filename.c_str(), FA_OPEN_APPEND | FA_WRITE);
  if (FR_OK != fr && FR_EXIST != fr)
  {
    usb_communication::send_string(std::format("f_open({}) error: {} ({})\n", filename, FRESULT_str(fr),
                                               static_cast<uint8_t>(fr)));
    mutex_exit(&launch_data_mtx);
    return false;
  }
  uint bytes_written = 0;
  fr = f_write(&fil, write_bytes, 4096/*write_size*/, &bytes_written);
  if (FR_OK != fr)
  {
    usb_communication::send_string(
      std::format("f_write failed error: {} {}", FRESULT_str(fr), static_cast<uint8_t>(fr)));
    mutex_exit(&launch_data_mtx);
    return false;
  }

  usb_communication::send_string(std::format("Wrote {} bytes, expected {}", bytes_written, 4096/*write_size*/));

  // Close the file
  fr = f_close(&fil);
  if (FR_OK != fr)
  {
    usb_communication::send_string(std::format("f_close error: {} ({})", FRESULT_str(fr), static_cast<uint8_t>(fr)));
    mutex_exit(&launch_data_mtx);
    return false;
  }

  f_unmount("");

  bool write_success = true;
  current_launch_data.next_sector++;
  if (!write_success)
  {
    mutex_exit(&launch_data_mtx);
    return false;
  }

  write_success = write_current_launch_data();
  mutex_exit(&launch_data_mtx);
  return write_success;
}
