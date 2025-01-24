#include "payload_data_manager.h"

#include <cstring>
#include <ff.h>
#include <f_util.h>

#include "byte_util.h"
#include "main.h"
#include "pin_outs.h"
#include "status_manager.h"
#include "usb_communication.h"
#include "storage/w25q64fv/w25q64fv.h"

payload_data_manager::LaunchData::LaunchData(const std::string& new_launch_name) // NOLINT(*-pro-type-member-init)
{
  strcpy(launch_name, new_launch_name.c_str());
  start_sector = START_SECTOR;
  next_sector = START_SECTOR;
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
  active_sector_buff = new SectorData;
}

void payload_data_manager::encode_launch_data(const LaunchData& launch_data, uint8_t* data_buff)
{
  memcpy(data_buff, launch_data.launch_name, 64);
  byte_util::encode_uint32(launch_data.start_sector, data_buff + 64);
  byte_util::encode_uint32(launch_data.next_sector, data_buff + 68);
  byte_util::encode_uint32(LAUNCH_DATA_PRESENT_CHECK, data_buff + 72);
}

void payload_data_manager::encode_data_instance(const DataInstance& data_inst, uint8_t* sector_buff)
{
  byte_util::encode_uint64(data_inst.us_since_last_data, sector_buff);
  byte_util::encode_uint32(data_inst.packet_seq, sector_buff + 8);
  byte_util::encode_uint64(data_inst.us_since_last_data, sector_buff + 12);
  // Pressure encoded later

  byte_util::encode_double(data_inst.temperature, sector_buff + 24);
  byte_util::encode_double(data_inst.altitude, sector_buff + 32);
  byte_util::encode_double(data_inst.accel_x, sector_buff + 40);
  byte_util::encode_double(data_inst.accel_y, sector_buff + 48);
  byte_util::encode_double(data_inst.accel_z, sector_buff + 56);
  byte_util::encode_double(data_inst.bat_voltage, sector_buff + 64);
  byte_util::encode_double(data_inst.bat_voltage, sector_buff + 72);

  sector_buff[80] = static_cast<uint8_t>(data_inst.events);
  byte_util::encode_int32(data_inst.pressure, sector_buff + 20, sector_buff[80], 7);
}

void payload_data_manager::encode_sector_data(const SectorData& sector_data, uint8_t* sector_buff)
{
  sector_buff[0] = sector_data.num_instances;
  byte_util::encode_uint16(sector_data.dropped_instances, sector_buff + 1);
  byte_util::encode_uint64(sector_data.crc, sector_buff + 3);


  uint8_t* write_loc = sector_buff + 11;
  for (size_t i = 1; i <= sector_data.num_instances; write_loc += ENCODED_DATA_INSTANCE_SIZE, ++i)
  {
    encode_data_instance(sector_data.data[i], write_loc);
  }

  byte_util::encode_uint32(SECTOR_DATA_PRESENT_CHECK, write_loc);
}

bool payload_data_manager::new_launch(const std::string& new_launch_name)
{
  mutex_enter_blocking(&active_sector_mtx);
  mutex_enter_blocking(&launch_data_mtx);

  current_launch_data = LaunchData(new_launch_name);
  const bool success = write_current_launch_data();

  delete active_sector_buff;
  active_sector_buff = new SectorData;

  mutex_exit(&active_sector_mtx);
  mutex_exit(&launch_data_mtx);
  return success;
}

bool payload_data_manager::write_current_launch_data()
{
  usb_communication::send_string(std::format(
    "Writing launch data for launch (current) \"{}\" starting at sector {} (next sector {})",
    std::string(current_launch_data.launch_name), current_launch_data.start_sector, current_launch_data.next_sector));


  FIL fil;
  FATFS fs;
  FRESULT fr = f_mount(&fs, "", 1);
  if (fr != FR_OK)
  {
    usb_communication::send_string(std::format("TEST MicroSD card failed to mount, error: {} ({})", FRESULT_str(fr), static_cast<uint8_t>(fr)));
    return false;
  }

  fr = f_open(&fil, "test.txt", FA_OPEN_APPEND | FA_WRITE);
  if (fr != FR_OK && fr != FR_EXIST)
  {
    usb_communication::send_string(std::format("Could not open test file on microSD card, error: {} ({})",
                                               FRESULT_str(fr), static_cast<uint8_t>(fr)));
    f_close(&fil);
    f_unmount("");
    return false;
  }

  const uint8_t data[5] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
  uint bytes_written;
  f_write(&fil, data, 5, &bytes_written);
  usb_communication::send_string(std::format("Wrote {} bytes", bytes_written));

  fr = f_close(&fil);
  if (fr != FR_OK)
  {
    usb_communication::send_string(std::format(
      "TEST could not close open file on microSD card, error: {} ({})",
      FRESULT_str(fr), static_cast<uint8_t>(fr)));
    f_unmount("");
    return false;
  }

  f_unmount("");
  return true;

  uint8_t encoded_launch_data[LAUNCH_DATA_SIZE];
  encode_launch_data(current_launch_data, encoded_launch_data);
  return w25q64fv::write_sector(METADATA_SECTOR * SECTOR_SIZE, encoded_launch_data, LAUNCH_DATA_SIZE);
}

bool payload_data_manager::load_stored_launch_data()
{
  return false;
  auto read_data = LaunchData("tmp");
  const bool success =
    w25q64fv::read_data(METADATA_SECTOR * SECTOR_SIZE, reinterpret_cast<uint8_t*>(&read_data), sizeof(LaunchData));
  if (!success)
  {
    return false;
  }

  // usb_communication::send_string(std::format(
  //   "Tried reading launch data for launch \"{}\" starting at sector {} (next sector {}) 0x{:08X}",
  //   std::string(read_data.launch_name), read_data.start_sector, read_data.next_sector, read_data.present_check));

  // if (read_data.present_check != LAUNCH_DATA_PRESENT_CHECK)
  // {
  //   usb_communication::send_string(std::format("present check failed {} != {}", read_data.present_check,
  //                                              LAUNCH_DATA_PRESENT_CHECK));
  //   return false;
  // }

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

  const SectorData* write_data = active_sector_buff;
  init_active_sector();
  mutex_exit(&active_sector_mtx);

  const size_t write_size = sizeof(SectorData::num_instances) + sizeof(SectorData::dropped_instances) + sizeof(
    SectorData::crc) + 4 + write_data->num_instances * ENCODED_DATA_INSTANCE_SIZE;
  uint8_t write_bytes[write_size];
  encode_sector_data(*write_data, write_bytes);
  delete write_data;

  bool write_success = w25q64fv::write_sector(current_launch_data.next_sector * SECTOR_SIZE, write_bytes, write_size);

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
