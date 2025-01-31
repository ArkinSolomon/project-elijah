#include "payload_data_manager.h"

#include <cstring>

#include "byte_util.h"
#include "main.h"
#include "status_manager.h"
#include "usb_communication.h"
#include "sensors/bmp_280/bmp_280.h"
#include "storage/w25q64fv/w25q64fv.h"

payload_data_manager::LaunchData::LaunchData(const std::string& new_launch_name) // NOLINT(*-pro-type-member-init)
{
  strcpy(launch_name, new_launch_name.c_str());
  start_sector = START_SECTOR;
  next_sector = START_SECTOR;
  sector_data_present_check = get_rand_32();
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

  gyro_x = -1;
  gyro_y = -1;
  gyro_z = -1;

  bat_voltage = -1;
  bat_percent = -1;
}

payload_data_manager::DataInstance::DataInstance(const CollectionData& collection_data, const uint64_t us_since_last_data) :
  DataInstance(collection_data, us_since_last_data, DataInstanceEvent::NONE)
{
}

payload_data_manager::DataInstance::DataInstance(const CollectionData& collection_data,
                                                 const uint64_t us_since_last_data,
                                                 const DataInstanceEvent events) :
us_since_last_data(us_since_last_data), events(events)
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

  gyro_x = collection_data.gyro_x;
  gyro_y = collection_data.gyro_y;
  gyro_z = collection_data.gyro_z;

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
  usb_communication::send_string("Loading stored data");
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

  send_current_launch_data_no_lock();
  init_active_sector();
}

void payload_data_manager::init_active_sector()
{
  active_sector_buff = new SectorData;
}

void payload_data_manager::encode_launch_data(const LaunchData& launch_data, uint8_t* data_buff)
{
  memcpy(data_buff, launch_data.launch_name, MAX_LAUNCH_NAME_SIZE);
  byte_util::encode_uint32(launch_data.start_sector, data_buff + MAX_LAUNCH_NAME_SIZE);
  byte_util::encode_uint32(launch_data.next_sector, data_buff + MAX_LAUNCH_NAME_SIZE + 4);
  byte_util::encode_uint32(launch_data.sector_data_present_check, data_buff + MAX_LAUNCH_NAME_SIZE + 8);
  byte_util::encode_double(bmp_280::bmp_280_calib_data.baro_pressure, data_buff + MAX_LAUNCH_NAME_SIZE + 12);
  byte_util::encode_uint32(LAUNCH_DATA_PRESENT_CHECK, data_buff + MAX_LAUNCH_NAME_SIZE + 20);
}

bool payload_data_manager::decode_launch_data(LaunchData& launch_data, const uint8_t* data_buff)
{
  const uint32_t present_check = byte_util::decode_uint32(data_buff + MAX_LAUNCH_NAME_SIZE + 20);
  if (present_check != LAUNCH_DATA_PRESENT_CHECK)
  {
    return false;
  }

  memcpy(launch_data.launch_name, data_buff, MAX_LAUNCH_NAME_SIZE);
  launch_data.start_sector = byte_util::decode_uint32(data_buff + MAX_LAUNCH_NAME_SIZE);
  launch_data.next_sector = byte_util::decode_uint32(data_buff + MAX_LAUNCH_NAME_SIZE + 4);
  launch_data.sector_data_present_check = byte_util::decode_uint32(data_buff + MAX_LAUNCH_NAME_SIZE + 8);
  return true;
}

void payload_data_manager::encode_data_instance(const DataInstance& data_inst, uint8_t* sector_buff)
{
  byte_util::encode_uint64(data_inst.collected_time, sector_buff);
  byte_util::encode_uint32(data_inst.packet_seq, sector_buff + 8);
  byte_util::encode_uint64(data_inst.us_since_last_data, sector_buff + 12);
  // Pressure encoded later

  byte_util::encode_double(data_inst.temperature, sector_buff + 24);
  byte_util::encode_double(data_inst.altitude, sector_buff + 32);
  byte_util::encode_double(data_inst.accel_x, sector_buff + 40);
  byte_util::encode_double(data_inst.accel_y, sector_buff + 48);
  byte_util::encode_double(data_inst.accel_z, sector_buff + 56);
  byte_util::encode_double(data_inst.gyro_x, sector_buff + 64);
  byte_util::encode_double(data_inst.gyro_y, sector_buff + 72);
  byte_util::encode_double(data_inst.gyro_z, sector_buff + 80);
  byte_util::encode_double(data_inst.bat_voltage, sector_buff + 88);
  byte_util::encode_double(data_inst.bat_percent, sector_buff + 96);


  sector_buff[104] = static_cast<uint8_t>(data_inst.events);
  byte_util::encode_int32(data_inst.pressure, sector_buff + 20, sector_buff[104], 7);
}

void payload_data_manager::encode_sector_data(const SectorData& sector_data, uint8_t* sector_buff,
                                              const uint32_t sector_data_present_check)
{
  uint32_t crc = CRC::Calculate(&sector_data.num_instances, 1, crc_table);
  crc = CRC::Calculate(&sector_data.dropped_instances, 2, crc_table, crc);
  crc = CRC::Calculate(&sector_data_present_check, 4, crc_table, crc);

  sector_buff[0] = sector_data.num_instances;
  byte_util::encode_uint16(sector_data.dropped_instances, sector_buff + 1);

  uint8_t* write_loc = sector_buff + 7;
  for (size_t i = 1; i <= sector_data.num_instances; write_loc += ENCODED_DATA_INSTANCE_SIZE, ++i)
  {
    encode_data_instance(sector_data.data[i], write_loc);
    crc = CRC::Calculate(write_loc, ENCODED_DATA_INSTANCE_SIZE, crc_table, crc);
  }

  byte_util::encode_uint32(crc, sector_buff + 3);
  byte_util::encode_uint32(sector_data_present_check, write_loc);
}

bool payload_data_manager::new_launch(const std::string& new_launch_name)
{
  mutex_enter_blocking(&active_sector_mtx);
  mutex_enter_blocking(&launch_data_mtx);

  current_launch_data = LaunchData(new_launch_name);
  const bool success = write_current_launch_data();

  delete active_sector_buff;
  active_sector_buff = new SectorData;

  usb_communication::send_string(std::format("Created new launch: {}", new_launch_name));
  send_current_launch_data_no_lock();

  mutex_exit(&active_sector_mtx);
  mutex_exit(&launch_data_mtx);
  return success;
}

void payload_data_manager::send_current_launch_data()
{
  mutex_enter_blocking(&launch_data_mtx);
  send_current_launch_data_no_lock();
  mutex_exit(&launch_data_mtx);
}

void payload_data_manager::send_current_launch_data_no_lock()
{
  uint8_t encoded_data[ENCODED_LAUNCH_DATA_SIZE];
  encode_launch_data(current_launch_data, encoded_data);
  send_packet(usb_communication::LAUNCH_DATA, encoded_data);
}

bool payload_data_manager::write_current_launch_data()
{
  uint8_t encoded_launch_data[ENCODED_LAUNCH_DATA_SIZE];
  encode_launch_data(current_launch_data, encoded_launch_data);
  return w25q64fv::write_sector(METADATA_SECTOR * SECTOR_SIZE, encoded_launch_data, ENCODED_LAUNCH_DATA_SIZE);
}


bool payload_data_manager::load_stored_launch_data()
{
  uint8_t launch_data_buff[ENCODED_LAUNCH_DATA_SIZE];
  const bool read_success = w25q64fv::read_data(METADATA_SECTOR * SECTOR_SIZE, launch_data_buff,
                                                ENCODED_LAUNCH_DATA_SIZE);

  if (!read_success)
  {
    return false;
  }

  return decode_launch_data(current_launch_data, launch_data_buff);
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
  const bool active_sector_claimed =
    mutex_enter_block_until(&active_sector_mtx, delayed_by_ms(get_absolute_time(), MAX_ACTIVE_SECTOR_LOCK_WAIT_MS));
  if (!active_sector_claimed)
  {
    mutex_exit(&launch_data_mtx);
    return false;
  }

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

  const size_t write_size = sizeof(SectorData::num_instances) + sizeof(SectorData::dropped_instances) + 4 /* CRC */ + 4
    /* Present check */ + write_data->num_instances * ENCODED_DATA_INSTANCE_SIZE;
  uint8_t write_bytes[write_size];
  encode_sector_data(*write_data, write_bytes, current_launch_data.sector_data_present_check);
  delete write_data;

  bool write_success = w25q64fv::write_sector(current_launch_data.next_sector * SECTOR_SIZE, write_bytes, write_size);

  current_launch_data.next_sector++;
  if (!write_success)
  {
    mutex_exit(&launch_data_mtx);
    return false;
  }

  write_success = write_current_launch_data();
  send_current_launch_data_no_lock();
  mutex_exit(&launch_data_mtx);
  return write_success;
}
