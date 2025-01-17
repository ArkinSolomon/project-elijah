#include "ds_1307.h"

#include <cstdio>
#include <format>

#include "../i2c/i2c_util.h"
#include "../../pin_outs.h"
#include "hardware/i2c.h"
#include "src/main.h"
#include "src/status_manager.h"
#include "src/usb_communication.h"
#include "src/sensors/bmp_280/bmp_280.h"

/**
 * Check if the clock is detected,
 *
 * Returns false if the device is not detected. Set is set to true if the clock is detected AND has been set.
 */
bool ds_1307::check_clock(bool& clock_set)
{
  uint8_t seconds_reg;
  const bool seconds_read_success = i2c_util::read_ubyte(I2C_BUS0, DS_1307_ADDR, _reg_defs::REG_SECONDS,
                                                         seconds_reg);
  if (!seconds_read_success)
  {
    clock_set = false;
    return false;
  }

  TimeInstance time_inst{};
  get_time_instance(time_inst);

  clock_set = (seconds_reg & 0x80) == 0 && time_inst.year > 0;
  return true;
}

/**
 * Set the clock.
 *
 * Use 24-hour time.
 */
bool ds_1307::set_clock(uint16_t year, const month_of_year month, const day_of_week day, const uint8_t date,
                        const uint8_t hours, const uint8_t minutes, const uint8_t seconds)
{
  const uint8_t seconds_data = seconds / 10 << 4 | seconds % 10;
  const uint8_t minutes_data = minutes / 10 << 4 | minutes % 10;
  const uint8_t hours_data = hours / 10 << 4 | hours % 10;
  const uint8_t date_data = date / 10 << 4 | date % 10;
  const uint8_t month_data = static_cast<uint8_t>(month) / 10 << 4 | month % 10;

  year -= 2000;
  const uint8_t year_data = static_cast<uint8_t>(year) / 10 << 4 | year % 10;
  const uint8_t write_data[8] = {
    _reg_defs::REG_SECONDS, seconds_data, minutes_data, hours_data, static_cast<uint8_t>(day), date_data, month_data,
    year_data
  };
  const int bytes_written = i2c_write_blocking_until(I2C_BUS0, DS_1307_ADDR, write_data, 8, false,
                                                     delayed_by_ms(get_absolute_time(), 32));

  return bytes_written == 8;
}

/**
 * Get the current time as an instance.
 */
bool ds_1307::get_time_instance(TimeInstance& time_inst)
{
  uint8_t reg_data[7];
  const bool success = i2c_util::read_bytes(I2C_BUS0, DS_1307_ADDR, _reg_defs::REG_SECONDS, reg_data, 7);
  if (!success)
  {
    return false;
  }

  const uint8_t seconds = reg_data[0];
  const uint8_t minutes = reg_data[1];
  const uint8_t hours = reg_data[2];
  const uint8_t day = reg_data[3];
  const uint8_t date = reg_data[4];
  const uint8_t month = reg_data[5];
  const uint8_t year = reg_data[6];

  time_inst.seconds = (seconds >> 4 & 0x7) * 10 + (seconds & 0xF);
  time_inst.minutes = (minutes >> 4) * 10 + (minutes & 0xF);
  time_inst.hours = (hours >> 4 & 0x3) * 10 + (hours & 0xF);
  time_inst.date = (date >> 4 & 0x3) * 10 + (date & 0xF);
  time_inst.day = static_cast<day_of_week>(day);
  time_inst.month = static_cast<month_of_year>(((month & 0x10) >> 4) * 10 + (month & 0xF));
  time_inst.year = 2000 + (year >> 4) * 10 + (year & 0xF);

  return true;
}

/**
 * Reset all values in the instance to 0.
 */
void ds_1307::load_blank_inst(TimeInstance& time_inst)
{
  time_inst.year = 0;
  time_inst.month = MONTH_NOT_SET;
  time_inst.date = 0;
  time_inst.hours = 0;
  time_inst.minutes = 0;
  time_inst.seconds = 0;
  time_inst.day = DAY_NOT_SET;
}

void ds_1307::handle_time_set_packet(const uint8_t* packet_data)
{
  const uint8_t seconds = packet_data[0];
  const uint8_t minutes = packet_data[1];
  const uint8_t hours = packet_data[2];
  const auto day = static_cast<day_of_week>(packet_data[3]);
  const uint8_t date = packet_data[4];
  const auto month = static_cast<month_of_year>(packet_data[5]);
  const uint16_t year = packet_data[6] << 8 | packet_data[7];

  const bool clock_did_set = set_clock(year, month, day, date, hours, minutes, seconds);
  if (!clock_did_set)
  {
    usb_communication::send_string("Fault: DS 1307, failed to set clock");
    set_fault(status_manager::fault_id::DEVICE_DS_1307, true);
    send_packet(usb_communication::TIME_SET_FAIL);
  }
  else
  {
    set_fault(status_manager::fault_id::DEVICE_DS_1307, false);
    send_packet(usb_communication::TIME_SET_SUCCESS);
  }
}

bool ds_1307::read_custom_register(const custom_register addr, uint8_t* output, const uint8_t size)
{
  return i2c_util::read_bytes(I2C_BUS0, DS_1307_ADDR, static_cast<uint8_t>(addr), output, size);
}

bool ds_1307::write_custom_register(const custom_register addr, const uint8_t* data, const uint8_t size)
{
  const auto write_addr = static_cast<uint8_t>(addr);
  uint8_t write_data[size + 1];
  for (uint8_t i = 0; i < size; i++)
  {
    write_data[i + 1] = data[i];
  }
  write_data[0] = write_addr;
  const int bytes_written = i2c_write_blocking_until(I2C_BUS0, DS_1307_ADDR, write_data, size + 1, false,
                                                     delayed_by_ms(get_absolute_time(), 32));
  return bytes_written == size + 1;
}

/**
 * Print all registers, or that a read failed.
 */
void ds_1307::reg_dump()
{
  std::string output = "     ";
  for (uint8_t i = 0; i < 16; i++)
  {
    output += std::format(" 0x_{:01x}", i);
  }
  for (uint8_t addr = 0; addr <= 0x3F; ++addr)
  {
    if (addr % 16 == 0)
    {
      output += std::format("\n0x{:01x}_ ", addr >> 4);
    }

    uint8_t read_data = 0;
    const bool success = i2c_util::read_ubyte(I2C_BUS0, DS_1307_ADDR, addr,
                                              read_data);
    if (!success)
    {
      usb_communication::send_string(std::format("Reg dump failed reading 0x{:02x}", addr));
    }

    output += std::format(" 0x{:02x}", read_data);
  }

  usb_communication::send_string(output);
}

/**
 * Clear the DS 1307.
 */
void ds_1307::erase_data()
{
  constexpr uint8_t zeros[0x3F] = {};
  const int bytes_written = i2c_write_blocking_until(I2C_BUS0, DS_1307_ADDR, zeros, 0x3F, false,
                                                     delayed_by_ms(get_absolute_time(), 32));
  const bool success = bytes_written == 0x3F;
  if (!success)
  {
    usb_communication::send_string("DS 1307 failed to erase");
    return;
  }

  usb_communication::send_string("DS 1307 erased");
}

void ds_1307::clock_loop(CollectionData& collection_data)
{
  static bool clock_detected = false, clock_set = false;

  if (!clock_detected || !clock_set)
  {
    clock_detected = check_clock(clock_set);
    if (!clock_detected)
    {
      usb_communication::send_string("Fault: DS 1307, device not detected (no acknowledgement)");
      set_fault(status_manager::fault_id::DEVICE_DS_1307, true);
      return;
    }

    // Clock is detected but not set
    if (!clock_set)
    {
      set_fault(status_manager::fault_id::DEVICE_DS_1307, true);

      usb_communication::send_string("Fault: DS 1307, clock not set");
      return;
    }
  }

  clock_detected = get_time_instance(collection_data.time_inst);
  if (!clock_detected)
  {
    load_blank_inst(collection_data.time_inst);
    return;
  }

  set_fault(status_manager::fault_id::DEVICE_DS_1307, false);
}
