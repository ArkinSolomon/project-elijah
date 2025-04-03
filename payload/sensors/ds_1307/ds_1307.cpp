#include "ds_1307.h"

#include <pico/aon_timer.h>

#include "i2c_util.h"
#include "payload_state_manager.h"

DS1307::DS1307(i2c_inst_t* i2c_inst, uint8_t i2c_addr) : i2c_inst(i2c_inst), i2c_addr(i2c_addr)
{
}

bool DS1307::check_clock(bool& clock_set) const
{
  uint8_t seconds_reg;
  const bool seconds_read_success = i2c_util::read_ubyte(i2c0, i2c_addr, REG_SECONDS,
                                                         seconds_reg);
  if (!seconds_read_success)
  {
    clock_set = false;
    return false;
  }

  tm time_inst{};
  const bool read_success = read_clock(time_inst);

  clock_set = read_success && (seconds_reg & 0x80) == 0 && time_inst.tm_year > 0;
  return true;
}

/**
 * Set the clock. Use 24-hour time.
 *
 * @param time_inst The time instance at which to set the clock.
 * @return True if the clock was set successfully.
 */
bool DS1307::set_clock(const tm& time_inst) const
{
  const uint8_t seconds_data = time_inst.tm_sec / 10 << 4 | time_inst.tm_sec % 10;
  const uint8_t minutes_data = time_inst.tm_min / 10 << 4 | time_inst.tm_min % 10;
  const uint8_t hours_data = time_inst.tm_hour / 10 << 4 | time_inst.tm_hour % 10;
  const uint8_t date_data = time_inst.tm_mday / 10 << 4 | time_inst.tm_mday % 10;
  const uint8_t month_data = ((time_inst.tm_mon + 1) / 10 << 4) | ((time_inst.tm_mon + 1) % 10);

  const uint8_t year_data = static_cast<uint8_t>(time_inst.tm_year) / 10 << 4 | time_inst.tm_year % 10;
  const uint8_t write_data[8] = {
    REG_SECONDS, seconds_data, minutes_data, hours_data, static_cast<uint8_t>(time_inst.tm_wday & 0xFF),
    date_data, month_data,
    year_data
  };
  const int bytes_written = i2c_write_blocking_until(i2c_inst, i2c_addr, write_data, 8, false,
                                                     delayed_by_ms(get_absolute_time(), 32));

  return bytes_written == 8;
}

/**
 * Check if the DS1307 is currently functional. Set it if it's not set.
 *
 * @param reset_inst The time to set if the clock is not set.
 * @return True if the clock is functional.
 */
bool DS1307::functional_check(const tm& reset_inst) const
{
  bool clock_set = false;
  if (!check_clock(clock_set))
  {
    payload_state_manager->set_fault(PayloadFaultKey::DS1307, true, "Not detected during functional check");
    return false;
  }

  // Clock detected but not set
  if (!clock_set)
  {
    if (set_clock(reset_inst))
    {
      payload_state_manager->set_fault(PayloadFaultKey::DS1307, false);
      return true;
    }

    payload_state_manager->set_fault(PayloadFaultKey::DS1307, true,
                                     "Detected, but unable to set during functional reset");
    return false;
  }

  payload_state_manager->set_fault(PayloadFaultKey::DS1307, false);
  return true;
}

/**
 * Get the current time as an instance.
 *
 * @param time_inst The location at which to set the read time.
 * @return True if the clock can be read from successfully.
 */
bool DS1307::read_clock(tm& time_inst) const
{
  uint8_t reg_data[7];
  const bool success = i2c_util::read_bytes(i2c_inst, i2c_addr, REG_SECONDS, reg_data, 7);
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

  time_inst.tm_sec = (seconds >> 4 & 0x7) * 10 + (seconds & 0xF);
  time_inst.tm_min = (minutes >> 4) * 10 + (minutes & 0xF);
  time_inst.tm_hour = (hours >> 4 & 0x3) * 10 + (hours & 0xF);
  time_inst.tm_mday = (date >> 4 & 0x3) * 10 + (date & 0xF);
  time_inst.tm_wday = day;
  time_inst.tm_mon = ((month & 0x10) >> 4) * 10 + (month & 0xF) - 1;
  time_inst.tm_year = (year >> 4) * 10 + (year & 0xF);

  return true;
}

bool DS1307::read_custom_register(const uint8_t addr, uint8_t* output, const uint8_t size)
{
  return i2c_util::read_bytes(i2c_inst, i2c_addr, addr, output, size);
}

bool DS1307::write_custom_register(const uint8_t addr, const uint8_t* data, const uint8_t size)
{
  const auto write_addr = static_cast<uint8_t>(addr);
  uint8_t write_data[size + 1];
  for (uint8_t i = 0; i < size; i++)
  {
    write_data[i + 1] = data[i];
  }
  write_data[0] = write_addr;
  const int bytes_written = i2c_write_blocking_until(i2c_inst, i2c_addr, write_data, size + 1, false,
                                                     delayed_by_ms(get_absolute_time(), 32));
  return bytes_written == size + 1;
}

/**
 * Print all registers, or that a read failed.
 */
void DS1307::reg_dump() const
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
    const bool success = i2c_util::read_ubyte(i2c_inst, i2c_addr, addr, read_data);
    if (!success)
    {
      payload_state_manager->log_message(std::format("Reg dump failed reading 0x{:02x}", addr),
                                         elijah_state_framework::LogLevel::Warning);
    }

    output += std::format(" 0x{:02x}", read_data);
  }

  payload_state_manager->log_message(output);
}

/**
 * Clear the DS 1307.
 */
void DS1307::erase_data() const
{
  // Overwrite from reg 0x00 to 0x3F (+ 1 byte for addr)
  constexpr uint8_t zeros[0x40] = {};
  const int bytes_written = i2c_write_blocking_until(i2c_inst, i2c_addr, zeros, 0x40, false,
                                                     delayed_by_ms(get_absolute_time(), 32));
  const bool success = bytes_written == 0x40;
  if (!success)
  {
    payload_state_manager->log_message("DS 1307 failed to erase", elijah_state_framework::LogLevel::Warning);
    return;
  }

  payload_state_manager->log_message("DS 1307 erased");
}

/**
 * Check if the clock is detected and set.
 *
 * @param time_inst The time instance to update with the read value.
 * @return True if the clock is detected.
 */
bool DS1307::check_and_read_clock(tm& time_inst) const
{
  bool clock_set = false;
  bool clock_detected = check_clock(clock_set);;

  if (!clock_detected)
  {
    payload_state_manager->set_fault(PayloadFaultKey::DS1307, true, "Clock not detected (no acknowledgement)");
    return false;
  }

  if (!clock_set)
  {
    payload_state_manager->set_fault(PayloadFaultKey::DS1307, true, "Clock detected, but not set");
    return false;
  }

  clock_detected = read_clock(time_inst);
  if (!clock_detected)
  {
    payload_state_manager->set_fault(PayloadFaultKey::DS1307, true,
                                     "Clock not detected (acknowledged but failed to read time)");
    return false;
  }

  payload_state_manager->set_fault(PayloadFaultKey::DS1307, false);
  return true;
}
