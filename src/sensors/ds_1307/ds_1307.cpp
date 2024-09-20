#include "ds_1307.h"

#include <cstdio>

#include "../i2c/i2c_util.h"
#include "../../pin_outs.h"
#include "hardware/i2c.h"

/**
 * Check if the clock is enabled and set.
 *
 * Returns false if the device is not detected, or if the clock needs to be set.
 */
bool ds_1307::verify_clock()
{
  uint8_t seconds_reg;
  const bool seconds_read_success = i2c_util::read_ubyte(I2C_BUS, DS_1307_ADDR, _reg_defs::REG_SECONDS,
                                                         seconds_reg);
  if (!seconds_read_success)
  {
    return false;
  }

  TimeInstance time_inst{};
  get_time_instance(time_inst);

  return (seconds_reg & 0x80) == 0 && time_inst.year > 0;
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
  const bool success = i2c_write_blocking(I2C_BUS, DS_1307_ADDR, write_data, 8, false);

  return success;
}

/**
 * Get the current time as an instance.
 */
bool ds_1307::get_time_instance(TimeInstance& time_inst)
{
  uint8_t reg_data[7];
  const bool success = i2c_util::read_bytes(I2C_BUS, DS_1307_ADDR, _reg_defs::REG_SECONDS, reg_data, 7);
  if (!success)
  {
    return false;
  }

  const uint8_t seconds = reg_data[0];
  const uint8_t minutes = reg_data[1];
  const uint8_t hours = reg_data[2];
  uint8_t day = reg_data[3];
  const uint8_t date = reg_data[4];
  const uint8_t month = reg_data[5];
  const uint8_t year = reg_data[6];

  time_inst.seconds = (seconds >> 4 & 0x7) * 10 + (seconds & 0xF);
  time_inst.minutes = (minutes >> 4) * 10 + (minutes & 0xF);
  time_inst.hours = (hours >> 4 & 0x3) * 10 + (hours & 0xF);
  time_inst.date = (date >> 4 & 0x3) * 10 + (date & 0xF);
  time_inst.day = static_cast<day_of_week>(day);
  time_inst.month = static_cast<month_of_year>(((month & 0x1) >> 4) * 10 + (month & 0xF));
  time_inst.year = 2000 + (year >> 4) * 10 + (year & 0xF);

  return true;
}
