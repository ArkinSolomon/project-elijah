#include "aprs.h"
#include "aprs_pico.h"
#include <hardware/clocks.h>

#include <cmath>
#include <cstring>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/structs/io_bank0.h>
#include <pico/time.h>

#include "pin_outs.h"

void aprs::transmit_all_data(const PayloadState& state, double apogee)
{
  if (!abp)
  {
    set_sys_clock_48mhz();
    abp = aprs_pico_init();
  }

  const tm& tmLand = state.time_inst;
  const std::string orientation = get_transmit_orientation(state);

  gpio_put(RADIO_PTT_PIN, false);
  transmit_str = std::format("  Test transmission on {}/{}/{}. ", tmLand.tm_mon, tmLand.tm_mday,
                             tmLand.tm_year + 1980);
  payload_state_manager->log_message(transmit_str);
  transmit_data(abp, transmit_str);

  transmit_str = std::format("  Current Temperature: {:.1f} degC ", state.temperature);
  payload_state_manager->log_message(transmit_str);
  transmit_data(abp, transmit_str);

  transmit_str = std::format("  Apogee Reached: {:.1f} meters ", apogee);
  payload_state_manager->log_message(transmit_str);
  transmit_data(abp, transmit_str);

  transmit_str = std::format("  STEMnaut Orientation: {} ({:.3f}, {:.3f}, {:.3f})", orientation, state.accel_x, state.accel_y, state.accel_z);
  payload_state_manager->log_message(transmit_str);
  transmit_data(abp, transmit_str);

  transmit_str = std::format("  Time of Landing: {}:{} ", tmLand.tm_hour, tmLand.tm_min);
  payload_state_manager->log_message(transmit_str);
  transmit_data(abp, transmit_str);

  transmit_str = std::format("  Battery Level: {:.1f}% ({:.2f} volts) ", state.bat_percent, state.bat_voltage);
  payload_state_manager->log_message(transmit_str);
  transmit_data(abp, transmit_str);
  sleep_ms(50);
  gpio_put(RADIO_PTT_PIN, true);
}

void aprs::transmit_data(audio_buffer_pool_t* audio_buffer_pool, const std::string& data)
{
  sleep_ms(500);
  aprs_pico_sendAPRS(audio_buffer_pool,
                     "KF8CDC-11", // Source call sign
                     "KF8CDC-7", // Destination call sign
                     "WIDE1-1", // APRS path #1
                     "WIDE2-2", // APRS path #2
                     data.c_str(), // Data message
                     39.74747, // Latitude  (in deg)
                     -83.81279, // Longitude (in deg)
                     318, // Altitude  (in m)
                     '\\', // APRS symbol table: Secondary
                     'O', // APRS symbol code:  Rocket
                     128u); // Volume    (0 ... 256)
}

std::string aprs::get_transmit_orientation(const PayloadState& state)
{
  std::string orientation = "Not Available";
  const double x = state.accel_x;
  const double y = state.accel_y;
  const double z = state.accel_z;

  if (abs(x) > abs(y) && abs(x) > abs(z) && x > 0)
  {
    orientation = "Port-Facing";
  }
  else if (abs(x) > abs(y) && abs(x) > abs(z) && x < 0)
  {
    orientation = "Starboard-Facing";
  }
  else if (abs(y) > abs(x) && abs(y) > abs(z) && y > 0)
  {
    orientation = "Inverted";
  }
  else if (abs(y) > abs(x) && abs(y) > abs(z) && y < 0)
  {
    orientation = "Upright";
  }
  else if (abs(z) > abs(x) && abs(z) > abs(y) && z > 0)
  {
    orientation = "Ground-Facing";
  }
  else if (abs(z) > abs(x) && abs(z) > abs(y) && z < 0)
  {
    orientation = "Sky-Facing";
  }
  return orientation;
}
