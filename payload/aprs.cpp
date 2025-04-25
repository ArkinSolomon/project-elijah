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

void aprs::transmitAllData(const PayloadState& state, double apogee)
{
  if (!abp)
  {
    set_sys_clock_48mhz();
    abp = aprs_pico_init();
  }

  const tm& tmLand = state.time_inst;
  std::string orientation = "Not Available";
  const double x = state.accel_x;
  const double y = state.accel_y;
  const double z = state.accel_z;
  if (abs(x) > abs(y) && abs(x) > abs(z) && x > 0) orientation = "Port-Facing";
  if (abs(x) > abs(y) && abs(x) > abs(z) && x < 0) orientation = "Starboard-Facing";
  if (abs(y) > abs(x) && abs(y) > abs(z) && y > 0) orientation = "Inverted";
  if (abs(y) > abs(x) && abs(y) > abs(z) && y < 0) orientation = "Upright";
  if (abs(z) > abs(x) && abs(z) > abs(y) && z > 0) orientation = "Sky-Facing";
  if (abs(z) > abs(x) && abs(z) > abs(y) && z < 0) orientation = "Ground-Facing";

  gpio_put(RADIO_PTT_PIN, false);
  transmit_data = std::format("  Test transmission on {}/{}/{}. ", tmLand.tm_mon, tmLand.tm_mday,
                              tmLand.tm_year + 1980);
  elijah_state_framework::log_serial_message(transmit_data);
  transmitData(abp, transmit_data);

  transmit_data = std::format("  Current Temperature: {:.1f} degC ", state.temperature);
  elijah_state_framework::log_serial_message(transmit_data);
  transmitData(abp, transmit_data);

  transmit_data = std::format("  Apogee Reached: {:.1f} meters ", apogee);
  elijah_state_framework::log_serial_message(transmit_data);
  transmitData(abp, transmit_data);

  transmit_data = std::format("  STEMnaut Orientation: {} ({:.3f}, {:.3f}, {:.3f})", orientation, x, y, z);
  elijah_state_framework::log_serial_message(transmit_data);
  transmitData(abp, transmit_data);

  transmit_data = std::format("  Time of Landing: {}:{} ", tmLand.tm_hour, tmLand.tm_min);
  elijah_state_framework::log_serial_message(transmit_data);
  transmitData(abp, transmit_data);

  transmit_data = std::format("  Battery Level: {:.1f}% ", state.bat_percent);
  elijah_state_framework::log_serial_message(transmit_data);
  transmitData(abp, transmit_data);
  sleep_ms(50);
  gpio_put(RADIO_PTT_PIN, true);
}

void aprs::transmitData(audio_buffer_pool_t* audio_buffer_pool, const std::string& data)
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
