#include "main.h"

#include <cstdio>
#include <cstring>
#include <format>
#include <hardware/clocks.h>

#include "core_1.h"
#include "pin_outs.h"
#include "status_manager.h"
#include "usb_communication.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "sensors/bmp_280/bmp_280.h"
#include "sensors/ds_1307/ds_1307.h"
#include "sensors/i2c/i2c_util.h"

int main()
{
  set_sys_clock_khz(133000, true);
  pin_init();
  usb_communication::init_usb_com();
  status_manager::status_manager_pio_init();
  launch_core_1();

  CollectionData collection_data{{}};
  while (true)
  {
    ds_1307::clock_loop(collection_data);
    bmp_280::data_collection_loop(collection_data);

    if (stdio_usb_connected())
    {
      if (status_manager::get_current_status() != status_manager::USB)
      {
        usb_communication::say_hello();
      }

      set_status(status_manager::USB);
      usb_communication::scan_for_packets();
      usb_communication::send_collection_data(collection_data);
    }
    else if (status_manager::get_current_status() == status_manager::BOOTING || (status_manager::get_current_status() ==
      status_manager::USB && !stdio_usb_connected()))
    {
      set_status(status_manager::NORMAL);
    }

    const double alt_ft = collection_data.altitude * 3.2808;
    const double temp_f = collection_data.temperature * (static_cast<double>(9) / 5) + 32;

    // const float conversion_factor = 3.3f / 0x1000;
    // uint16_t adc_raw = adc_read();
    // float voltage = adc_raw * conversion_factor;
    // float t = 27 - (voltage - 0.706) / 0.001721;

    // usb_communication::send_string(
    //   "[%02d/%02d/%04d %02d:%02d:%02d] temp (C): %.1f temp: (F): %.2f press (Pa): %d, altitude (m): %.3f altitude (ft): %.3f\n",
    //   collection_data.time_inst.month, collection_data.time_inst.date, collection_data.time_inst.year,
    //   collection_data.time_inst.hours, collection_data.time_inst.minutes, collection_data.time_inst.seconds,
    //   collection_data.temperature, temp_f, collection_data.pressure, collection_data.altitude, alt_ft);
  }
}

void pin_init()
{
  i2c_util::i2c_init(I2C_BUS, I2C_SDA_PIN, I2C_SCL_PIN);

  gpio_init(TEMP_LED_PIN);
  gpio_set_dir(TEMP_LED_PIN, true);

  adc_init();
  adc_select_input(ONBOARD_TEMP_PIN);
}