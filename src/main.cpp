#include "main.h"

#include <cstdio>
#include <cstring>
#include <format>
#include <hardware/clocks.h>

#include "core_1.h"
#include "pin_outs.h"
#include "status_manager.h"
#include "usb_communication.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "sensors/bmp_280/bmp_280.h"
#include "sensors/ds_1307/ds_1307.h"
#include "sensors/hmc_5883l/hmc_5883l.h"
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
    // ds_1307::clock_loop(collection_data);
    // bmp_280::data_collection_loop(collection_data);
    hmc_5883l::accel_loop(collection_data);

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
  }
}

void pin_init()
{
  i2c_util::i2c_init(I2C_BUS, I2C_SDA_PIN, I2C_SCL_PIN);

  gpio_init(TEMP_LED_PIN);
  gpio_set_dir(TEMP_LED_PIN, true);

  gpio_init(HMC_5883L_RDY_PIN);
  gpio_set_dir(HMC_5883L_RDY_PIN, false);
}