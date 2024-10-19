#include "main.h"

#include <cstdio>
#include <cstring>
#include <format>
#include <hardware/clocks.h>
#include <hardware/watchdog.h>

#include "byte_util.h"
#include "core_1.h"
#include "pin_outs.h"
#include "status_manager.h"
#include "usb_communication.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "sensors/bmp_280/bmp_280.h"
#include "sensors/ds_1307/ds_1307.h"
#include "sensors/i2c/i2c_util.h"
#include "sensors/mpu_6050/mpu_6050.h"

int main()
{
  set_sys_clock_khz(133000, true);
  pin_init();
  usb_communication::init_usb_com();
  status_manager::status_manager_pio_init();

  if (watchdog_caused_reboot())
  {
    usb_communication::send_string("Reboot caused by watchdog");
  }
  watchdog_enable(2500, true);

  launch_core_1();

  CollectionData collection_data{{}};
  while (true)
  {
    const uint32_t start_time = time_us_32();
    ds_1307::clock_loop(collection_data);
    bmp_280::data_collection_loop(collection_data);
    mpu_6050::accel_loop(collection_data);
    watchdog_update();

    const uint32_t elapsed_time = time_us_32() - start_time;
    if (stdio_usb_connected())
    {
      if (status_manager::get_current_status() != status_manager::USB)
      {
        usb_communication::say_hello();
      }

      set_status(status_manager::USB);
      usb_communication::scan_for_packets();
      usb_communication::send_collection_data(collection_data);

      uint8_t loop_time_data[4];
      byte_util::encode_uint32(elapsed_time, loop_time_data);
      send_packet(usb_communication::LOOP_TIME, loop_time_data);
    }
    else if (status_manager::get_current_status() == status_manager::BOOTING || (status_manager::get_current_status() ==
      status_manager::USB && !stdio_usb_connected()))
    {
      set_status(status_manager::NORMAL);
    }
  }

  watchdog_disable();
}

void pin_init()
{
  i2c_util::i2c_init(I2C_BUS0, I2C0_SDA_PIN, I2C0_SCL_PIN);
  i2c_util::i2c_init(I2C_BUS1, I2C1_SDA_PIN, I2C1_SCL_PIN);

  gpio_init(TEMP_LED_PIN);
  gpio_set_dir(TEMP_LED_PIN, true);

  gpio_init(MPU_6050_INT_PIN);
  gpio_set_dir(MPU_6050_INT_PIN, false);
}
