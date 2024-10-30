#include "main.h"

#include <cstdio>
#include <cstring>
#include <format>
#include <hardware/clocks.h>
#include <hardware/watchdog.h>
#include <pico/multicore.h>

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

  gpio_put(CORE_0_LED_PIN, true);
  sleep_ms(200);
  gpio_put(CORE_1_LED_PIN, true);
  sleep_ms(200);

  if (watchdog_caused_reboot())
  {
  usb_communication::send_string("Reboot caused by watchdog");
  }
  watchdog_enable(5000, true);

  gpio_put(CORE_0_LED_PIN, false);
  // launch_core_1();
  //
  // while (multicore_fifo_get_status() & 0x1 == 0)
  // {
  //   if (multicore_fifo_pop_blocking() == CORE_1_READY_FLAG)
  //   {
  //     break;
  //   }
  // }

  if (status_manager::get_current_status() == status_manager::BOOTING)
  {
    set_status(status_manager::NORMAL);
  }

  static bool led_on = false;
  CollectionData collection_data{{}};
  while (true)
  {
    const uint32_t start_time = time_us_32();
    ds_1307::clock_loop(collection_data);
    bmp_280::data_collection_loop(collection_data);
    mpu_6050::accel_loop(collection_data);

    watchdog_update();
    gpio_put(CORE_0_LED_PIN, led_on = !led_on);

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
  i2c_util::i2c_bus_init(I2C_BUS0, I2C0_SDA_PIN, I2C0_SCL_PIN, 400 * 1000);
  i2c_util::i2c_bus_init(I2C_BUS1, I2C1_SDA_PIN, I2C1_SCL_PIN, 100 * 1000);

  gpio_init(CORE_0_LED_PIN);
  gpio_set_dir(CORE_0_LED_PIN, true);
  gpio_init(CORE_1_LED_PIN);
  gpio_set_dir(CORE_1_LED_PIN, true);

  gpio_init(MPU_6050_INT_PIN);
  gpio_set_dir(MPU_6050_INT_PIN, false);

  gpio_set_function(SPI0_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI0_TX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI0_RX_PIN, GPIO_FUNC_SPI);

  gpio_init(SPI0_CSN_PIN);
  gpio_set_dir(SPI0_CSN_PIN, true);
  gpio_put(SPI0_CSN_PIN, true);

  spi_set_slave(spi0, false);

  // SPI at 104MHz
  spi_init(spi0, 104 * 1000 * 1000);
}
