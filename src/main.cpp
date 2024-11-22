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
#ifdef PICO_RP2040
  // set_sys_clock_khz(133000, true);
#elifdef PICO_RP2350
  set_sys_clock_khz(150000, true);
#endif

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

  launch_core_1();

  while (multicore_fifo_get_status() & 0x1 == 0)
  {
    if (multicore_fifo_pop_blocking() == CORE_1_READY_FLAG)
    {
      break;
    }
  }

  if (status_manager::get_current_status() == status_manager::BOOTING)
  {
    set_status(status_manager::NORMAL);
  }
  while (true)
  {
    usb_communication::send_string("hello!");
    usb_communication::check_for_send_data();
  }


  return 0;
  static bool led_on = false;
  CollectionData collection_data{{}};
  absolute_time_t last_loop_time = 0;
  while (true)
  {
    const absolute_time_t start_time = get_absolute_time();
    ds_1307::clock_loop(collection_data);
    bmp_280::data_collection_loop(collection_data);
    mpu_6050::data_int(0, 0);
    mpu_6050::accel_loop(collection_data);

    watchdog_update();
    // gpio_put(CORE_0_LED_PIN, led_on = !led_on);

    const absolute_time_t main_loop_time = absolute_time_diff_us(start_time, get_absolute_time());
    if (stdio_usb_connected())
    {
      if (status_manager::get_current_status() != status_manager::USB)
      {
        usb_communication::say_hello();
        bmp_280::send_calibration_data();
      }

      status_manager::check_faults();

      set_status(status_manager::USB);
      usb_communication::scan_for_packets();
      usb_communication::send_collection_data(collection_data);
      usb_communication::check_for_send_data();

      uint8_t loop_time_data[32];
      byte_util::encode_uint64(main_loop_time, loop_time_data);

      mutex_enter_blocking(&core_1_stats::loop_time_mtx);
      byte_util::encode_uint64(core_1_stats::loop_time, &loop_time_data[8]);
      mutex_exit(&core_1_stats::loop_time_mtx);

      const absolute_time_t time_between_loops = last_loop_time > 0 ? absolute_time_diff_us(last_loop_time, start_time) : 0;
      byte_util::encode_uint64(time_between_loops, &loop_time_data[16]);

      const absolute_time_t usb_loop_time = absolute_time_diff_us(start_time, get_absolute_time());
      byte_util::encode_uint64(usb_loop_time, &loop_time_data[24]);

      send_packet(usb_communication::LOOP_TIME, loop_time_data);
      last_loop_time = get_absolute_time();
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
  i2c_util::i2c_bus_init(I2C_BUS1, I2C1_SDA_PIN, I2C1_SCL_PIN, 400 * 1000);

  gpio_init(CORE_0_LED_PIN);
  gpio_set_dir(CORE_0_LED_PIN, GPIO_OUT);
  gpio_init(CORE_1_LED_PIN);
  gpio_set_dir(CORE_1_LED_PIN, GPIO_OUT);

  gpio_init(MPU_6050_INT_PIN);
  gpio_set_dir(MPU_6050_INT_PIN, GPIO_IN);
  // gpio_set_irq_enabled_with_callback(MPU_6050_INT_PIN, GPIO_IRQ_EDGE_RISE, true, mpu_6050::data_int);

  // SPI at 104MHz
  gpio_set_function(SPI1_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI1_TX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI1_RX_PIN, GPIO_FUNC_SPI);

  gpio_init(SPI1_CSN_PIN);
  gpio_set_dir(SPI1_CSN_PIN, GPIO_OUT);
  gpio_put(SPI1_CSN_PIN, true);

  spi_set_slave(spi1, false);

  spi_init(spi1, 104 * 1000 * 1000);
}
