#include "main.h"

#include <cmath>
#include <format>
#include <hardware/clocks.h>
#include <hardware/watchdog.h>
#include <pico/flash.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <pico/aon_timer.h>

#include "byte_util.h"
#include "core_1.h"
#include "pin_outs.h"
#include "status_manager.h"
#include "usb_communication.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "sensors/onboard_clock/onboard_clock.h"
#include "sensors/battery/battery.h"
#include "sensors/bmp_280/bmp_280.h"
#include "sensors/i2c/i2c_util.h"
#include "sensors/mpu_6050/mpu_6050.h"

#include "sd_card.h"

int main()
{
#ifdef DEBUG
  busy_wait_ms(150);
#endif

#ifdef PICO_RP2040
  set_sys_clock_khz(133000, true);
#elifdef PICO_RP2350
    set_sys_clock_khz(150000, true);
#endif

  pin_init();

  usb_communication::init_usb_com();
  status_manager::status_manager_init();
  flash_safe_execute_core_init();
  mpu_6050::init_crit_section();
  mpu_6050::configure_default_with_lock();

  gpio_put(CORE_0_LED_PIN, true);
  multicore_reset_core1();
  sleep_ms(200);
  gpio_put(CORE_1_LED_PIN, true);
  sleep_ms(200);

  if (watchdog_caused_reboot())
  {
    usb_communication::send_string("Reboot caused by watchdog");
  }
  watchdog_enable(5000, true);

  sleep_ms(1000);
  gpio_put(CORE_0_LED_PIN, false);

  launch_core_1();

  while (multicore_fifo_get_status() & 0x1 == 0)
  {
    if (multicore_fifo_pop_blocking() == CORE_1_READY_FLAG)
    {
      usb_communication::send_string("Core 1 ready");
      break;
    }
  }

  if (status_manager::get_current_status() == status_manager::BOOTING)
  {
    set_status(status_manager::NORMAL);
  }

  constexpr uint64_t us_between_loops = 1000000 / MAX_UPDATES_PER_SECOND;

  bool led_on = false;
  bool usb_connected = false;

  CollectionData collection_data{{}};
  absolute_time_t last_loop_start_time = nil_time, last_loop_end_time = nil_time;
  while (true)
  {
    const absolute_time_t start_time = get_absolute_time();

    onboard_clock::clock_loop(collection_data);
    bmp_280::data_collection_loop(collection_data);
    mpu_6050::accel_loop(collection_data);
    battery::collect_bat_information(collection_data);

    const absolute_time_t time_since_last_collection = absolute_time_diff_us(last_loop_start_time, get_absolute_time());
    buffer_data(payload_data_manager::DataInstance(collection_data, time_since_last_collection));

    watchdog_update();
    // gpio_put(CORE_0_LED_PIN, led_on = !led_on);

    const absolute_time_t main_loop_time = absolute_time_diff_us(start_time, get_absolute_time());
    if (stdio_usb_connected())
    {
      const absolute_time_t usb_start_time = get_absolute_time();
      if (!usb_connected)
      {
        usb_communication::say_hello();
        bmp_280::send_calibration_data();
        status_manager::send_status();
        usb_connected = true;
      }

      status_manager::check_faults();

      usb_communication::scan_for_packets();
      usb_communication::send_collection_data(collection_data);

      uint8_t loop_time_data[32];
      byte_util::encode_uint64(main_loop_time, loop_time_data);

      mutex_enter_blocking(&core_1_stats::loop_time_mtx);
      byte_util::encode_uint64(core_1_stats::loop_time, &loop_time_data[8]);
      mutex_exit(&core_1_stats::loop_time_mtx);

      const absolute_time_t time_between_loops = last_loop_end_time > 0
                                                   ? absolute_time_diff_us(last_loop_end_time, start_time)
                                                   : 0;
      byte_util::encode_uint64(time_between_loops, &loop_time_data[16]);

      const absolute_time_t usb_loop_time = absolute_time_diff_us(usb_start_time, get_absolute_time());
      byte_util::encode_uint64(usb_loop_time, &loop_time_data[24]);

      send_packet(usb_communication::LOOP_TIME, loop_time_data);
    }
    else if (usb_connected)
    {
      usb_connected = false;
    }

    last_loop_start_time = start_time;
    sleep_until(delayed_by_us(last_loop_end_time, us_between_loops));
    last_loop_end_time = get_absolute_time();
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
  gpio_set_irq_enabled_with_callback(MPU_6050_INT_PIN, GPIO_IRQ_EDGE_RISE, true, mpu_6050::data_int);

  // // SPI at 25MHz for MicroSD
  // gpio_set_function(SPI0_SCK_PIN, GPIO_FUNC_SPI);
  // gpio_set_function(SPI0_TX_PIN, GPIO_FUNC_SPI);
  // gpio_set_function(SPI0_RX_PIN, GPIO_FUNC_SPI);
  //
  // gpio_init(SPI0_CSN_PIN);
  // gpio_set_dir(SPI0_CSN_PIN, GPIO_OUT);
  // gpio_put(SPI0_CSN_PIN, true);
  //
  // spi_set_slave(spi0, false);
  // spi_init(spi0, 25 * 1000 * 1000);

  // SPI at 33MHz for W25Q64FV
  gpio_set_function(SPI1_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI1_TX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI1_RX_PIN, GPIO_FUNC_SPI);

  gpio_init(SPI1_CSN_PIN);
  gpio_set_dir(SPI1_CSN_PIN, GPIO_OUT);
  gpio_put(SPI1_CSN_PIN, true);

  spi_set_slave(spi1, false);
  spi_init(spi1, 33 * 1000 * 1000);

  // Battery ADC
  adc_init();
  adc_gpio_init(BAT_VOLTAGE_PIN);
  adc_select_input(BAT_ADC_INPUT);
}
