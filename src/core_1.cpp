#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "pin_outs.h"
#include "core_1.h"

#include <format>
#include <pico/flash.h>

#include "status_manager.h"
#include "usb_communication.h"
#include "storage/payload_data_manager.h"
#include "storage/w25q64fv.h"

void launch_core_1()
{
  mutex_init(&core_1_stats::loop_time_mtx);
  multicore_launch_core1(core_1_main);
}

void core_1_main()
{
  gpio_put(CORE_1_LED_PIN, false);
  flash_safe_execute_core_init();

  const bool did_w25q64fv_init = w25q64fv::init();
  if (!did_w25q64fv_init)
  {
    usb_communication::send_string("Fault: W25Q64FV, device failed to initialize");
    set_fault(status_manager::fault_id::DEVICE_W25Q64FV, true);
    multicore_fifo_push_blocking(CORE_1_READY_FLAG);
    return;
  }

  // payload_data_manager::init_launch_data();

  uint8_t test_data[32];
  for (size_t i = 0; i < 32; i++)
  {
    test_data[i] = get_rand_32() & 0xFF;
  }

  w25q64fv::write_sector(0, test_data, 32);
  uint8_t test_data_out[32];
  w25q64fv::read_data(0, test_data_out, 32);
  for (size_t i = 0; i < 32; i++)
  {
    usb_communication::send_string(std::format("data {:02X} == {:02X}", test_data[i],
      test_data_out[i]));
  }

  set_status(status_manager::NORMAL);
  multicore_fifo_push_blocking(CORE_1_READY_FLAG);

  static bool led_on = false;
  while (true)
  {
    const absolute_time_t start_time = get_absolute_time();
    gpio_put(CORE_1_LED_PIN, led_on = !led_on);

    sleep_ms(100);

    mutex_enter_blocking(&core_1_stats::loop_time_mtx);
    const uint64_t elapsed_time = absolute_time_diff_us(start_time, get_absolute_time());
    core_1_stats::loop_time = elapsed_time;
    mutex_exit(&core_1_stats::loop_time_mtx);
  }
}