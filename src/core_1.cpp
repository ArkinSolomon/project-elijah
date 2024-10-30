#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "pin_outs.h"
#include "core_1.h"

#include <format>

#include "status_manager.h"
#include "usb_communication.h"
#include "storage/w25q64fv.h"

void launch_core_1()
{
  multicore_launch_core1(core_1_main);
}

void core_1_main()
{
  gpio_put(CORE_1_LED_PIN, false);

  const bool did_w25q64fv_init = w25q64fv::init();
  if (!did_w25q64fv_init)
  {
    usb_communication::send_string("Fault W25Q64FV, device failed to initialize");
    set_fault(status_manager::fault_id::DEVICE_W25Q64FV, true);
    multicore_fifo_push_blocking(CORE_1_READY_FLAG);
    return;
  }

  set_status(status_manager::NORMAL);
  multicore_fifo_push_blocking(CORE_1_READY_FLAG);

  static bool led_on = false;
  while (true)
  {

    gpio_put(CORE_1_LED_PIN, led_on = !led_on);
  }
}
