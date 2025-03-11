#include "core1.h"

#include <pico/flash.h>
#include <pico/multicore.h>

#include "override_state_manager.h"
#include "sensors.h"
#include "state_framework_logger.h"

void core1::launch_core1()
{
  queue_init(&core0_ready_queue, 1, 1);
  queue_init(&core1_ready_queue, 1, 1);

  multicore_reset_core1();
  sleep_ms(100);
  multicore_launch_core1(core1_main);
}

void core1::core1_main()
{
  flash_safe_execute_core_init();
  multicore_lockout_victim_init();

  elijah_state_framework::StateFrameworkLogger::init_driver_on_core();

  uint8_t core_ready = 0xAA;
  queue_add_blocking(&core1_ready_queue, &core_ready);
  queue_remove_blocking(&core0_ready_queue, &core_ready);

  while (true)
  {
    // gpio_put(LED_3_PIN, true);
    // override_state_manager->lock_logger();
    // override_state_manager->get_logger()->write_full_buff();
    // override_state_manager->release_logger();
    // gpio_put(LED_3_PIN, false);

    sleep_ms(50);
  }
}