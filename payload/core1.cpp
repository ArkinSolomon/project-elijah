#include "core1.h"

#include <pico/flash.h>
#include <pico/multicore.h>

#include "payload_state_manager.h"
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
    if (payload_state_manager->get_current_flight_phase() == StandardFlightPhase::LANDED)
    {
      // TODO
    }

    sleep_ms(50);
  }
}
