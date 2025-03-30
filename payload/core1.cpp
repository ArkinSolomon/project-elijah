#include "core1.h"

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <pico/flash.h>
#include <pico/multicore.h>

#include "aprs.h"
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

  constexpr uint8_t core_ready = 0xAA;
  queue_add_blocking(&core1_ready_queue, &core_ready);
  queue_remove_blocking(&core0_ready_queue, nullptr);

  gpio_put(LED_3_PIN, true);
  sleep_ms(5000);
  gpio_put(25, true);
  for (uint i = 0; i < 12; ++i)
  {
      printf(std::format("Channel {}: {}", i, dma_channel_is_claimed(i)).c_str());
  }
  gpio_put(25, false);
  // gpio_put(25, false);
  //
  // // while (payload_state_manager->get_current_flight_phase() != StandardFlightPhase::LANDED || to_ms_since_boot(get_absolute_time()) < 5000);
  // PayloadState last_state = payload_state_manager->get_state_history()[0];
  // aprs::transmitAllData(last_state);

}
