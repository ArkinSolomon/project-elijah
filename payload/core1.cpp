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
  bool led_on = true;

  absolute_time_t next_transmission_time = nil_time;

  bool did_land = false;
  PayloadState land_state;
  while (true)
  {
    payload_state_manager->check_for_log_write();

    if (payload_state_manager->get_current_flight_phase() ==
      elijah_state_framework::std_helpers::StandardFlightPhase::LANDED && (next_transmission_time == nil_time ||
        next_transmission_time <= get_absolute_time()))
    {
      if (!did_land)
      {
        land_state = payload_state_manager->get_state_history()[0];
        did_land = true;
      }

      payload_state_manager->log_message("Transmit!");

      double apogee = payload_state_manager->get_flight_phase_controller()->get_apogee();
      aprs::transmitAllData(land_state, apogee);
      next_transmission_time = delayed_by_ms(get_absolute_time(), 5000);

      payload_state_manager->log_message("Transmission complete!");
    }
    gpio_put(LED_3_PIN, led_on = !led_on);
    sleep_ms(50);
  }
}
