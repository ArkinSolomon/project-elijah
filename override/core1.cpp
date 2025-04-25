#include "core1.h"

#include <pico/flash.h>
#include <pico/multicore.h>

#include "override_state_manager.h"
#include "pin_outs.h"
#include "override_sensors.h"
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

  gpio_put(LED_3_PIN, true);
  bool led_on = true;

  bool is_first_detection = false;
  bool has_disabled = false;
  absolute_time_t land_time = 0;
  while (true)
  {
    constexpr uint32_t disable_ms = OVERRIDE_DISABLE_TIME_MIN * 60000;
    if (override_state_manager->get_current_flight_phase() ==
      elijah_state_framework::std_helpers::StandardFlightPhase::LANDED)
    {
      if (!is_first_detection)
      {
        land_time = get_absolute_time();
        override_state_manager->log_message(std::format("Land detected, will override PTT in {}ms", disable_ms));
        gpio_put(PTT_ENABLE, true);
        is_first_detection = true;
      }
      else if (delayed_by_ms(land_time, disable_ms) < get_absolute_time() && !has_disabled)
      {
        override_state_manager->log_message("PTT disabled");
        gpio_put(PTT_ENABLE, false);
        has_disabled = true;
      }
    }
    else
    {
      has_disabled = false;
      is_first_detection = false;
      gpio_put(PTT_ENABLE, false);
    }

    gpio_put(LED_3_PIN, led_on = !led_on);
    sleep_ms(50);
  }
}
