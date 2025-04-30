#include <format>
#include <sd_card.h>

#include <pico/flash.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <pico/util/queue.h>

#include "aprs.h"
#include "core1.h"
#include "pin_outs.h"
#include "payload_state_manager.h"
#include "payload_sensors.h"

#ifdef USE_TEST_DATA
#include "test_data.h"
#endif

int main()
{
  flash_safe_execute_core_init();
  multicore_lockout_victim_init();
  elijah_state_framework::init_usb_comm();

  watchdog_enable(30000, true);

  pin_init();

  elijah_state_framework::StateFrameworkLogger::init_driver_on_core();

  core1::launch_core1();
  queue_remove_blocking(&core1::core1_ready_queue, nullptr);

  payload_state_manager = new PayloadStateManager();

  constexpr uint8_t core_data = 0xBB;
  queue_add_blocking(&core1::core0_ready_queue, &core_data);

  sensors_init();

  gpio_put(LED_2_PIN, true);
  bool led_on = true;

  PayloadState state{{}};
  absolute_time_t next_update_time = nil_time;
  while (true)
  {
    next_update_time = delayed_by_ms(get_absolute_time(), 30);
    payload_state_manager->check_for_commands();

    bmp280->update(state);
    mpu6050->update(state);
    reliable_clock->update(state);

    state.bat_voltage = battery->get_voltage();
    state.bat_percent = battery->calc_charge_percent(state.bat_voltage) * 100;

#ifdef USE_TEST_DATA
    OVERWRITE_STATE_WITH_TEST_DATA()
#endif

    payload_state_manager->state_changed(state);
    payload_state_manager->check_for_log_write();

    gpio_put(LED_2_PIN, led_on = !led_on);

#ifdef USE_TEST_DATA
    INCREASE_TEST_DATA_IDX()
#endif

    watchdog_update();
    while (get_absolute_time() < next_update_time)
    {
      tight_loop_contents();
    }
  }
}
