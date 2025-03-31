#include <format>
#include <sd_card.h>
#include <hardware/clocks.h>

#include <pico/flash.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <pico/util/queue.h>

#include "aprs.h"
#include "core1.h"
#include "pin_outs.h"
#include "payload_state_manager.h"
#include "elijah_state_framework.h"

int main()
{
  set_sys_clock_khz(SYS_CLK_KHZ, true);

  flash_safe_execute_core_init();
  multicore_lockout_victim_init();

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
  while (true)
  {
    payload_state_manager->check_for_commands();

    absolute_time_t before_sensors = get_absolute_time();
    bmp280->update(state);
    mpu6050->update(state);
    reliable_clock->update(state);
    absolute_time_t reliable_clock_t = get_absolute_time();
    payload_state_manager->log_message(std::format("Command check: {}us", absolute_time_diff_us(before_sensors, reliable_clock_t)), elijah_state_framework::LogLevel::Debug);

    /*payload_state_manager->log_message(std::format("bmp280: {}, uses i2c: {}, p: {}, t: {}, a: {}",
                                                   payload_state_manager->is_faulted(PayloadFaultKey::BMP280),
                                                   bmp280->get_bmp280().uses_i2c(), p, t, a));
                                                   */

    state.bat_voltage = battery->get_voltage();
    state.bat_percent = battery->calc_charge_percent(state.bat_voltage) * 100;

    absolute_time_t before_sc = get_absolute_time();
    payload_state_manager->state_changed(state);
    absolute_time_t state_change_t = get_absolute_time();

    payload_state_manager->log_message(std::format("State change: {}us", absolute_time_diff_us(before_sc, state_change_t)), elijah_state_framework::LogLevel::Debug);

    gpio_put(LED_2_PIN, led_on = !led_on);
    sleep_ms(50);
  }
}
