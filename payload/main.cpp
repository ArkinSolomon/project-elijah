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
#include "elijah_state_framework.h"
#include "sensors/onboard_clock/onboard_clock.h"

#define APRS_FLAG 0x7E

int main()
{
  flash_safe_execute_core_init();
  multicore_lockout_victim_init();

  pin_init();

  elijah_state_framework::StateFrameworkLogger::init_driver_on_core();

  core1::launch_core1();

  queue_remove_blocking(&core1::core1_ready_queue, nullptr);
  payload_state_manager = new PayloadStateManager();

  constexpr uint8_t core_data = 0xBB;
  queue_add_blocking(&core1::core0_ready_queue, &core_data);

  gpio_put(LED_2_PIN, true);

  sensors_init();

  PayloadState state{{}};

  while (true)
  {
    payload_state_manager->check_for_commands();

    bmp280->update(state);
    int32_t p;
    double t, a;
    bmp280->get_bmp280().read_press_temp_alt(p, t, a, 101325);
    mpu6050->update(state);
    onboard_clock::clock_loop(state);

    /*payload_state_manager->log_message(std::format("bmp280: {}, uses i2c: {}, p: {}, t: {}, a: {}",
                                                   payload_state_manager->is_faulted(PayloadFaultKey::BMP280),
                                                   bmp280->get_bmp280().uses_i2c(), p, t, a));
                                                   */

    state.bat_voltage = battery->get_voltage();
    state.bat_percent = battery->calc_charge_percent(state.bat_voltage) * 100;

    payload_state_manager->state_changed(state);
    payload_state_manager->check_for_log_write();

    sleep_ms(50);
  }
}
