#include <hardware/gpio.h>
#include <pico/flash.h>
#include <pico/multicore.h>

#include "core1.h"
#include "override_state_manager.h"
#include "sensors.h"

int main()
{
  flash_safe_execute_core_init();
  multicore_lockout_victim_init();

  pin_init();

  elijah_state_framework::StateFrameworkLogger::init_driver_on_core();

  core1::launch_core1();

  uint8_t core_data;
  queue_remove_blocking(&core1::core1_ready_queue, &core_data);
  override_state_manager = new OverrideStateManager();

  core_data = 0xBB;
  queue_add_blocking(&core1::core0_ready_queue, &core_data);

  sensors_init();

  OverrideState state{};

  while (true)
  {
    override_state_manager->check_for_commands();

    bmp280->update(state);
    mpu6050->update(state);

    state.bat_voltage = battery->get_voltage();
    state.bat_percent = battery->calc_charge_percent(state.bat_voltage) * 100;

    override_state_manager->state_changed(state);
    override_state_manager->check_for_log_write();
    sleep_ms(50);
  }
}
