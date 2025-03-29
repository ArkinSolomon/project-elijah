#include <hardware/gpio.h>
#include <pico/flash.h>
#include <pico/multicore.h>

#include "core1.h"
#include "airbrakes_state_manager.h"
#include "airbrake_controls.h"
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
  airbrakes_state_manager = new AirbrakesStateManager();

  core_data = 0xBB;
  queue_add_blocking(&core1::core0_ready_queue, &core_data);

  sensors_init();

  AirbrakesState state{};

  absolute_time_t last_calculated = get_absolute_time();
  while (true)
  {
    airbrakes_state_manager->check_for_commands();

    int32_t last_pressure = state.pressure;

    bmp280->update(state);
    mpu6050->update(state);

    state.bat_voltage = battery->get_voltage();
    state.bat_percent = battery->calc_charge_percent(state.bat_voltage) * 100;

    const absolute_time_t current_time = get_absolute_time();
    double new_target_angle;
    int32_t new_target_pos;
    const bool is_coast_phase = airbrakes_state_manager->get_current_flight_phase() == StandardFlightPhase::COAST;
    if (is_coast_phase)
    {
      const int64_t dt_us = absolute_time_diff_us(last_calculated, current_time);
      new_target_angle = state.calculated_angle = calculate_airbrake_target_angle(
        state.pressure, last_pressure,
        airbrakes_state_manager->get_persistent_data_storage()
                               ->get_double(AirbrakesPersistentStateKey::GroundPressure),
        airbrakes_state_manager->get_persistent_data_storage()
                               ->get_double(AirbrakesPersistentStateKey::GroundTemperature),
        dt_us / 1000
      );

      new_target_pos = encoder_pos_from_angle(new_target_angle);
      if (abs(new_target_pos - state.target_encoder_pos) < 5)
      {
        new_target_pos = state.target_encoder_pos;
        new_target_angle = state.target_angle;
      }
    }
    else
    {
      state.calculated_angle = 0;
      new_target_pos = 0;
      new_target_angle = 0;
    }

    if (new_target_pos != state.target_encoder_pos && is_coast_phase)
    {
      if (queue_is_full(&core1::core0_ready_queue))
      {
        queue_remove_blocking(&core1::core0_ready_queue, nullptr);
      }
      queue_add_blocking(&core1::encoder_target_queue, &new_target_pos);
    }

    state.target_encoder_pos = new_target_pos;
    state.target_angle = new_target_angle;

    last_calculated = current_time;

    critical_section_enter_blocking(&core1::encoder_pos_cs);
    state.curr_encoder_pos = core1::current_encoder_pos;
    critical_section_exit(&core1::encoder_pos_cs);
    state.curr_angle = angle_from_encoder_pos(state.curr_encoder_pos);

    airbrakes_state_manager->state_changed(state);
    airbrakes_state_manager->check_for_log_write();

    sleep_ms(50);
  }
}
