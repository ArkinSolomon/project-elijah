#include <hardware/gpio.h>
#include <pico/flash.h>
#include <pico/multicore.h>

#include "core1.h"
#include "airbrakes_state_manager.h"
#include "airbrake_controls.h"
#include "sensors.h"

#define AIBRAKES_TEST
#ifdef AIBRAKES_TEST
#include "test_data.h"
#endif

int main()
{
  flash_safe_execute_core_init();
  multicore_lockout_victim_init();

  busy_wait_ms(100);
  elijah_state_framework::init_usb_comm();

  pin_init();

  elijah_state_framework::StateFrameworkLogger::init_driver_on_core();

  core1::launch_core1();

  uint8_t core_data;
  queue_remove_blocking(&core1::core1_ready_queue, &core_data);
  airbrakes_state_manager = new AirbrakesStateManager();

  core_data = 0xBB;
  queue_add_blocking(&core1::core0_ready_queue, &core_data);

  sensors_init();

  gpio_put(LED_2_PIN, true);
  bool led_on = true;

  AirbrakesState state{};

  absolute_time_t last_calculated = get_absolute_time();
  bool entered_coast_phase = false;
  double initial_velocity = 0;
  double initial_altitude = 0;
  while (true)
  {
    airbrakes_state_manager->check_for_commands();

    double last_alt = state.altitude;

    bmp280->update(state);
    mpu6050->update(state);

    state.bat_voltage = battery->get_voltage();
    state.bat_percent = battery->calc_charge_percent(state.bat_voltage) * 100;

    const absolute_time_t current_time = get_absolute_time();
    const int64_t dt_us = absolute_time_diff_us(last_calculated, current_time);
    const double dt_ms = static_cast<double>(dt_us) / 1000.0;
    state.ms_since_last = dt_ms;

    int32_t new_target_pos;
    const bool is_coast_phase = airbrakes_state_manager->get_current_flight_phase() == StandardFlightPhase::COAST;
    if (is_coast_phase)
    {
      double new_target_angle;
#ifdef AIBRAKES_TEST
      if (test_data::curr_idx == test_data::curr_alt.size())
      {
        test_data::curr_idx = 3;
      }

      state.altitude = test_data::curr_alt[test_data::curr_idx];
      last_alt = test_data::curr_alt[test_data::curr_idx - 1];
      state.pressure = static_cast<int32_t>(std::round(test_data::curr_p[test_data::curr_idx]));
#endif

      if (!entered_coast_phase)
      {
        airbrakes_state_manager->log_message("Performing coast phase entrance calculations",
                                             elijah_state_framework::LogLevel::Debug);
        airbrakes_state_manager->lock_state_history();
        const std::deque<AirbrakesState>* history = &airbrakes_state_manager->get_state_history();
        initial_altitude = state.altitude;

        double tot_vel = 0;
        for (size_t i = 1; i < history->size(); ++i)
        {
          const AirbrakesState curr_state = history->at(i);
          const AirbrakesState prev_state = history->at(i - 1);

          tot_vel += (curr_state.altitude - prev_state.altitude) / curr_state.ms_since_last;
        }
        initial_velocity = tot_vel / history->size();
        airbrakes_state_manager->release_state_history();

#ifdef AIBRAKES_TEST
        initial_velocity = test_data::init_vel;
        initial_altitude = test_data::init_alt;
#endif

        entered_coast_phase = true;
        airbrakes_state_manager->log_message(std::format(
          "Coast phase entered with initial velocity {}m/s and altitude {}m", initial_velocity, initial_altitude));
      }

#ifndef AIBRAKES_TEST
      const double ground_temp  =airbrakes_state_manager->get_persistent_storage()->get_double(
              AirbrakesPersistentStateKey::GroundTemperature);
      const double dt_s = static_cast<double>(dt_ms) / 1000.0;
#else
      constexpr double ground_temp = test_data::ground_temp;
      constexpr double dt_s = test_data::dt_s;
      test_data::curr_idx++;
#endif
      new_target_angle = state.calculated_angle = calculate_target_angle(
        state.altitude, last_alt, initial_altitude, initial_velocity, state.pressure,
        ground_temp, dt_s
      );
#ifdef AIBRAKES_TEST
      airbrakes_state_manager->log_message(std::format(
                                             "calculate_target_angle({}, {}, {}, {}, {}, {}, {}) = {}",
                                             state.altitude, last_alt, initial_altitude, initial_velocity,
                                             state.pressure, ground_temp, dt_s, new_target_angle
                                           ), elijah_state_framework::LogLevel::Debug);
#endif
      new_target_pos = encoder_pos_from_angle(new_target_angle);
      if (abs(new_target_pos - state.calculated_encoder_pos) < 2)
      {
        new_target_pos = state.calculated_encoder_pos;
      }
    }
    else
    {
      state.calculated_angle = 0;
      new_target_pos = 0;
    }

    if (new_target_pos != state.calculated_encoder_pos && is_coast_phase)
    {
      if (queue_is_full(&core1::core0_ready_queue))
      {
        queue_remove_blocking(&core1::core0_ready_queue, nullptr);
      }
      queue_add_blocking(&core1::encoder_target_queue, &new_target_pos);
    }

    critical_section_enter_blocking(&core1::target_access_cs);
    state.target_encoder_pos = core1::target_encoder_pos;
    critical_section_exit(&core1::target_access_cs);

    last_calculated = current_time;

    critical_section_enter_blocking(&core1::encoder_pos_cs);
    state.curr_encoder_pos = core1::current_encoder_pos;
    critical_section_exit(&core1::encoder_pos_cs);

    airbrakes_state_manager->state_changed(state);
    airbrakes_state_manager->check_for_log_write();

    gpio_put(LED_2_PIN, led_on = !led_on);

    sleep_ms(40);
  }
}
