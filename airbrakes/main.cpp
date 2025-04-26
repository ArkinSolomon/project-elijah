#include <hardware/gpio.h>
#include <pico/flash.h>
#include <pico/multicore.h>

#include "core1.h"
#include "airbrakes_state_manager.h"
#include "airbrake_controls.h"
#include "airbrakes_sensors.h"

#ifdef USE_TEST_DATA
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
  absolute_time_t next_update_time = nil_time;

  double additional_ax_calib = airbrakes_state_manager->get_persistent_storage()->get_double(
    AirbrakesPersistentKey::AdditionalAccelXCalib);
  double additional_ay_calib = airbrakes_state_manager->get_persistent_storage()->get_double(
    AirbrakesPersistentKey::AdditionalAccelYCalib);
  double additional_az_calib = airbrakes_state_manager->get_persistent_storage()->get_double(
    AirbrakesPersistentKey::AdditionalAccelZCalib);

  // We always expect a 1g acceleration by default, so this shouldn't be true unless additional calibration has occurred
  bool has_additional_calib = additional_ax_calib != 0 && additional_ay_calib != 0 && additional_az_calib != 0;

  bool entered_coast_phase = false;
  double initial_velocity = 0;
  double initial_altitude = 0;

  while (true)
  {
    next_update_time = delayed_by_ms(get_absolute_time(), 30);
    airbrakes_state_manager->check_for_commands();

    double last_modified_az = state.modified_accel_z;

    const bool is_coast_phase = airbrakes_state_manager->get_current_flight_phase() ==
      elijah_state_framework::std_helpers::StandardFlightPhase::COAST;

    bmp280->update(state);

    const absolute_time_t current_time = get_absolute_time();
    const int64_t dt_us = absolute_time_diff_us(last_calculated, current_time);
    const double dt_ms = static_cast<double>(dt_us) / 1000.0;
    double ms_since_last = dt_ms;

    const double last_ax = state.accel_x;
    const double last_ay = state.accel_y;
    const double last_az = state.accel_z;
    const double last_accel_mag = std::sqrt(last_ax * last_ax + last_ay * last_ay + last_az * last_az);
    mpu6050->update(state);

#ifdef USE_TEST_DATA
    OVERWRITE_STATE_WITH_TEST_DATA();
    if (test_data::test_data_enable)
    {
      ms_since_last = test_data::dt[test_data::curr_idx] * 1000.0f;
    }
#endif
    state.ms_since_last = ms_since_last;

    const double accel_mag = std::sqrt(
      state.accel_x * state.accel_x + state.accel_y * state.accel_y + state.accel_z * state.accel_z);

    if (is_coast_phase && last_accel_mag > 100 && accel_mag > 2 * last_accel_mag)
    {
      airbrakes_state_manager->log_message(
        std::format("Acceleration doubled in one time step from last {}m/s^2 to {}m/s^2 (and last acceleration magnitude was > 100m/s^2)", last_accel_mag, accel_mag),
        elijah_state_framework::LogLevel::Warning);
      state.accel_x = last_ax;
      state.accel_y = last_ay;
      state.accel_z = last_az;
    }

    state.bat_voltage = battery->get_voltage();
    state.bat_percent = battery->calc_charge_percent(state.bat_voltage) * 100;

    const bool is_launch_phase = airbrakes_state_manager->get_current_flight_phase() ==
      elijah_state_framework::std_helpers::StandardFlightPhase::LAUNCH;
    const bool is_preflight_phase = airbrakes_state_manager->get_current_flight_phase() ==
      elijah_state_framework::std_helpers::StandardFlightPhase::PREFLIGHT;

    if (is_preflight_phase)
    {
      did_choose_trajectory = false;
    }

    if (is_launch_phase && !has_additional_calib)
    {
      airbrakes_state_manager->lock_state_history();
      auto rit = airbrakes_state_manager->get_state_history().rbegin();
      auto end_it = airbrakes_state_manager->get_state_history().rend();

      size_t accel_samples = 0;
      size_t launch_point_off = 0;
      for (auto it = rit; it != end_it; ++it, ++accel_samples)
      {
        const AirbrakesState& eval_state = *it;
        const double eval_accel_mag = sqrt(
          eval_state.accel_x * eval_state.accel_x + eval_state.accel_y * eval_state.accel_y + eval_state.accel_z *
          eval_state.accel_z);
        if (eval_accel_mag > 20)
        {
          launch_point_off = accel_samples - 2;
          // Have a slight buffer so that we don't get too close to launch
          accel_samples = accel_samples < 6 ? 1 : accel_samples - 5;
          break;
        }
      }

      if (accel_samples == airbrakes_state_manager->get_state_history().size())
      {
        airbrakes_state_manager->log_message("No launch sample detected when recalibrating",
                                             elijah_state_framework::LogLevel::Warning);
      }

      rit = airbrakes_state_manager->get_state_history().rbegin();
      double avg_ax = 0, avg_ay = 0, avg_az = 0;
      for (size_t i = 0; i < accel_samples; ++i)
      {
        const AirbrakesState& eval_state = *(rit + i);
        avg_ax += eval_state.accel_x;
        avg_ay += eval_state.accel_y;
        avg_az += eval_state.accel_z;
      }
      avg_ax /= accel_samples;
      avg_ay /= accel_samples;
      avg_az /= accel_samples;

      additional_ax_calib = -avg_ax;
      additional_ay_calib = -avg_ay;
      additional_az_calib = -avg_az;
      airbrakes_state_manager->log_message(std::format(
        "Additional calibration values calculated from {} samples: x = {}, y = {}, z = {} (all m/s^2)",
        accel_samples, additional_ax_calib, additional_ay_calib, additional_az_calib));
      airbrakes_state_manager->get_persistent_storage()->set_double(AirbrakesPersistentKey::AdditionalAccelXCalib,
                                                                    additional_ax_calib);
      airbrakes_state_manager->get_persistent_storage()->set_double(AirbrakesPersistentKey::AdditionalAccelYCalib,
                                                                    additional_ay_calib);
      airbrakes_state_manager->get_persistent_storage()->set_double(AirbrakesPersistentKey::AdditionalAccelZCalib,
                                                                    additional_az_calib);
      airbrakes_state_manager->get_persistent_storage()->commit_data();
      last_modified_az += additional_ax_calib;

      rit = airbrakes_state_manager->get_state_history().rbegin();
      state.velocity = 0;
      for (size_t i = launch_point_off; i < airbrakes_state_manager->get_state_history().size(); ++i)
      {
        const AirbrakesState& curr_state = *(rit + i);
        const AirbrakesState& prev_state = *(rit + i - 1);

        airbrakes_state_manager->log_message(
          std::format("Integrating {} += ({} + {} + 2 * {}) * {} / 2", state.velocity, curr_state.accel_z,
                      prev_state.accel_z, additional_az_calib, curr_state.ms_since_last / 1000),
          elijah_state_framework::LogLevel::Debug);
        state.velocity += (curr_state.accel_z + prev_state.accel_z + 2 * additional_az_calib) * (curr_state.
            ms_since_last / 1000)
          / 2;
      }

      airbrakes_state_manager->log_message(std::format(
        "Retroactivly calculated velocity from integration: {} m/s (without current state), by integrating {} states", state.velocity,
        airbrakes_state_manager->get_state_history().size() - launch_point_off));

      airbrakes_state_manager->release_state_history();
      has_additional_calib = true;
    }
    else if (is_preflight_phase && has_additional_calib)
    {
      airbrakes_state_manager->get_persistent_storage()->set_double(AirbrakesPersistentKey::AdditionalAccelXCalib, 0);
      airbrakes_state_manager->get_persistent_storage()->set_double(AirbrakesPersistentKey::AdditionalAccelYCalib, 0);
      airbrakes_state_manager->get_persistent_storage()->set_double(AirbrakesPersistentKey::AdditionalAccelZCalib, 0);
      airbrakes_state_manager->get_persistent_storage()->commit_data();

      additional_ax_calib = 0;
      additional_ay_calib = 0;
      additional_az_calib = 0;
      state.velocity = 0;
      has_additional_calib = false;
    }

    int32_t new_target_pos;
    state.modified_accel_x = state.accel_x + additional_ax_calib;
    state.modified_accel_y = state.accel_y + additional_ay_calib;
    state.modified_accel_z = state.accel_z + additional_az_calib;
    if (is_launch_phase || is_coast_phase)
    {
      state.velocity += (state.modified_accel_z + last_modified_az) * (state.ms_since_last / 1000) / 2;
      airbrakes_state_manager->log_message(std::format("az {} m/s^2, az+: {} m/s^2, vel: {} m/s, expect: {} m/s, dt: {} s",test_data::accel_z[test_data::curr_idx], state.modified_accel_z, state.velocity,
                                                       test_data::vel_expect[test_data::curr_idx], state.ms_since_last / 1000));
    }

    if (is_coast_phase)
    {
      double new_target_angle;
      if (!entered_coast_phase)
      {
        initial_velocity = state.velocity;
        initial_altitude = state.altitude;

        airbrakes_state_manager->get_persistent_storage()->set_uint8(AirbrakesPersistentKey::ChosenTrajectory, 0);
        airbrakes_state_manager->get_persistent_storage()->commit_data();

        entered_coast_phase = true;
        airbrakes_state_manager->log_message(std::format(
          "Coast phase entered with initial velocity {}m/s and altitude {}m", initial_velocity, initial_altitude));
      }

#ifndef USE_TEST_DATA
      const double ground_temp = airbrakes_state_manager->get_persistent_storage()->get_double(
        AirbrakesPersistentKey::GroundTemperature);
      const double dt_s = static_cast<double>(dt_ms) / 1000.0;
#else
      constexpr double ground_temp = test_data::ground_temp;
#endif
      new_target_angle = state.calculated_angle = calculate_target_angle(
        state.altitude, state.velocity, initial_altitude, initial_velocity, state.pressure,
        ground_temp, state.ms_since_last / 1000.0
      );

      airbrakes_state_manager->log_message(std::format(
        "calculate_target_angle({}, {}, {}, {}, {}, {}, {}) = {}",
        state.altitude, state.velocity, initial_altitude, initial_velocity, state.pressure,
        ground_temp, state.ms_since_last / 1000.0, new_target_angle
      ));

      new_target_pos = encoder_pos_from_angle(new_target_angle);
      state.calculated_encoder_pos = new_target_pos;
    }
    else
    {
      state.calculated_angle = 0;
      state.calculated_encoder_pos = 0;
      new_target_pos = 0;
    }

    if (is_coast_phase)
    {
      if (queue_is_full(&core1::encoder_target_queue))
      {
        queue_remove_blocking(&core1::encoder_target_queue, nullptr);
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
#ifdef USE_TEST_DATA
    INCREASE_TEST_DATA_IDX()
#endif

    while (get_absolute_time() < next_update_time)
    {
      tight_loop_contents();
    }
  }
}
