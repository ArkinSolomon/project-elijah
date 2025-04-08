#include "airbrake_controls.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <bits/stl_algo.h>
#include <hardware/gpio.h>

#include "airbrakes_state_manager.h"
#include "pin_outs.h"

double calculate_target_angle(double current_alt, double prev_alt, double init_alt, double init_vel,
                              const double curr_press, double T0, const double dt)
{
  // Constants
  constexpr double weight = 25.07f; // lb
  constexpr double mass = weight / 32.2f; // slug

  constexpr double L = .0065; // temperature lapse rate in Kelvin/m
  constexpr double R = 287.06; // specific gas constant J/(kg*K)
  T0 = T0 + 273.15; // C to K

  current_alt = current_alt * 3.28084; // ft
  prev_alt = prev_alt * 3.28084; // ft
  init_alt = init_alt * 3.28084; // ft
  init_vel = init_vel * 3.28084; // ft/s

  const double vel = (current_alt - prev_alt) / dt; // ft/s

  static bool did_matrix_init = false;
  static size_t chosen_trajectory_idx = 0;
  if (!did_matrix_init)
  {
    const uint8_t saved_trajectory = airbrakes_state_manager->get_persistent_storage()->get_uint8(
      AirbrakesPersistentKey::ChosenTrajectory);
    if ((saved_trajectory & 0x80) > 0)
    {
      chosen_trajectory_idx = saved_trajectory & 0x7F;
    }

    std::vector<double> cost;
    cost.reserve(5);

    for (size_t i = 0; i < H.size(); i++)
    {
      size_t min_idx = 0;
      double min_diff = std::numeric_limits<double>::max();
      for (size_t j = 0; j < H[i].size(); j++)
      {
        double curr_diff = std::abs(H[i][j] - init_alt);
        if (curr_diff < min_diff)
        {
          min_diff = curr_diff;
          min_idx = j;
        }
      }

      // double h_traj = H[i][min_idx];
      const double v_traj = V[i][min_idx];
      cost.push_back(std::pow(init_vel - v_traj, 2));
    }

    double min_cost = std::numeric_limits<double>::max();

    for (size_t i = 0; i < cost.size(); i++)
    {
      if (cost[i] < min_cost)
      {
        min_cost = cost[i];
        chosen_trajectory_idx = i;
      }
    }

    airbrakes_state_manager->log_message(std::format("Choosing airbrakes trajectory: {}", chosen_trajectory_idx));
    airbrakes_state_manager->get_persistent_storage()->set_uint8(AirbrakesPersistentKey::ChosenTrajectory,
                                                                 0x80 | static_cast<uint8_t>(chosen_trajectory_idx));
    airbrakes_state_manager->get_persistent_storage()->commit_data();
    did_matrix_init = true;
  }

  const std::vector<float>* alt_ideal = &H[chosen_trajectory_idx];
  const std::vector<float>* vel_ideal = &V[chosen_trajectory_idx];

  // Find ideal velocity at current altitude
  int i = 0;
  if (current_alt <= alt_ideal->front())
  {
    i = 0;
  }
  else if (current_alt >= alt_ideal->back())
  {
    i = alt_ideal->size() - 2;
  }
  else
  {
    for (i = alt_ideal->size() - 2; i >= 0; --i)
    {
      if (alt_ideal->at(i) <= current_alt && alt_ideal->at(i + 1) > current_alt)
      {
        break;
      }
    }
  }

  const double x1_vel = alt_ideal->at(i), x2_vel = alt_ideal->at(i + 1);
  const double y1_vel = vel_ideal->at(i), y2_vel = vel_ideal->at(i + 1);
  const double slope_vel = (y2_vel - y1_vel) / (x2_vel - x1_vel);
  const double vel_ideal_interp = y1_vel + slope_vel * (current_alt - x1_vel);

  // Velocity error and required deceleration
  const double vel_err = vel - vel_ideal_interp;
  const double tau = dt;
  const double req_decel = vel_err / tau;
  const double rho = curr_press / (R * (T0 - current_alt / 3.28084 * L)) / 515.4;
  const double cd_area_req = (2 * mass * req_decel) / (rho * std::pow(vel, 2));

  static std::vector<double> angle_range, cd_area_range;
  static bool did_ranges_init = false;
  if (!did_ranges_init)
  {
    // We want this on the heap because pico only has 4kb of stack, and we don't use them again
    const std::unique_ptr<std::vector<double>> cd_range(new std::vector<double>(angle_range.size()));
    const std::unique_ptr<std::vector<double>> area_range(new std::vector<double>(angle_range.size()));

    for (double a = 0; a <= 55.05; a += 0.1)
    {
      angle_range.push_back(a);
    }

    cd_area_range.reserve(angle_range.size());

    for (const auto& a : angle_range)
    {
      cd_range->push_back(
        0.00000011 * pow(a, 4) - 0.00001733 * pow(a, 3) + 0.00087041 * pow(a, 2) - 0.00884173 * a + 0.68);
    }

    for (const auto& a : angle_range)
    {
      area_range->push_back(-0.00001548 * pow(a, 2) + 0.00272439 * a + 0.09652135);
    }

    for (size_t ar_idx = 0; ar_idx < angle_range.size(); ++ar_idx)
    {
      cd_area_range.push_back(cd_range->at(ar_idx) * area_range->at(ar_idx));
    }

    did_ranges_init = true;
  }

  // Find desired angle
  if (cd_area_req <= cd_area_range.front())
  {
    i = 0;
  }
  else if (cd_area_req >= cd_area_range.back())
  {
    i = cd_area_range.size() - 2;
  }
  else
  {
    for (i = cd_area_range.size() - 2; i >= 0; --i)
    {
      if (cd_area_range[i] <= cd_area_req && cd_area_range[i + 1] > cd_area_req)
      {
        break;
      }
    }
  }

  const double x1_drag = cd_area_range[i];
  const double x2_drag = cd_area_range[i + 1];
  const double y1_drag = angle_range[i];
  const double y2_drag = angle_range[i + 1];
  const double slope_drag = (y2_drag - y1_drag) / (x2_drag - x1_drag);
  const double desired_angle = y1_drag + slope_drag * (cd_area_req - x1_drag);

  return std::clamp(desired_angle, 0.0, 55.0);
}

int32_t encoder_pos_from_angle(const double angle)
{
  return static_cast<int32_t>(3.3387 * angle - 9.3383);
}

double angle_from_encoder_pos(const int32_t encoderPos)
{
  return (encoderPos + 9.3383) / 3.3387;
}

void airbrakes_open()
{
  gpio_put(MOTOR_PIN_1, true);
  gpio_put(MOTOR_PIN_2, false);
}

void airbrakes_close()
{
  gpio_put(MOTOR_PIN_1, false);
  gpio_put(MOTOR_PIN_2, true);
}

void airbrakes_freeze()
{
  gpio_put(MOTOR_PIN_1, false);
  gpio_put(MOTOR_PIN_2, false);
}
