#include "airbrake_controls.h"

#include <cmath>
#include <cstdint>
#include <bits/stl_algo.h>
#include <hardware/gpio.h>

#include "pin_outs.h"

double calculate_airbrake_target_angle(const double current_pressure, const double prev_pressure,
                                       const double ground_pressure, double ground_temperature, const double dt)
{
  constexpr double weight = 24.732f; // lb
  constexpr double mass = weight / 32.2f; // slug
  constexpr double g = 32.2f; // ft/s^2
  constexpr double R = 1716.46f; // (ft*lb)/(slug*R)
  constexpr double lapse = 0.0033f; // Temperature lapse rate Rankine/ft
  constexpr double pa_ft = 0.020887542f; // pa to lb/ft^2

  ground_temperature = ground_temperature * 9.0f / 5.0f + 32.0f + 459.67f;

  // Compute altitude
  const double current_alt = (1 - std::pow(current_pressure / ground_pressure, 0.190284f)) * 145366.45f;
  const double prev_alt = (1 - std::pow(prev_pressure / ground_pressure, 0.190284f)) * 145366.45f;

  // Calculate velocity
  const double vel = (current_alt - prev_alt) / dt;

  // Find ideal velocity at current altitude
  int i = 0;
  if (current_alt <= alt_ideal.front())
  {
    i = 0;
  }
  else if (current_alt >= alt_ideal.back())
  {
    i = static_cast<int>(alt_ideal.size()) - 2;
  }
  else
  {
    for (i = static_cast<int>(alt_ideal.size()) - 2; i >= 0; --i)
    {
      if (alt_ideal[i] <= current_alt && alt_ideal[i + 1] > current_alt)
      {
        break;
      }
    }
  }

  double x1 = alt_ideal[i], x2 = alt_ideal[i + 1];
  double y1 = vel_ideal[i], y2 = vel_ideal[i + 1];
  double slope = (y2 - y1) / (x2 - x1);
  const double vel_ideal_interp = y1 + slope * (current_alt - x1);

  // Velocity error and required deceleration
  const double vel_err = vel - vel_ideal_interp;
  const double tau = dt;
  const double req_decel = vel_err / tau;
  const double rho = (current_pressure * pa_ft) / (R * (ground_temperature - current_alt * lapse));
  const double cd_area_req = (2 * mass * req_decel) / (rho * vel * vel);

  static std::vector<double> angle_range;
  static std::vector<double> cd_range;
  static std::vector<double> area_range;
  static std::vector<double> cd_area_range;
  if (angle_range.empty())
  {
    for (double a = 0; a <= 55.08; a += 0.1)
    {
      angle_range.push_back(a);
    }

    cd_range.reserve(angle_range.size());
    for (const auto& a : angle_range)
    {
      cd_range.push_back(
        0.00000011 * pow(a, 4) - 0.00001733 * pow(a, 3) + 0.00087041 * pow(a, 2) - 0.00884173 * a + 0.68);
    }

    area_range.reserve(angle_range.size());
    for (const auto& a : angle_range)
    {
      area_range.push_back(-0.00001793 * pow(a, 2) + 0.00306557 * a + 0.08801463);
    }

    cd_area_range.reserve(angle_range.size());
    for (unsigned int i = 0; i < angle_range.size(); ++i)
    {
      cd_area_range.push_back(cd_range[i] * area_range[i]);
    }
  }

  if (cd_area_req <= cd_area_range.front())
  {
    i = 0;
  }
  else if (cd_area_req >= cd_area_range.back())
  {
    i = static_cast<int>(cd_area_range.size()) - 2;
  }
  else
  {
    for (i = static_cast<int>(cd_area_range.size()) - 2; i >= 0; --i)
    {
      if (cd_area_range[i] <= cd_area_req && cd_area_range[i + 1] > cd_area_req)
      {
        break;
      }
    }
  }

  x1 = cd_area_range[i];
  x2 = cd_area_range[i + 1];
  y1 = angle_range[i];
  y2 = angle_range[i + 1];
  slope = (y2 - y1) / (x2 - x1);

  const double desiredAngle = y1 + slope * (cd_area_req - x1);
  return std::clamp(desiredAngle, 0.0, 55.0);
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
