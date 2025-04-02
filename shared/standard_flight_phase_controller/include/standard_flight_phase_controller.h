#pragma once
#include <cmath>
#include <cstdint>
#include <deque>
#include <format>
#include <limits>
#include <string>

#include "elijah_state_framework.h"
#include "flight_phase_controller.h"
#include "usb_comm.h"

enum class StandardFlightPhase : uint8_t
{
  // Before the rocket goes to the launch pad
  PREFLIGHT = 1,

  // Ignition, motor burning
  LAUNCH = 2,

  // Motor done burning
  COAST = 3,

  // Apogee passed
  DESCENT = 4,

  // Rocket settled on ground
  LANDED = 5
};

template <typename TStateData>
class StandardFlightPhaseController : public elijah_state_framework::FlightPhaseController<
    TStateData, StandardFlightPhase>
{
public:
  StandardFlightPhaseController();

  [[nodiscard]] StandardFlightPhase initial_flight_phase() const override;
  [[nodiscard]] bool should_log(StandardFlightPhase current_phase) const override;

  [[nodiscard]] StandardFlightPhase update_phase(StandardFlightPhase current_phase,
                                                 const std::deque<TStateData>& state_history) override;
  [[nodiscard]] StandardFlightPhase predict_phase(StandardFlightPhase last_known_phase,
                                                  const std::deque<TStateData>& states) override;

  [[nodiscard]] std::string get_phase_name(StandardFlightPhase phase) const override;

  [[nodiscard]] double get_apogee() const;

protected:
  virtual void extract_state_data(TStateData state, double& accel_x, double& accel_y, double& accel_z,
                                  double& altitude) const = 0;

  [[nodiscard]] virtual bool is_calibrated() const = 0;

  double max_coast_alt = 0;
};

template <typename TStateData>
StandardFlightPhaseController<
  TStateData>::StandardFlightPhaseController(): elijah_state_framework::FlightPhaseController<
  TStateData, StandardFlightPhase>()
{
  max_coast_alt = std::numeric_limits<double>::min();
}

template <typename TStateData>
StandardFlightPhase StandardFlightPhaseController<TStateData>::initial_flight_phase() const
{
  return StandardFlightPhase::PREFLIGHT;
}

template <typename TStateData>
bool StandardFlightPhaseController<TStateData>::should_log(
  const StandardFlightPhase current_phase) const
{
  return current_phase != StandardFlightPhase::PREFLIGHT;
}

template <typename TStateData>
StandardFlightPhase StandardFlightPhaseController<TStateData>::update_phase(
  const StandardFlightPhase current_phase, const std::deque<TStateData>& state_history)
{
  if (!is_calibrated())
  {
    return StandardFlightPhase::PREFLIGHT;
  }

  double accel_x, accel_y, accel_z, altitude;
  extract_state_data(state_history.front(), accel_x, accel_y, accel_z, altitude);

  if (current_phase == StandardFlightPhase::PREFLIGHT)
  {
    if (altitude > 300)
    {
      return StandardFlightPhase::PREFLIGHT;
    }

    if (stdio_usb_connected() || state_history.size() < 50)
    {
      return StandardFlightPhase::PREFLIGHT;
    }

    // Once altitude increases by 30m, and an acceleration is recently greater than 50m/s^2
    if (altitude > 30)
    {
      for (const auto& state : state_history)
      {
        extract_state_data(state, accel_x, accel_y, accel_z, altitude);
        const double accel_mag = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
        if (accel_mag > 50)
        {
          return StandardFlightPhase::LAUNCH;
        }
      }
      return StandardFlightPhase::PREFLIGHT;
    }
    return StandardFlightPhase::PREFLIGHT;
  }
  else if (current_phase == StandardFlightPhase::LAUNCH)
  {
    if (state_history.size() < 25)
    {
      return StandardFlightPhase::LAUNCH;
    }

    uint8_t missCount = 0;
    double last_accel = std::numeric_limits<double>::max();
    auto begin = std::begin(state_history);
    auto end = std::begin(state_history) + 25;
    if (std::distance(begin, end) > state_history.size())
      end = std::end(state_history);

    for (auto it = begin; it != end; ++it)
    {
      double curr_accel_x, curr_accel_y, curr_accel_z, curr_altitude;
      extract_state_data(*it, curr_accel_x, curr_accel_y, curr_accel_z, curr_altitude);
      const double curr_accel = sqrt(
        curr_accel_x * curr_accel_x + curr_accel_y * curr_accel_y + curr_accel_z * curr_accel_z);
      if (curr_accel < last_accel)
      {
        last_accel = curr_accel;
      }
      else
      {
        missCount++;
      }

      // Once rocket has had consistently decreasing acceleration (no more than 2 non-decreasing)
      if (missCount > 2)
      {
        return StandardFlightPhase::LAUNCH;
      }
    }

    return StandardFlightPhase::COAST;
  }
  else if (current_phase == StandardFlightPhase::COAST)
  {
    if (altitude > max_coast_alt)
    {
      max_coast_alt = altitude;
    }

    // Once rocket dropped 50m
    const double diff_from_apogee = max_coast_alt - altitude;
    if (diff_from_apogee > 50)
    {
      elijah_state_framework::log_serial_message(std::format(
        "Apogee reached! {}m, detected because of an altitude difference of {}m", max_coast_alt, diff_from_apogee));
      return StandardFlightPhase::DESCENT;
    }
    return StandardFlightPhase::COAST;
  }
  else if (current_phase == StandardFlightPhase::DESCENT)
  {
    double minRecentAlt = std::numeric_limits<double>::max();
    double maxRecentAlt = std::numeric_limits<double>::min();;

    for (TStateData state : state_history)
    {
      double curr_accel_x, curr_accel_y, curr_accel_z, curr_altitude;
      extract_state_data(state, curr_accel_x, curr_accel_y, curr_accel_z, curr_altitude);

      if (curr_altitude < minRecentAlt)
      {
        minRecentAlt = curr_altitude;
      }

      if (curr_altitude > maxRecentAlt)
      {
        maxRecentAlt = curr_altitude;
      }
    }

    // If recently there is a total deviation of less than 3m
    double recent_deviation = std::abs(maxRecentAlt - minRecentAlt);
    if (recent_deviation <= 3)
    {
      elijah_state_framework::log_serial_message(std::format("Landed because recent deviation = {}m ({} states)",
                                                             recent_deviation, state_history.size()));
      return StandardFlightPhase::LANDED;
    }
    return StandardFlightPhase::DESCENT;
  }
  else if (current_phase == StandardFlightPhase::LANDED)
  {
    return StandardFlightPhase::LANDED;
  }

  return current_phase;
}

template <typename TStateData>
StandardFlightPhase StandardFlightPhaseController<TStateData>::predict_phase(const StandardFlightPhase last_known_phase,
                                                                             const std::deque<TStateData>& states)
{
  if (last_known_phase != StandardFlightPhase::PREFLIGHT && last_known_phase != StandardFlightPhase::LANDED)
  {
    double first_ax, first_ay, first_az, first_alt;
    extract_state_data(states[0], first_ax, first_ay, first_az, first_alt);

    double last_ax, last_ay, last_az, last_alt;
    extract_state_data(states[states.size() - 1], last_ax, last_ay, last_az, last_alt);

    // If there's a deviation of < 3m assume we landed
    if (std::abs(first_alt - last_alt) < 3)
    {
      return StandardFlightPhase::LANDED;
    }

    if (first_alt > last_alt)
    {
      return StandardFlightPhase::COAST;
    }
    else
    {
      return StandardFlightPhase::DESCENT;
    }
  }
  return last_known_phase;
}

template <typename TStateData>
std::string StandardFlightPhaseController<TStateData>::get_phase_name(const StandardFlightPhase phase) const
{
  switch (phase)
  {
  case StandardFlightPhase::PREFLIGHT:
    return "Preflight";
  case StandardFlightPhase::LAUNCH:
    return "Launch";
  case StandardFlightPhase::COAST:
    return "Coast";
  case StandardFlightPhase::DESCENT:
    return "Descent";
  case StandardFlightPhase::LANDED:
    return "Landed";
  }
  return "Unknown";
}

template <typename TStateData>
double StandardFlightPhaseController<TStateData>::get_apogee() const
{
  return max_coast_alt;
}
