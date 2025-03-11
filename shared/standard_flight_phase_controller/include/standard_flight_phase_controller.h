#pragma once
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>

#include "flight_phase_controller.h"

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
                                                  const std::deque<TStateData>& states) const override;

  [[nodiscard]] std::string get_phase_name(StandardFlightPhase phase) const override;

protected:
  virtual void extract_state_data(TStateData state, double& accel_x, double& accel_y, double& accel_z,
                                  double& altitude) const = 0;

private:
  double min_preflight_alt, max_preflight_accel;

  double max_coast_alt;
};

template <typename TStateData>
StandardFlightPhaseController<TStateData>::StandardFlightPhaseController()
{
  min_preflight_alt = std::numeric_limits<double>::max();
  max_preflight_accel = std::numeric_limits<double>::min();

  max_coast_alt = std::numeric_limits<double>::min();
}

template <typename TStateData>
StandardFlightPhase StandardFlightPhaseController<TStateData>::initial_flight_phase() const
{
  return StandardFlightPhase::PREFLIGHT;
}

template <typename TStateData>
bool StandardFlightPhaseController<TStateData>::should_log(const StandardFlightPhase current_phase) const
{
  return current_phase != StandardFlightPhase::PREFLIGHT;
}

template <typename TStateData>
StandardFlightPhase StandardFlightPhaseController<TStateData>::update_phase(
  const StandardFlightPhase current_phase, const std::deque<TStateData>& state_history)
{
  double accel_x, accel_y, accel_z, altitude;
  extract_state_data(state_history.front(), accel_x, accel_y, accel_z, altitude);
  const double accel = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

  if (current_phase == StandardFlightPhase::PREFLIGHT)
  {
    if (altitude < min_preflight_alt)
    {
      min_preflight_alt = altitude;
    }

    if (accel > max_preflight_accel)
    {
      max_preflight_accel = accel;
    }

    // Once altitude increases by 30m overall, and an acceleration at some point of greater than 50m/s^2 was reached
    if (altitude - min_preflight_alt > 30 && max_preflight_accel > 50)
    {
      return StandardFlightPhase::LAUNCH;
    }
    return StandardFlightPhase::PREFLIGHT;
  }
  else if (current_phase == StandardFlightPhase::LAUNCH)
  {
    if (state_history.size() < 3)
    {
      return StandardFlightPhase::LAUNCH;
    }

    uint8_t missCount = 0;
    double last_accel = std::numeric_limits<double>::max();
    for (TStateData state : state_history)
    {
      double curr_accel_x, curr_accel_y, curr_accel_z, curr_altitude;
      extract_state_data(state, curr_accel_x, curr_accel_y, curr_accel_z, curr_altitude);
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
    if (max_coast_alt - altitude > 50)
    {
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
    if (std::abs(maxRecentAlt - minRecentAlt) <= 3)
    {
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
StandardFlightPhase StandardFlightPhaseController<TStateData>::predict_phase(StandardFlightPhase last_known_phase,
                                                                             const std::deque<TStateData>& states) const
{
  return last_known_phase;
}

template <typename TStateData>
std::string StandardFlightPhaseController<TStateData>::get_phase_name(StandardFlightPhase phase) const
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
