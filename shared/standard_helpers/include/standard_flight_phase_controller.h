#pragma once
#include <cmath>
#include <cstdint>
#include <deque>
#include <format>
#include <limits>
#include <string>

#include "elijah_state_framework.h"
#include "flight_phase_controller.h"

#define STANDARD_STATE_COLLECTION_DATA \
int32_t pressure; \
double temperature; \
double altitude; \
double accel_x, accel_y, accel_z; \
double gyro_x, gyro_y, gyro_z;

#define RECENT_ALT_MEDIAN_COUNT 5
#define REQ_COAST_PHASE_ALT_M 350
#define MOTOR_BURNOUT_TIME_MS 1400
#define REQ_APOGEE_DROP_M 30

#define MAX_LAND_ALT_DEVIATION_M 3

namespace elijah_state_framework::std_helpers
{
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
  class StandardFlightPhaseController : public FlightPhaseController<TStateData, StandardFlightPhase>
  {
  public:
    StandardFlightPhaseController();

    [[nodiscard]] StandardFlightPhase initial_flight_phase() const override;

    StandardFlightPhase force_next_phase(StandardFlightPhase current_phase, TStateData current_state) override;
    StandardFlightPhase update_phase(StandardFlightPhase current_phase,
                                     const std::deque<TStateData>& state_history) override;
    StandardFlightPhase predict_phase(StandardFlightPhase last_known_phase,
                                      const std::deque<TStateData>& states) override;

    [[nodiscard]] std::string get_phase_name(StandardFlightPhase phase) const override;
    [[nodiscard]] size_t get_speaker_pattern(StandardFlightPhase phase, const uint16_t*& pattern_freqs,
                                             const uint16_t*& pattern_timing_ms) const override;

    [[nodiscard]] double get_apogee() const;

  protected:
    [[nodiscard]] virtual bool is_calibrated() const = 0;
    virtual void log_message(const std::string& msg) const = 0;
    virtual void set_apogee(double apogee) const = 0;

    absolute_time_t burnout_time = nil_time;
    absolute_time_t coast_enter_time = nil_time;
    double max_coast_alt = std::numeric_limits<double>::lowest();

    void reset_data();

  private:
    static constexpr size_t preflight_audio_len = 2;
    static constexpr uint16_t preflight_audio_freqs[preflight_audio_len] = {280, 0};
    static constexpr uint16_t preflight_audio_timing_ms[preflight_audio_len] = {250, 10000};

    static constexpr size_t launch_audio_len = 4;
    static constexpr uint16_t launch_audio_freqs[launch_audio_len] = {333, 1000, 200, 0};
    static constexpr uint16_t launch_audio_timing_ms[launch_audio_len] = {400, 50, 100, 400};

    static constexpr size_t coast_audio_len = 5;
    static constexpr uint16_t coast_audio_freqs[coast_audio_len] = {220, 800, 8500, 40, 0};
    static constexpr uint16_t coast_audio_timing_ms[coast_audio_len] = {100, 100, 50, 130, 5000};

    static constexpr size_t descent_audio_len = 12;
    static constexpr uint16_t descent_audio_freqs[descent_audio_len] = {
      1000, 900, 800, 700, 600, 500, 400, 300, 200, 100, 50, 0
    };
    static constexpr uint16_t descent_audio_timing_ms[descent_audio_len] = {
      50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 7000
    };

    static constexpr size_t landed_audio_len = 9;
    static constexpr uint16_t landed_audio_freqs[landed_audio_len] = {220, 98, 174, 146, 220, 196, 130, 65, 0};
    static constexpr uint16_t landed_audio_timing_ms[landed_audio_len] = {
      500, 250, 500, 500, 250, 250, 250, 250, 15000
    };

    static constexpr size_t bad_audio_len = 2;
    static constexpr uint16_t bad_audio_freqs[bad_audio_len] = {220, 100};
    static constexpr uint16_t bad_audio_timing_ms[bad_audio_len] = {250, 100};
  };
}

template <typename TStateData>
elijah_state_framework::std_helpers::StandardFlightPhaseController<
  TStateData>::StandardFlightPhaseController(): FlightPhaseController<
  TStateData, StandardFlightPhase>()
{
  reset_data();
}

template <typename TStateData>
elijah_state_framework::std_helpers::StandardFlightPhase
elijah_state_framework::std_helpers::StandardFlightPhaseController<TStateData>::initial_flight_phase() const
{
  return StandardFlightPhase::PREFLIGHT;
}

template <typename TStateData>
elijah_state_framework::std_helpers::StandardFlightPhase
elijah_state_framework::std_helpers::StandardFlightPhaseController<TStateData>::force_next_phase(
  StandardFlightPhase current_phase, TStateData current_state)
{
  if (current_phase == StandardFlightPhase::LANDED)
  {
    reset_data();
    set_apogee(std::numeric_limits<double>::lowest());
    return StandardFlightPhase::PREFLIGHT;
  }

  const auto next_phase = static_cast<StandardFlightPhase>(static_cast<std::underlying_type_t<StandardFlightPhase>>(
    current_phase) + 1);

  if (next_phase == StandardFlightPhase::LAUNCH)
  {
    burnout_time = delayed_by_ms(get_absolute_time(), MOTOR_BURNOUT_TIME_MS);
  }
  if (next_phase == StandardFlightPhase::COAST)
  {
    coast_enter_time = get_absolute_time();
  }
  else if (next_phase == StandardFlightPhase::DESCENT)
  {
    max_coast_alt = current_state.altitude;
  }
  return next_phase;
}

template <typename TStateData>
elijah_state_framework::std_helpers::StandardFlightPhase
elijah_state_framework::std_helpers::StandardFlightPhaseController<TStateData>::update_phase(
  const StandardFlightPhase current_phase, const std::deque<TStateData>& state_history)
{
  if (state_history.size() < RECENT_ALT_MEDIAN_COUNT)
  {
    return current_phase;
  }

  if (!is_calibrated())
  {
    reset_data();
    return StandardFlightPhase::PREFLIGHT;
  }

  double recent_alts_sorted[RECENT_ALT_MEDIAN_COUNT];
  for (size_t i = 0; i < RECENT_ALT_MEDIAN_COUNT; i++)
  {
    recent_alts_sorted[i] = state_history.at(i).altitude;
  }
  std::sort(std::begin(recent_alts_sorted), std::begin(recent_alts_sorted) + RECENT_ALT_MEDIAN_COUNT);
  constexpr size_t median_idx = std::floor(RECENT_ALT_MEDIAN_COUNT / 2);
  const double median_alt = recent_alts_sorted[median_idx];

  if (current_phase == StandardFlightPhase::PREFLIGHT)
  {
    if (
#ifndef USE_TEST_DATA
      stdio_usb_connected() ||
#endif
      state_history.size() < RECENT_ALT_MEDIAN_COUNT * 2)
    {
      return StandardFlightPhase::PREFLIGHT;
    }

    if (median_alt > REQ_COAST_PHASE_ALT_M)
    {
      coast_enter_time = get_absolute_time();
      log_message(std::format(
        "Coast phase entered from preflight phase due to recent median altitude, altitude ({}m) > {}m",
        median_alt,
        REQ_COAST_PHASE_ALT_M));
      return StandardFlightPhase::COAST;
    }

    // Once altitude increases by 30m, and an acceleration is recently greater than 50m/s^2
    if (median_alt > 30)
    {
      for (const auto& state : state_history)
      {
        const double accel_mag = sqrt(
          state.accel_x * state.accel_x + state.accel_y * state.accel_y + state.accel_z * state.accel_z);
        if (accel_mag > 50)
        {
          burnout_time = delayed_by_ms(get_absolute_time(), MOTOR_BURNOUT_TIME_MS);
          log_message(std::format(
            "Launch detected! Entering launch phase (recent median altitude: {}m, acceleration: {}m/s^2), burnout in {}ms",
            median_alt, accel_mag, MOTOR_BURNOUT_TIME_MS));
          return StandardFlightPhase::LAUNCH;
        }
      }
      return StandardFlightPhase::PREFLIGHT;
    }
    return StandardFlightPhase::PREFLIGHT;
  }
  else if (current_phase == StandardFlightPhase::LAUNCH)
  {
    if (median_alt > REQ_COAST_PHASE_ALT_M)
    {
      coast_enter_time = get_absolute_time();
      float time_diff_ms = static_cast<float>(absolute_time_diff_us(burnout_time, coast_enter_time)) / 1000;
      log_message(std::format(
        "Coast phase entered from launch phase due to median altitude, altitude ({}m) > {}m, {:.3f}ms left for motor burn",
        median_alt, REQ_COAST_PHASE_ALT_M, time_diff_ms));
      return StandardFlightPhase::COAST;
    }

    if (get_absolute_time() >= burnout_time)
    {
      coast_enter_time = get_absolute_time();
      log_message("Motor burnout, entering coast phase");
      return StandardFlightPhase::COAST;
    }
    return StandardFlightPhase::LAUNCH;
  }
  else if (current_phase == StandardFlightPhase::COAST)
  {
    if (median_alt > max_coast_alt)
    {
      max_coast_alt = median_alt;
    }

    // Once rocket dropped from apogee
    const double drop_from_apogee = max_coast_alt - median_alt;
    if (drop_from_apogee > REQ_APOGEE_DROP_M)
    {
      log_message(std::format(
        "Apogee reached (entering descent phase)! Apogee {}m, detected because of a median altitude drop of {}m (required {}m)",
        max_coast_alt, drop_from_apogee, REQ_APOGEE_DROP_M));
      set_apogee(max_coast_alt);
      return StandardFlightPhase::DESCENT;
    }
    return StandardFlightPhase::COAST;
  }
  else if (current_phase == StandardFlightPhase::DESCENT)
  {
    double min_recent_alt = std::numeric_limits<double>::max();
    double max_recent_alt = std::numeric_limits<double>::lowest();

    for (const TStateData& state : state_history)
    {
      const double curr_altitude = state.altitude;

      if (curr_altitude < min_recent_alt)
      {
        min_recent_alt = curr_altitude;
      }

      if (curr_altitude > max_recent_alt)
      {
        max_recent_alt = curr_altitude;
      }
    }

    // If recently there is a small total deviation
    double recent_deviation = std::abs(max_recent_alt - min_recent_alt);
    if (recent_deviation <= MAX_LAND_ALT_DEVIATION_M)
    {
      log_message(std::format("Landed! Recent deviation = {}m (min: {}m, max: {}m) (maximum deviation {}m) ({} states)",
                              recent_deviation, min_recent_alt, max_recent_alt, MAX_LAND_ALT_DEVIATION_M,
                              state_history.size()));
      return StandardFlightPhase::LANDED;
    }
    log_message(std::format("NOT LANDED! Recent deviation = {}m min: {}m, max: {}m (maximum deviation {}m) ({} states)",
                            recent_deviation, min_recent_alt, max_recent_alt, MAX_LAND_ALT_DEVIATION_M,
                            state_history.size()));
    return StandardFlightPhase::DESCENT;
  }
  else if (current_phase == StandardFlightPhase::LANDED)
  {
    return StandardFlightPhase::LANDED;
  }

  return current_phase;
}

template <typename TStateData>
elijah_state_framework::std_helpers::StandardFlightPhase
elijah_state_framework::std_helpers::StandardFlightPhaseController<TStateData>::predict_phase(
  const StandardFlightPhase last_known_phase,
  const std::deque<TStateData>& states)
{
  return last_known_phase;
}

template <typename TStateData>
std::string elijah_state_framework::std_helpers::StandardFlightPhaseController<TStateData>::get_phase_name(
  const StandardFlightPhase phase) const
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
size_t elijah_state_framework::std_helpers::StandardFlightPhaseController<TStateData>::get_speaker_pattern(
  const StandardFlightPhase phase,
  const uint16_t*& pattern_freqs,
  const uint16_t*& pattern_timing_ms) const
{
  switch (phase)
  {
  case StandardFlightPhase::PREFLIGHT:
    pattern_freqs = preflight_audio_freqs;
    pattern_timing_ms = preflight_audio_timing_ms;
    return preflight_audio_len;
  case StandardFlightPhase::LAUNCH:
    pattern_freqs = launch_audio_freqs;
    pattern_timing_ms = launch_audio_timing_ms;
    return launch_audio_len;
  case StandardFlightPhase::COAST:
    pattern_freqs = coast_audio_freqs;
    pattern_timing_ms = coast_audio_timing_ms;
    return coast_audio_len;
  case StandardFlightPhase::DESCENT:
    pattern_freqs = descent_audio_freqs;
    pattern_timing_ms = descent_audio_timing_ms;
    return descent_audio_len;
  case StandardFlightPhase::LANDED:
    pattern_freqs = landed_audio_freqs;
    pattern_timing_ms = landed_audio_timing_ms;
    return landed_audio_len;
  }
  pattern_freqs = bad_audio_freqs;
  pattern_timing_ms = bad_audio_timing_ms;
  return bad_audio_len;
}

template <typename TStateData>
double elijah_state_framework::std_helpers::StandardFlightPhaseController<TStateData>::get_apogee() const
{
  return max_coast_alt;
}

template <typename TStateData>
void elijah_state_framework::std_helpers::StandardFlightPhaseController<TStateData>::reset_data()
{
  burnout_time = nil_time;
  coast_enter_time = nil_time;
  max_coast_alt = std::numeric_limits<double>::lowest();
}
