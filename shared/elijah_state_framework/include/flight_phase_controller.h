#pragma once
#include "enum_type.h"

namespace elijah_state_framework
{
  template <typename TStateData, internal::EnumType EFlightPhase>
  class FlightPhaseController
  {
  public:
    virtual ~FlightPhaseController() = default;
    virtual EFlightPhase initial_flight_phase() const = 0;
    virtual bool should_log(EFlightPhase current_phase) const = 0;

    virtual EFlightPhase update_phase(EFlightPhase current_phase, const std::deque<TStateData>& state_history) = 0;
    virtual EFlightPhase predict_phase(EFlightPhase last_known_phase, const std::deque<TStateData>& state_history) const = 0;

    virtual std::string get_phase_name(EFlightPhase phase) const = 0;
  };
}
