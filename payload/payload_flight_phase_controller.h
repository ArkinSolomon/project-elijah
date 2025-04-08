#pragma once

#include "standard_flight_phase_controller.h"

#define MAX_COAST_ALT_REG 0x08

struct PayloadState;

class PayloadFlightPhaseController final : public elijah_state_framework::std_helpers::StandardFlightPhaseController<
    PayloadState>
{
  using StandardFlightPhase = elijah_state_framework::std_helpers::StandardFlightPhase;

public:
  [[nodiscard]] StandardFlightPhase update_phase(StandardFlightPhase current_phase,
                                                 const std::deque<PayloadState>& state_history) override;
  [[nodiscard]] StandardFlightPhase predict_phase(StandardFlightPhase last_known_phase,
                                                  const std::deque<PayloadState>& state_history) override;

protected:
  [[nodiscard]] bool is_calibrated() const override;
  void log_message(const std::string& msg) const override;
};
