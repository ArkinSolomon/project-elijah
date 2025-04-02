#pragma once

#include "standard_flight_phase_controller.h"

#define MAX_COAST_ALT_REG 0x08

struct PayloadState;

class PayloadFlightPhaseController final : public StandardFlightPhaseController<PayloadState>
{
public:
  [[nodiscard]] StandardFlightPhase update_phase(StandardFlightPhase current_phase,
                                                 const std::deque<PayloadState>& state_history) override;
  [[nodiscard]] StandardFlightPhase predict_phase(StandardFlightPhase last_known_phase,
                                                  const std::deque<PayloadState>& state_history) override;
protected:
  void extract_state_data(PayloadState state, double& accel_x, double& accel_y, double& accel_z,
                          double& altitude) const override;

  [[nodiscard]] bool is_calibrated() const override;
};
