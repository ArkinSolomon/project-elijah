#pragma once

#include "override_flight_phase_controller.h"
#include "reliable_bmp_280.h"

class OverrideStateManager;
enum class FaultKey : uint8_t;
enum class OverridePersistentStateKey : uint8_t;

class OverrideReliableBMP280 final : public ReliableBMP280<OverrideState, OverridePersistentStateKey, FaultKey, StandardFlightPhase, OverrideFlightPhaseController> {
public:
  explicit OverrideReliableBMP280(OverrideStateManager* override_state_manager);

protected:
  void update_state(OverrideState& state, int32_t pressure, double temperature, double altitude) const override;
};

