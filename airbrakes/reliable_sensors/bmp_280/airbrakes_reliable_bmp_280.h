#pragma once

#include "airbrakes_flight_phase_controller.h"
#include "reliable_bmp_280.h"

class AirbrakesStateManager;
enum class AirbrakesFaultKey : uint8_t;
enum class AirbrakesPersistentStateKey : uint8_t;

class AirbrakesReliableBMP280 final : public ReliableBMP280<AirbrakesState, AirbrakesPersistentStateKey, AirbrakesFaultKey, StandardFlightPhase, AirbrakesFlightPhaseController> {
public:
  explicit AirbrakesReliableBMP280(AirbrakesStateManager* override_state_manager);

protected:
  void update_state(AirbrakesState& state, int32_t pressure, double temperature, double altitude) const override;
};

