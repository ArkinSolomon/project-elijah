#pragma once

#include "payload_flight_phase_controller.h"
#include "reliable_bmp_280.h"
#include "standard_flight_phase_controller.h"

class PayloadStateManager;
enum class PayloadPersistentDataKey : uint8_t;
struct PayloadState;
enum class OverrideFaultKey : uint8_t;
enum class OverridePersistentStateKey : uint8_t;

class PayloadReliableBMP280 final : public ReliableBMP280<
    PayloadState, PayloadPersistentDataKey, OverrideFaultKey, StandardFlightPhase, PayloadFlightPhaseController>
{
public:
  explicit PayloadReliableBMP280(PayloadStateManager* payload_state_manager);

protected:
  void update_state(PayloadState& state, int32_t pressure, double temperature, double altitude) const override;
};
