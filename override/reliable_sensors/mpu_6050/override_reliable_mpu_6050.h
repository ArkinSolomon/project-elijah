#pragma once

#include "override_flight_phase_controller.h"
#include "reliable_mpu_6050.h"

class OverrideStateManager;
struct OverrideState;
enum class OverrideFaultKey : uint8_t;
enum class OverridePersistentStateKey : uint8_t;

class OverrideReliableMPU6050 final : public ReliableMPU6050<OverrideState, OverridePersistentStateKey, OverrideFaultKey, StandardFlightPhase, OverrideFlightPhaseController>
{
public:
  explicit OverrideReliableMPU6050(OverrideStateManager* override_state_manager);

protected:
  void update_state(OverrideState& state, double xa, double ya, double za, double xg, double yg, double zg) const override;
};
