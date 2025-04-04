#pragma once

#include "airbrakes_flight_phase_controller.h"
#include "reliable_mpu_6050.h"

#define MPU_6050_ADDR 0x68

class AirbrakesStateManager;
struct AirbrakesState;
enum class AirbrakesFaultKey : uint8_t;
enum class AirbrakesPersistentStateKey : uint8_t;

class AirbrakesReliableMPU6050 final : public ReliableMPU6050<AirbrakesState, AirbrakesPersistentStateKey, AirbrakesFaultKey, StandardFlightPhase, AirbrakesFlightPhaseController>
{
public:
  explicit AirbrakesReliableMPU6050(AirbrakesStateManager* override_state_manager);

protected:
  void update_state(AirbrakesState& state, double xa, double ya, double za, double xg, double yg, double zg) const override;
};
