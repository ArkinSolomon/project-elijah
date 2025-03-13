#pragma once

#include "payload_flight_phase_controller.h"
#include "standard_flight_phase_controller.h"
#include "reliable_mpu_6050.h"

enum class PayloadPersistentDataKey : uint8_t;
struct PayloadState;
class PayloadStateManager;
enum class FaultKey : uint8_t;

class PayloadReliableMPU6050 final : public ReliableMPU6050<
    PayloadState, PayloadPersistentDataKey, FaultKey, StandardFlightPhase, PayloadFlightPhaseController>
{
public:
  explicit PayloadReliableMPU6050(PayloadStateManager* payload_state_manager);

protected:
  void update_state(PayloadState& state, double xa, double ya, double za, double xg, double yg,
                    double zg) const override;
};
