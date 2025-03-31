#pragma once

#include "payload_flight_phase_controller.h"
#include "reliable_component_helper.h"
#include "sensors/ds_1307/ds_1307.h"

#define DS_1307_FUNCTIONAL_CHECK_CYCLES 100

enum class PayloadPersistentDataKey : uint8_t;
struct PayloadState;
class PayloadStateManager;
enum class PayloadFaultKey : uint8_t;

class ReliableClock : public elijah_state_framework::ReliableComponentHelper<PayloadState, PayloadPersistentDataKey, PayloadFaultKey, StandardFlightPhase, PayloadFlightPhaseController>
{
  public:
    ReliableClock();

    [[nodiscard]] DS1307& get_ds_1307();
    void set_clock(const tm& time_inst);

  protected:
    std::string on_init(PayloadState& state) override;
    std::string on_update(PayloadState& state) override;

  private:
    DS1307 ds_1307;

    uint cycles_since_last_check = 0;
};