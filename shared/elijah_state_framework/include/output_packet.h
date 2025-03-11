#pragma once
#include <cstdint>

namespace elijah_state_framework::internal
{
  enum class OutputPacket : uint8_t
  {
    LogMessage = 1,
    StateUpdate = 2,
    PersistentStateUpdate = 3,
    Metadata = 4,
    DeviceRestartMarker = 5,
    FaultsChanged = 6,
    PhaseChanged = 7
  };
}
