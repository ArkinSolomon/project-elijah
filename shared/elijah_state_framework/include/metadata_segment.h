#pragma once

#include <cstdint>

namespace elijah_state_framework::internal
{
  enum class MetadataSegment : uint8_t
  {
    ApplicationName = 1,
    Commands = 2,
    VariableDefinitions = 3,
    PersistentStorageEntries = 4,
    FaultInformation = 5,
    InitialPhase = 6,
    MetadataEnd = 255
  };
}
