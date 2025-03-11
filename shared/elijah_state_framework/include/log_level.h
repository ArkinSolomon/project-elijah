#pragma once
#include <cstdint>

namespace elijah_state_framework
{
  enum class LogLevel : uint8_t
  {
    Debug = 1,
    Default = 2,
    Warning = 3,
    Error = 4,
    SerialOnly = 5
  };
}
