#pragma once
#include <cstdint>

enum class CommunicationChannel : uint8_t
{
  SPI_0 = 0,
  SPI_1 = 1,
  I2C_0 = 2,
  I2C_1 = 3,
  None = 33
};
