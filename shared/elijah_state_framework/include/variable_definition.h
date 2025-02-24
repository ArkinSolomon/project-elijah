#pragma once
#include <cstdint>
#include <string>

#include "data_type.h"

enum class VariableDefCategory : uint8_t
{
  UNKNOWN = 0,
  DATA = 1,
  PERSISTENT = 2,
  TIMING = 3
};

struct VariableDefinition
{
  VariableDefCategory var_cat;
  uint8_t variable_id;
  std::string display_name;
  std::string display_unit;
  size_t data_offset;
  DataType data_type;

  VariableDefinition();
  VariableDefinition(VariableDefCategory var_cat, uint8_t variable_id, std::string display_name,
                     std::string display_unit, size_t data_offset, DataType data_type);
};
