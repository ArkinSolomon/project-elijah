#include "variable_definition.h"

VariableDefinition::VariableDefinition() :
  var_cat(VariableDefCategory::UNKNOWN),
  variable_id(0),
  data_offset(0),
  data_type(DataType::INT8)
{
}

VariableDefinition::VariableDefinition(const VariableDefCategory var_cat, const uint8_t variable_id,
                                       std::string display_name,
                                       std::string display_unit, const size_t data_offset,
                                       const DataType data_type): var_cat(var_cat),
                                                                  variable_id(variable_id),
                                                                  display_name(std::move(display_name)),
                                                                  display_unit(std::move(display_unit)),
                                                                  data_offset(data_offset), data_type(data_type)
{
}
