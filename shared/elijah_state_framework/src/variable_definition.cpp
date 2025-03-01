#include "variable_definition.h"

#include <cstring>

VariableDefinition::VariableDefinition() :
  variable_id(0),
  data_offset(0),
  data_type(DataType::INT8)
{
}

VariableDefinition::VariableDefinition(const uint8_t variable_id,
                                       std::string display_name,
                                       std::string display_unit, const size_t data_offset,
                                       const DataType data_type): variable_id(variable_id),
                                                                  display_name(std::move(display_name)),
                                                                  display_unit(std::move(display_unit)),
                                                                  data_offset(data_offset), data_type(data_type)
{
}

uint8_t VariableDefinition::get_variable_id() const
{
  return variable_id;
}

const std::string& VariableDefinition::get_display_name() const
{
  return display_name;
}

const std::string& VariableDefinition::get_display_unit() const
{
  return display_unit;
}

size_t VariableDefinition::get_offset() const
{
  return data_offset;
}

DataType VariableDefinition::get_data_type() const
{
  return data_type;
}

std::unique_ptr<uint8_t> VariableDefinition::encode_var(size_t& encoded_size) const
{
  encoded_size = sizeof(variable_id) + sizeof(data_offset) + sizeof(data_type)
    + display_name.size() + 1 + display_unit.size() + 1;
  std::unique_ptr<uint8_t> encoded_command(new uint8_t[encoded_size]);

  encoded_command.get()[0] = variable_id;
  *reinterpret_cast<decltype(data_offset)*>(encoded_command.get()[sizeof(variable_id)]) = data_offset;
  encoded_command.get()[sizeof(variable_id) + sizeof(data_offset)] = static_cast<uint8_t>(data_type);

  constexpr size_t str_off = sizeof(variable_id) + sizeof(data_offset) + sizeof(uint8_t);
  memcpy(encoded_command.get() + str_off, display_name.c_str(), display_name.size() + 1);
  memcpy(encoded_command.get() + str_off + display_name.size() + 1, display_unit.c_str(), display_unit.size() + 1);

  return encoded_command;
}
