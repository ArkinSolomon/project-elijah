#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "data_type.h"

class VariableDefinition
{
public:
  VariableDefinition();
  VariableDefinition(uint8_t variable_id, std::string display_name,
                     std::string display_unit, size_t data_offset, DataType data_type);

  [[nodiscard]] uint8_t get_variable_id() const;
  [[nodiscard]] const std::string& get_display_name() const;
  [[nodiscard]] const std::string& get_display_unit() const;
  [[nodiscard]] size_t get_offset() const;
  [[nodiscard]] DataType get_data_type() const;

  [[nodiscard]] std::unique_ptr<uint8_t> encode_var(size_t& encoded_size) const;

private:
  uint8_t variable_id;
  std::string display_name;
  std::string display_unit;
  size_t data_offset;
  DataType data_type;
};
