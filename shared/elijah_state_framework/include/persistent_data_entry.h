#pragma once

#include <memory>
#include <string>
#include <utility>

#include "enum_type.h"
#include "data_type.h"

template <EnumType PersistentKeyType>
class PersistentDataEntry
{
public:
  PersistentDataEntry();
  PersistentDataEntry(PersistentKeyType key, std::string display_value, DataType data_type, size_t offset,
                      void* default_value, size_t default_value_size);

  ~PersistentDataEntry();

  [[nodiscard]] PersistentKeyType get_key() const;
  [[nodiscard]] const std::string& get_display_value() const;
  [[nodiscard]] DataType get_data_type() const;
  [[nodiscard]] size_t get_offset() const;

  [[nodiscard]] void* get_default_value_ptr() const;
  [[nodiscard]] size_t get_default_value_size() const;
  void delete_default_value();

  [[nodiscard]] std::unique_ptr<uint8_t> encode_data_entry(size_t& encoded_size) const;

private:
  PersistentKeyType key;
  std::string display_value;
  DataType data_type;

  // offset is the string# for a string entry, or byte offset for regular
  size_t offset;

  void* default_value;
  size_t default_value_size;
};

template <EnumType PersistentKeyType>
PersistentDataEntry<PersistentKeyType>::PersistentDataEntry(): data_type(), offset(0), default_value(nullptr),
                                                               default_value_size(0)
{
}

template <EnumType PersistentKeyType>
PersistentDataEntry<PersistentKeyType>::PersistentDataEntry(PersistentKeyType key, std::string display_value,
                                                            const DataType data_type, const size_t offset,
                                                            void* default_value,
                                                            const size_t default_value_size) : key(key),
  display_value(std::move(display_value)),
  data_type(data_type), offset(offset),
  default_value(default_value),
  default_value_size(default_value_size)
{
}

template <EnumType PersistentKeyType>
PersistentDataEntry<PersistentKeyType>::~PersistentDataEntry()
{
  if (default_value != nullptr)
  {
    free(default_value);
  }
}

template <EnumType PersistentKeyType>
PersistentKeyType PersistentDataEntry<PersistentKeyType>::get_key() const
{
  return key;
}

template <EnumType PersistentKeyType>
const std::string& PersistentDataEntry<PersistentKeyType>::get_display_value() const
{
  return display_value;
}

template <EnumType PersistentKeyType>
DataType PersistentDataEntry<PersistentKeyType>::get_data_type() const
{
  return data_type;
}

template <EnumType PersistentKeyType>
size_t PersistentDataEntry<PersistentKeyType>::get_offset() const
{
  return offset;
}

template <EnumType PersistentKeyType>
void* PersistentDataEntry<PersistentKeyType>::get_default_value_ptr() const
{
  return default_value;
}

template <EnumType PersistentKeyType>
size_t PersistentDataEntry<PersistentKeyType>::get_default_value_size() const
{
  return default_value_size;
}

template <EnumType PersistentKeyType>
void PersistentDataEntry<PersistentKeyType>::delete_default_value()
{
  if (default_value == nullptr)
  {
    return;
  }

  free(default_value);
  default_value = nullptr;
  default_value_size = 0;
}

template <EnumType PersistentKeyType>
std::unique_ptr<uint8_t> PersistentDataEntry<PersistentKeyType>::encode_data_entry(size_t& encoded_size) const
{
  encoded_size = sizeof(uint8_t) /* data_type */ + sizeof(offset) + display_value.size() + 1;
  std::unique_ptr<uint8_t> encoded_entry(new uint8_t[encoded_size]);

  encoded_entry.get()[0] = static_cast<uint8_t>(data_type);
  *reinterpret_cast<decltype(offset)*>(encoded_entry.get()[sizeof(uint8_t)]) = offset;

  memcpy(encoded_entry.get() + sizeof(uint8_t) + sizeof(offset), display_value.c_str(), display_value.size() + 1);

  return encoded_entry;
}
