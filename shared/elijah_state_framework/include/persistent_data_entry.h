#pragma once

#include <memory>
#include <string>
#include <utility>

#include "enum_type.h"
#include "data_type.h"

namespace elijah_state_framework::internal
{
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

    [[nodiscard]] size_t get_encoded_size() const;
    void encode_data_entry(uint8_t* dest) const;

  private:
    PersistentKeyType key;
    std::string display_value;
    DataType data_type;

    // offset is the string# for a string entry, or byte offset for regular
    size_t offset;

    void* default_value;
    size_t default_value_size;
  };
}

template <elijah_state_framework::internal::EnumType PersistentKeyType>
elijah_state_framework::internal::PersistentDataEntry<PersistentKeyType>::PersistentDataEntry(): data_type(), offset(0),
  default_value(nullptr),
  default_value_size(0)
{
}

template <elijah_state_framework::internal::EnumType PersistentKeyType>
elijah_state_framework::internal::PersistentDataEntry<PersistentKeyType>::PersistentDataEntry(
  PersistentKeyType key, std::string display_value,
  const DataType data_type, const size_t offset,
  void* default_value,
  const size_t default_value_size) : key(key),
                                     display_value(std::move(display_value)),
                                     data_type(data_type), offset(offset),
                                     default_value(default_value),
                                     default_value_size(default_value_size)
{
}

template <elijah_state_framework::internal::EnumType PersistentKeyType>
elijah_state_framework::internal::PersistentDataEntry<PersistentKeyType>::~PersistentDataEntry()
{
  if (default_value != nullptr)
  {
    free(default_value);
  }
}

template <elijah_state_framework::internal::EnumType PersistentKeyType>
PersistentKeyType elijah_state_framework::internal::PersistentDataEntry<PersistentKeyType>::get_key() const
{
  return key;
}

template <elijah_state_framework::internal::EnumType PersistentKeyType>
const std::string& elijah_state_framework::internal::PersistentDataEntry<PersistentKeyType>::get_display_value() const
{
  return display_value;
}

template <elijah_state_framework::internal::EnumType PersistentKeyType>
DataType elijah_state_framework::internal::PersistentDataEntry<PersistentKeyType>::get_data_type() const
{
  return data_type;
}

template <elijah_state_framework::internal::EnumType PersistentKeyType>
size_t elijah_state_framework::internal::PersistentDataEntry<PersistentKeyType>::get_offset() const
{
  return offset;
}

template <elijah_state_framework::internal::EnumType PersistentKeyType>
void* elijah_state_framework::internal::PersistentDataEntry<PersistentKeyType>::get_default_value_ptr() const
{
  return default_value;
}

template <elijah_state_framework::internal::EnumType PersistentKeyType>
size_t elijah_state_framework::internal::PersistentDataEntry<PersistentKeyType>::get_default_value_size() const
{
  return default_value_size;
}

template <elijah_state_framework::internal::EnumType PersistentKeyType>
size_t elijah_state_framework::internal::PersistentDataEntry<PersistentKeyType>::get_encoded_size() const
{
  return 2 * sizeof(uint8_t) /* key, data_type */ + sizeof(offset) + display_value.size() + 1;
}

template <elijah_state_framework::internal::EnumType PersistentKeyType>
void elijah_state_framework::internal::PersistentDataEntry<PersistentKeyType>::encode_data_entry(uint8_t* dest) const
{
  dest[0] = static_cast<uint8_t>(key);
  dest[sizeof(uint8_t)] = static_cast<uint8_t>(data_type);

  memcpy(dest + 2 * sizeof(uint8_t), &offset, sizeof(offset));
  memcpy(dest + 2 * sizeof(uint8_t) + sizeof(offset), display_value.c_str(), display_value.size() + 1);
}
