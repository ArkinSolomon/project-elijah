#pragma once

#include <string>

#include "enum_type.h"
#include "data_type.h"

template <EnumType PersistentKeyType>
struct PersistentDataEntry
{
  PersistentKeyType key;
  const std::string& display_value;
  DataType data_type;

  // offset is the string# for a string entry, or byte offset for regular
  size_t offset;

  void* default_value;
  size_t default_value_size;

  PersistentDataEntry();
  PersistentDataEntry(PersistentKeyType key, const std::string& display_value, DataType data_type, size_t offset,
                      void* default_value, size_t default_value_size);

  ~PersistentDataEntry();
};

template <EnumType PersistentKeyType>
PersistentDataEntry<PersistentKeyType>::PersistentDataEntry():
  display_value(), data_type(), offset(0), default_value(nullptr), default_value_size(0)
{
}

template <EnumType PersistentKeyType>
PersistentDataEntry<PersistentKeyType>::PersistentDataEntry(PersistentKeyType key, const std::string& display_value,
                                                            const DataType data_type, const size_t offset,
                                                            void* default_value,
                                                            const size_t default_value_size) : key(key),
  display_value(display_value),
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