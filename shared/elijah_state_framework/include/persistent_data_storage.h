#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>
#include <hardware/flash.h>
#include <hardware/regs/addressmap.h>
#include <cmath>

#include "shared_mutex.h"
#include "data_type.h"
#include "enum_type.h"
#include "persistent_data_entry.h"

#define PERSISTENT_DATA_START_SECTOR_NUM 500

#define CREATE_REGISTRATION_FOR_TYPE(TYPE_NAME, DATA_TYPE) \
  void register_key(PersistentKeyType key, const std::string& display_name, const TYPE_NAME default_value) \
  { \
    assert(!done_registering_keys); \
    assert(!data_entries.contains(key)); \
    void* default_data = malloc(sizeof(TYPE_NAME)); \
    memcpy(default_data, &default_value, sizeof(TYPE_NAME)); \
    data_entries[key] = new PersistentDataEntry<PersistentKeyType>(key, display_name, DATA_TYPE, static_size, default_data, sizeof(TYPE_NAME)); \
    static_size += sizeof(TYPE_NAME); \
  }

#define CREATE_SETTER_FOR_TYPE(TYPE_NAME, HUMAN_NAME) \
  void set_##HUMAN_NAME(PersistentKeyType key, const TYPE_NAME value) \
  { \
    const uint32_t saved_ints = save_and_disable_interrupts(); \
    shared_mutex_enter_blocking_exclusive(&persistent_storage_smtx); \
    assert(data_entries.contains(key)); \
    const PersistentDataEntry<PersistentKeyType>* entry = data_entries[key]; \
    if (active_data_loc == flash_data_loc) \
    { \
      active_data_loc = malloc(get_total_size()); \
    } \
    const auto data_start = reinterpret_cast<TYPE_NAME*>(static_cast<uint8_t*>(active_data_loc) + entry->offset); \
    *data_start = value; \
    shared_mutex_exit_exclusive(&persistent_storage_smtx); \
    restore_interrupts_from_disabled(saved_ints); \
  }

#define CREATE_GETTER_FOR_TYPE(TYPE_NAME, HUMAN_NAME) \
  TYPE_NAME get_##HUMAN_NAME(PersistentKeyType key) \
  { \
    const uint32_t saved_ints = save_and_disable_interrupts(); \
    shared_mutex_enter_blocking_shared(&persistent_storage_smtx); \
    assert(data_entries.contains(key)); \
    const PersistentDataEntry<PersistentKeyType>* entry = data_entries[key]; \
    const auto data_start = reinterpret_cast<TYPE_NAME*>(static_cast<uint8_t*>(active_data_loc) + entry->offset); \
    const TYPE_NAME value = *data_start; \
    shared_mutex_exit_shared(&persistent_storage_smtx); \
    restore_interrupts_from_disabled(saved_ints); \
    return value; \
  }

using commit_callback_t = std::function<void(const void*, size_t)>;

template <EnumType PersistentKeyType>
class PersistentDataStorage
{
public:
  PersistentDataStorage();
  ~PersistentDataStorage();

  void register_key(PersistentKeyType key, const std::string& display_name, const std::string& default_value);
  std::string get_string(PersistentKeyType key);
  void set_string(PersistentKeyType key, const std::string& value);

  CREATE_REGISTRATION_FOR_TYPE(int8_t, DataType::INT8)
  CREATE_REGISTRATION_FOR_TYPE(uint8_t, DataType::UINT8)
  CREATE_REGISTRATION_FOR_TYPE(int16_t, DataType::INT16)
  CREATE_REGISTRATION_FOR_TYPE(uint16_t, DataType::UINT16)
  CREATE_REGISTRATION_FOR_TYPE(int32_t, DataType::INT32)
  CREATE_REGISTRATION_FOR_TYPE(uint32_t, DataType::UINT32)
  CREATE_REGISTRATION_FOR_TYPE(int64_t, DataType::INT64)
  CREATE_REGISTRATION_FOR_TYPE(uint64_t, DataType::UINT64)
  CREATE_REGISTRATION_FOR_TYPE(float, DataType::FLOAT)
  CREATE_REGISTRATION_FOR_TYPE(double, DataType::DOUBLE)
  CREATE_REGISTRATION_FOR_TYPE(tm, DataType::TIME)

  CREATE_SETTER_FOR_TYPE(int8_t, int8)
  CREATE_SETTER_FOR_TYPE(uint8_t, uint8)
  CREATE_SETTER_FOR_TYPE(int16_t, int16)
  CREATE_SETTER_FOR_TYPE(uint16_t, uint16)
  CREATE_SETTER_FOR_TYPE(int32_t, int32)
  CREATE_SETTER_FOR_TYPE(uint32_t, uint32)
  CREATE_SETTER_FOR_TYPE(int64_t, int64)
  CREATE_SETTER_FOR_TYPE(uint64_t, uint64)
  CREATE_SETTER_FOR_TYPE(float, float)
  CREATE_SETTER_FOR_TYPE(double, double)
  CREATE_SETTER_FOR_TYPE(tm, time)

  CREATE_GETTER_FOR_TYPE(int8_t, int8)
  CREATE_GETTER_FOR_TYPE(uint8_t, uint8)
  CREATE_GETTER_FOR_TYPE(int16_t, int16)
  CREATE_GETTER_FOR_TYPE(uint16_t, uint16)
  CREATE_GETTER_FOR_TYPE(int32_t, int32)
  CREATE_GETTER_FOR_TYPE(uint32_t, uint32)
  CREATE_GETTER_FOR_TYPE(int64_t, int64)
  CREATE_GETTER_FOR_TYPE(uint64_t, uint64)
  CREATE_GETTER_FOR_TYPE(float, float)
  CREATE_GETTER_FOR_TYPE(double, double)
  CREATE_GETTER_FOR_TYPE(tm, time)

  void finish_registration();

  [[nodiscard]] size_t get_total_size() const;

  void commit_data();
  void on_commit(commit_callback_t&& commit_callback);

private:
  shared_mutex_t persistent_storage_smtx;

  // For fixed length vars (next offset is this size)
  size_t static_size = 0;

  // Saved length of strings (including null chars)
  size_t string_size = 0;

  bool done_registering_keys = false;
  uint32_t tag = 0;

  void* active_data_loc = nullptr;
  const void* flash_data_loc = reinterpret_cast<void*>(XIP_BASE + PERSISTENT_DATA_START_SECTOR_NUM * 4096);

  std::vector<PersistentDataEntry<PersistentKeyType>*> string_registrations;
  std::map<PersistentKeyType, PersistentDataEntry<PersistentKeyType>*> data_entries;

  commit_callback_t commit_callback;
};

template <EnumType PersistentKeyType>
PersistentDataStorage<PersistentKeyType>::PersistentDataStorage()
{
  shared_mutex_init(&persistent_storage_smtx);
}

template <EnumType PersistentKeyType>
PersistentDataStorage<PersistentKeyType>::~PersistentDataStorage()
{
  if (active_data_loc != nullptr)
  {
    free(active_data_loc);
  }
  for (auto& entry : data_entries)
  {
    delete entry.second;
  }
}

template <EnumType PersistentKeyType>
std::string PersistentDataStorage<PersistentKeyType>::get_string(PersistentKeyType key)
{
  const uint32_t saved_ints = save_and_disable_interrupts();
  shared_mutex_enter_blocking_shared(&persistent_storage_smtx);

  assert(data_entries.contains(key));
  const PersistentDataEntry<PersistentKeyType>* entry = data_entries[key];

  void* str_data_loc = static_cast<uint8_t*>(active_data_loc) + sizeof(tag) + static_size;

  size_t str_byte_offset = 0;
  for (uint i = 0; i < entry->offset; i++)
  {
    const auto curr_c_str = reinterpret_cast<char*>(static_cast<uint8_t*>(str_data_loc) + str_byte_offset);
    str_byte_offset += strlen(curr_c_str + 1);
  }

  const auto str_start = reinterpret_cast<char*>(static_cast<uint8_t*>(str_data_loc) + str_byte_offset);
  const std::string str(str_start);

  shared_mutex_exit_shared(&persistent_storage_smtx);
  restore_interrupts_from_disabled(saved_ints);

  return str;
}

template <EnumType PersistentKeyType>
void PersistentDataStorage<PersistentKeyType>::set_string(PersistentKeyType key, const std::string& value)
{
  const uint32_t saved_ints = save_and_disable_interrupts();
  shared_mutex_enter_blocking_exclusive(&persistent_storage_smtx);

  assert(data_entries.contains(key));
  const PersistentDataEntry<PersistentKeyType>* entry = data_entries[key];

  const void* str_data_loc = static_cast<uint8_t*>(active_data_loc) + sizeof(tag) + static_size;

  size_t str_byte_offset = 0;
  for (uint i = 0; i < entry->offset; i++)
  {
    const auto curr_c_str = reinterpret_cast<const char*>(static_cast<const uint8_t*>(str_data_loc) + str_byte_offset);
    str_byte_offset += strlen(curr_c_str + 1);
  }

  const auto str_start = reinterpret_cast<const char*>(static_cast<const uint8_t*>(str_data_loc) + str_byte_offset);

  const size_t old_str_bytes = strlen(str_start) + 1;
  const size_t prev_data_size = sizeof(tag) + static_size + str_byte_offset;
  const size_t post_data_size = string_size - str_byte_offset - old_str_bytes;

  string_size -= old_str_bytes;
  string_size += value.length() + 1;

  void* new_data_loc = malloc(get_total_size());

  if (prev_data_size > 0)
  {
    memcpy(new_data_loc, active_data_loc, prev_data_size);
  }

  char* new_str_loc = static_cast<char*>(new_data_loc) + prev_data_size;
  memcpy(new_str_loc, value.c_str(), value.length());
  char* new_str_end_off = new_str_loc + value.length();
  *new_str_end_off = '\0';

  if (post_data_size > 0)
  {
    memcpy(reinterpret_cast<uint8_t*>(new_str_end_off) + 1,
           static_cast<uint8_t*>(active_data_loc) + prev_data_size + old_str_bytes, post_data_size);
  }

  if (active_data_loc != flash_data_loc)
  {
    free(active_data_loc);
  }

  shared_mutex_exit_exclusive(&persistent_storage_smtx);
  restore_interrupts_from_disabled(saved_ints);
}

template <EnumType PersistentKeyType>
void PersistentDataStorage<PersistentKeyType>::register_key(PersistentKeyType key, const std::string& display_name,
                                                            const std::string& default_value)
{
  assert(!done_registering_keys);

  void* default_data = malloc(default_value.size());
  memcpy(default_data, default_value.c_str(), default_value.size());
  data_entries[key] = new PersistentDataEntry(key, display_name, default_value, string_registrations.size(),
                                              default_data,
                                              default_value.size());
  string_registrations.push_back(key);
  string_size += default_value.size() + 1;
}

template <EnumType PersistentKeyType>
void PersistentDataStorage<PersistentKeyType>::finish_registration()
{
  done_registering_keys = true;
  tag = static_size * 1000 + data_entries.size() + string_registrations.size() * 1000;

  const uint32_t saved_tag = *static_cast<const uint32_t*>(flash_data_loc);
  if (tag == saved_tag)
  {
    active_data_loc = const_cast<void*>(flash_data_loc);

    for (auto& map_entry : data_entries)
    {
      auto data_entry = map_entry.second;
      data_entry->delete_default_value();
    }

    string_registrations.clear();
    return;
  }

  active_data_loc = malloc(get_total_size());
  memcpy(active_data_loc, &tag, sizeof(tag));

  auto static_data_loc = static_cast<uint8_t*>(active_data_loc) + sizeof(tag);

  for (auto& map_entry : data_entries)
  {
    auto data_entry = map_entry.second;
    if (data_entry->get_data_type() == DataType::STRING)
    {
      continue;
    }

    memcpy(static_data_loc + data_entry->get_offset(), data_entry->get_default_value_ptr(),
           data_entry->get_default_value_size());
    data_entry->delete_default_value();
  }

  auto string_data_loc = reinterpret_cast<char*>(static_data_loc + static_size);
  size_t string_offset = 0;
  for (auto& string_entry : string_registrations)
  {
    memcpy(string_data_loc + string_offset, string_entry->get_default_value_ptr(),
           string_entry->get_default_value_size());
    *(string_data_loc + string_offset + string_entry->get_default_value_size()) = '\0';
    string_offset += string_entry->get_default_value_size() + 1;

    string_entry->delete_default_value();
  }

  string_registrations.clear();
}

template <EnumType PersistentKeyType>
size_t PersistentDataStorage<PersistentKeyType>::get_total_size() const
{
  return sizeof(tag) + static_size + string_size;
}

template <EnumType PersistentKeyType>
void PersistentDataStorage<PersistentKeyType>::commit_data()
{
  const uint32_t saved_ints = save_and_disable_interrupts();
  shared_mutex_enter_blocking_exclusive(&persistent_storage_smtx);

  // Just assume it worked tbh
  flash_safe_execute([](void* storage_ptr)
  {
    auto storage = static_cast<PersistentDataStorage*>(storage_ptr);

    constexpr uint32_t flash_offset = PERSISTENT_DATA_START_SECTOR_NUM * 4096;

    const size_t total_size = storage->get_total_size();
    const auto pages = static_cast<size_t>(ceil(total_size / 256.0));

    flash_range_erase(flash_offset, pages * 256);
    flash_range_program(flash_offset, storage->active_data_loc, total_size);
    free(storage->active_data_loc);
    storage->active_data_loc = storage->flash_data_loc;
  }, this, 250);

  shared_mutex_exit_exclusive(&persistent_storage_smtx);

  if (commit_callback)
  {
    shared_mutex_enter_blocking_shared(&persistent_storage_smtx);
    commit_callback(flash_data_loc, get_total_size());
    shared_mutex_exit_shared(&persistent_storage_smtx);
  }

  restore_interrupts_from_disabled(saved_ints);
}

template <EnumType PersistentKeyType>
void PersistentDataStorage<PersistentKeyType>::on_commit(commit_callback_t&& commit_callback)
{
  this->commit_callback = commit_callback;
}
