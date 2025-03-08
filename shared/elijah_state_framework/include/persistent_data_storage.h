#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>
#include <hardware/flash.h>
#include <pico/flash.h>
#include <hardware/regs/addressmap.h>
#include <cmath>

#include "shared_mutex.h"
#include "data_type.h"
#include "enum_type.h"
#include "persistent_data_entry.h"

#define PERSISTENT_DATA_START_SECTOR_NUM 511

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
    shared_mutex_enter_blocking_exclusive(&persistent_storage_smtx); \
    const uint32_t saved_ints = save_and_disable_interrupts(); \
    assert(data_entries.contains(key)); \
    const PersistentDataEntry<PersistentKeyType>* entry = data_entries[key]; \
    if (active_data_loc == flash_data_loc) \
    { \
      active_data_loc = malloc(get_total_byte_size()); \
      memcpy(active_data_loc, flash_data_loc, get_total_byte_size()); \
    } \
    const auto data_start = reinterpret_cast<TYPE_NAME*>(static_cast<uint8_t*>(active_data_loc) + sizeof(tag) + entry->get_offset()); \
    *data_start = value; \
    shared_mutex_exit_exclusive(&persistent_storage_smtx); \
    restore_interrupts_from_disabled(saved_ints); \
  }

#define CREATE_GETTER_FOR_TYPE(TYPE_NAME, HUMAN_NAME) \
  TYPE_NAME get_##HUMAN_NAME(PersistentKeyType key) \
  { \
    assert(data_entries.contains(key)); \
    shared_mutex_enter_blocking_shared(&persistent_storage_smtx); \
    const uint32_t saved_ints = save_and_disable_interrupts(); \
    const PersistentDataEntry<PersistentKeyType>* entry = data_entries[key]; \
    const auto data_start = reinterpret_cast<TYPE_NAME*>(static_cast<uint8_t*>(active_data_loc) + sizeof(tag) + entry->get_offset()); \
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

  CREATE_REGISTRATION_FOR_TYPE(int8_t, DataType::Int8)
  CREATE_REGISTRATION_FOR_TYPE(uint8_t, DataType::Uint8)
  CREATE_REGISTRATION_FOR_TYPE(int16_t, DataType::Int16)
  CREATE_REGISTRATION_FOR_TYPE(uint16_t, DataType::UInt16)
  CREATE_REGISTRATION_FOR_TYPE(int32_t, DataType::Int32)
  CREATE_REGISTRATION_FOR_TYPE(uint32_t, DataType::UInt32)
  CREATE_REGISTRATION_FOR_TYPE(int64_t, DataType::Int64)
  CREATE_REGISTRATION_FOR_TYPE(uint64_t, DataType::UInt64)
  CREATE_REGISTRATION_FOR_TYPE(float, DataType::Float)
  CREATE_REGISTRATION_FOR_TYPE(double, DataType::Double)
  CREATE_REGISTRATION_FOR_TYPE(tm, DataType::Time)

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
  [[nodiscard]] std::unique_ptr<uint8_t[]> encode_all_entries(size_t& encoded_size) const;
  [[nodiscard]] uint32_t get_tag() const;
  [[nodiscard ]] size_t get_entry_count() const;

  void lock_active_data();
  void release_active_data();
  [[nodiscard]] void* get_active_data_loc() const;
  [[nodiscard]] size_t get_total_byte_size() const;

  void commit_data();
  void commit_data(bool use_callback, bool lock_mtx);
  void load_default_data();
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

  active_data_loc = const_cast<void*>(flash_data_loc);
}

template <EnumType PersistentKeyType>
PersistentDataStorage<PersistentKeyType>::~PersistentDataStorage()
{
  if (active_data_loc != nullptr)
  {
    free(active_data_loc);
  }

  for (const PersistentDataEntry<PersistentKeyType>* entry : std::views::values(data_entries))
  {
    delete entry;
  }
}

template <EnumType PersistentKeyType>
std::string PersistentDataStorage<PersistentKeyType>::get_string(PersistentKeyType key)
{
  shared_mutex_enter_blocking_shared(&persistent_storage_smtx);
  const uint32_t saved_ints = save_and_disable_interrupts();

  assert(data_entries.contains(key));
  const PersistentDataEntry<PersistentKeyType>* entry = data_entries[key];

  void* str_data_loc = static_cast<uint8_t*>(active_data_loc) + sizeof(tag) + static_size;

  size_t str_byte_offset = 0;
  for (uint i = 0; i < entry->get_offset(); i++)
  {
    const auto curr_c_str = reinterpret_cast<char*>(static_cast<uint8_t*>(str_data_loc) + str_byte_offset);
    str_byte_offset += strlen(curr_c_str + 1);
  }

  const auto str_start = reinterpret_cast<char*>(static_cast<uint8_t*>(str_data_loc) + str_byte_offset);
  std::string str(str_start);

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
  for (uint i = 0; i < entry->get_offset(); i++)
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

  // this probably could've been a realloc but whatever
  void* new_data_loc = malloc(get_total_byte_size());

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
  active_data_loc = new_data_loc;

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
  auto new_entry = new PersistentDataEntry<PersistentKeyType>(key, std::move(display_name), DataType::String,
                                                              string_registrations.size(),
                                                              default_data, default_value.size());
  data_entries[key] = new_entry;
  string_registrations.push_back(new_entry);
  string_size += default_value.size() + 1;
}

template <EnumType PersistentKeyType>
void PersistentDataStorage<PersistentKeyType>::finish_registration()
{
  shared_mutex_enter_blocking_exclusive(&persistent_storage_smtx);
  done_registering_keys = true;
  tag = static_size * 8409 + data_entries.size() + string_registrations.size() * 48;
  for (PersistentDataEntry<PersistentKeyType>* entry : std::views::values(data_entries))
  {
    for (auto& n : entry->get_display_value())
    {
      tag += (n << 1) + string_registrations.size() * 2;
    }
  }

  const uint32_t saved_tag = *static_cast<const uint32_t*>(flash_data_loc);
  if (tag == saved_tag)
  {
    active_data_loc = const_cast<void*>(flash_data_loc);
    shared_mutex_exit_exclusive(&persistent_storage_smtx);
    return;
  }

  active_data_loc = malloc(get_total_byte_size());
  memcpy(active_data_loc, &tag, sizeof(tag));

  auto static_data_loc = sizeof(tag) + static_cast<uint8_t*>(active_data_loc);
  for (PersistentDataEntry<PersistentKeyType>* data_entry : std::views::values(data_entries))
  {
    if (data_entry->get_data_type() == DataType::String)
    {
      continue;
    }

    memcpy(static_data_loc + data_entry->get_offset(), data_entry->get_default_value_ptr(),
           data_entry->get_default_value_size());
  }

  auto string_data_loc = reinterpret_cast<char*>(static_data_loc + static_size);
  size_t string_offset = 0;
  for (auto& string_entry : string_registrations)
  {
    memcpy(string_data_loc + string_offset, string_entry->get_default_value_ptr(),
           string_entry->get_default_value_size());
    *(string_data_loc + string_offset + string_entry->get_default_value_size()) = '\0';
    string_offset += string_entry->get_default_value_size() + 1;
  }

  commit_data(false, false);
}

template <EnumType PersistentKeyType>
std::unique_ptr<uint8_t[]> PersistentDataStorage<PersistentKeyType>::encode_all_entries(size_t& encoded_size) const
{
  encoded_size = 0;
  for (PersistentDataEntry<PersistentKeyType>* entry : std::views::values(data_entries))
  {
    encoded_size += entry->get_encoded_size();
  }

  std::unique_ptr<uint8_t[]> encoded_data(new uint8_t[encoded_size]);
  size_t running_size = 0;

  for (PersistentDataEntry<PersistentKeyType>* entry : std::views::values(data_entries))
  {
    entry->encode_data_entry(encoded_data.get() + running_size);
    running_size += entry->get_encoded_size();
  }

  return encoded_data;
}

template <EnumType PersistentKeyType>
uint32_t PersistentDataStorage<PersistentKeyType>::get_tag() const
{
  return tag;
}

template <EnumType PersistentKeyType>
size_t PersistentDataStorage<PersistentKeyType>::get_entry_count() const
{
  return data_entries.size();
}

template <EnumType PersistentKeyType>
void PersistentDataStorage<PersistentKeyType>::lock_active_data()
{
  shared_mutex_enter_blocking_shared(&persistent_storage_smtx);
}

template <EnumType PersistentKeyType>
void PersistentDataStorage<PersistentKeyType>::release_active_data()
{
  shared_mutex_exit_shared(&persistent_storage_smtx);
}

template <EnumType PersistentKeyType>
void* PersistentDataStorage<PersistentKeyType>::get_active_data_loc() const
{
  return active_data_loc;
}

template <EnumType PersistentKeyType>
size_t PersistentDataStorage<PersistentKeyType>::get_total_byte_size() const
{
  return sizeof(tag) + static_size + string_size;
}

template <EnumType PersistentKeyType>
void PersistentDataStorage<PersistentKeyType>::commit_data()
{
  commit_data(true, true);
}

template <EnumType PersistentKeyType>
void PersistentDataStorage<PersistentKeyType>::commit_data(const bool use_callback, const bool lock_mtx)
{
  const uint32_t saved_ints = save_and_disable_interrupts();
  if (lock_mtx)
  {
    shared_mutex_enter_blocking_exclusive(&persistent_storage_smtx);
  }

  // TODO error handling?
  flash_safe_execute([](void* storage_ptr)
  {
    auto storage = static_cast<PersistentDataStorage*>(storage_ptr);

    constexpr uint32_t flash_offset = PERSISTENT_DATA_START_SECTOR_NUM * 4096;

    const size_t total_size = storage->get_total_byte_size();
    const auto pages = static_cast<size_t>(ceil(total_size / 256.0));

    flash_range_erase(flash_offset, pages * 256);
    flash_range_program(flash_offset, static_cast<uint8_t*>(storage->active_data_loc), total_size);
    free(storage->active_data_loc);
    storage->active_data_loc = const_cast<void*>(storage->flash_data_loc);
  }, this, 250);

  if (commit_callback && use_callback)
  {
    // we CAN NOT read/write to the data here
    commit_callback(flash_data_loc, get_total_byte_size());
  }

  shared_mutex_exit_exclusive(&persistent_storage_smtx);

  restore_interrupts_from_disabled(saved_ints);
}

template <EnumType PersistentKeyType>
void PersistentDataStorage<PersistentKeyType>::load_default_data()
{
  shared_mutex_enter_blocking_exclusive(&persistent_storage_smtx);
  for (PersistentDataEntry<PersistentKeyType>* entry : std::views::values(data_entries))
  {
    switch (entry->get_data_type())
    {
    case DataType::String:
      {
        char def_c_str[entry->get_default_value_size() + 1];
        memcpy(def_c_str, entry->get_default_value_ptr(), entry->get_default_value_size());
        def_c_str[entry->get_default_value_size()] = '\0';

        std::string def_str(def_c_str);

        set_string(entry->get_key(), def_str);
        break;
      }
    case DataType::Int8:
      set_int8(entry->get_key(), *reinterpret_cast<int8_t*>(entry->get_default_value_ptr()));
      break;
    case DataType::Uint8:
      set_uint8(entry->get_key(), *reinterpret_cast<uint8_t*>(entry->get_default_value_ptr()));
      break;
    case DataType::Int16:
      set_int16(entry->get_key(), *reinterpret_cast<int16_t*>(entry->get_default_value_ptr()));
      break;
    case DataType::UInt16:
      set_uint16(entry->get_key(), *reinterpret_cast<uint16_t*>(entry->get_default_value_ptr()));
      break;
    case DataType::Int32:
      set_int32(entry->get_key(), *reinterpret_cast<int32_t*>(entry->get_default_value_ptr()));
      break;
    case DataType::UInt32:
      set_uint32(entry->get_key(), *reinterpret_cast<uint32_t*>(entry->get_default_value_ptr()));
      break;
    case DataType::Int64:
      set_int64(entry->get_key(), *reinterpret_cast<int64_t*>(entry->get_default_value_ptr()));
      break;
    case DataType::UInt64:
      set_uint64(entry->get_key(), *reinterpret_cast<uint64_t*>(entry->get_default_value_ptr()));
      break;
    case DataType::Float:
      set_float(entry->get_key(), *reinterpret_cast<float*>(entry->get_default_value_ptr()));
      break;
    case DataType::Double:
      set_double(entry->get_key(), *reinterpret_cast<double*>(entry->get_default_value_ptr()));
      break;
    case DataType::Time:
      // todo: implement time
      assert(false);
      break;
    }
  }

  // Mutex unlocked after commit
  commit_data(true, false);
}

template <EnumType PersistentKeyType>
void PersistentDataStorage<PersistentKeyType>::on_commit(commit_callback_t&& commit_callback)
{
  this->commit_callback = commit_callback;
}
