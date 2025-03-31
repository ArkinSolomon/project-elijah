#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <ranges>
#include <malloc.h>

#include "communication_channel.h"
#include "enum_type.h"
#include "fault_definition.h"
#include "shared_mutex.h"

template <elijah_state_framework::internal::EnumType FaultKeyType>
class FaultManager
{
public:
  explicit FaultManager(uint32_t default_faults);

  void register_fault(FaultKeyType key, std::string fault_name, CommunicationChannel communication_channel);

  [[nodiscard]] uint8_t get_fault_count() const;
  [[nodiscard]] std::unique_ptr<uint8_t[]> encode_all_faults(size_t& encoded_size) const;

  [[ nodiscard ]] uint32_t get_all_faults();
  uint32_t set_fault_status(FaultKeyType key, bool is_faulted, uint8_t& fault_bit, bool& did_fault_change);
  [[nodiscard]] bool is_faulted(FaultKeyType key);
  [[nodiscard]] bool is_faulted(CommunicationChannel communication_channel);

private:
  // the shared mutex really only locks the fault uint32
  shared_mutex_t faults_smtx;

  // Leave room for communication channel bits
  uint8_t next_fault_bit = 4;
  uint32_t faults;

  std::map<uint8_t, elijah_state_framework::internal::FaultDefinition<FaultKeyType>> fault_map;

  void register_fault(CommunicationChannel communication_channel, std::string fault_name);
  elijah_state_framework::internal::FaultDefinition<FaultKeyType>* find_fault(FaultKeyType fault_key);
};

template <elijah_state_framework::internal::EnumType FaultKeyType>
FaultManager<FaultKeyType>::FaultManager(const uint32_t default_faults) : faults(default_faults)
{
  shared_mutex_init(&faults_smtx);

  register_fault(CommunicationChannel::SPI_0, "SPI 0");
  register_fault(CommunicationChannel::SPI_1, "SPI 1");
  register_fault(CommunicationChannel::I2C_0, "I2C 0");
  register_fault(CommunicationChannel::I2C_1, "I2C 1");
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
void FaultManager<FaultKeyType>::register_fault(FaultKeyType key, std::string fault_name,
                                                CommunicationChannel communication_channel)
{
  fault_map.emplace(
    next_fault_bit,
    elijah_state_framework::internal::FaultDefinition<FaultKeyType>(key, fault_name, next_fault_bit,
                                                                    communication_channel, false)
  );
  next_fault_bit++;
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
uint8_t FaultManager<FaultKeyType>::get_fault_count() const
{
  return fault_map.size();
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
std::unique_ptr<uint8_t[]> FaultManager<FaultKeyType>::encode_all_faults(size_t& encoded_size) const
{
  encoded_size = 0;
  for (const elijah_state_framework::internal::FaultDefinition<FaultKeyType>& entry : std::views::values(fault_map))
  {
    encoded_size += entry.get_encoded_definition_size();
  }

  std::unique_ptr<uint8_t[]> encoded_data(new uint8_t[encoded_size]);
  size_t running_size = 0;

  for (const elijah_state_framework::internal::FaultDefinition<FaultKeyType>& entry : std::views::values(fault_map))
  {
    entry.encode_definition(encoded_data.get() + running_size);
    running_size += entry.get_encoded_definition_size();
  }

  return encoded_data;
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
uint32_t FaultManager<FaultKeyType>::get_all_faults()
{
  shared_mutex_enter_blocking_shared(&faults_smtx);
  const uint32_t saved_ints = save_and_disable_interrupts();
  const uint32_t curr_faults = faults;
  shared_mutex_exit_shared(&faults_smtx);
  restore_interrupts(saved_ints);
  return curr_faults;
}

template <elijah_state_framework::internal::EnumType FaultKeyEnumType>
uint32_t FaultManager<FaultKeyEnumType>::set_fault_status(FaultKeyEnumType key, const bool is_faulted,
                                                          uint8_t& fault_bit, bool& did_fault_change)
{
  shared_mutex_enter_blocking_exclusive(&faults_smtx);
  const uint32_t saved_ints = save_and_disable_interrupts();

  elijah_state_framework::internal::FaultDefinition<FaultKeyEnumType>* changing_fault_def = find_fault(key);

  assert(changing_fault_def != nullptr);

  fault_bit = changing_fault_def->get_fault_bit();
  const bool is_currently_faulted = (faults & 0x01 << fault_bit) > 0;
  if (is_currently_faulted == is_faulted)
  {
    shared_mutex_exit_exclusive(&faults_smtx);
    restore_interrupts_from_disabled(saved_ints);

    did_fault_change = false;
    return faults;
  }
  did_fault_change = true;

  if (is_faulted)
  {
    faults |= 0x01 << fault_bit;
  }
  else
  {
    faults &= ~(0x01 << fault_bit);
  }

  if (changing_fault_def->get_communication_channel() != CommunicationChannel::None)
  {
    bool one_device_not_faulted = false;
    for (const elijah_state_framework::internal::FaultDefinition<FaultKeyEnumType>& other_com_fault_def :
         std::views::values(fault_map))
    {
      if (!other_com_fault_def.is_communication_channel() &&
        other_com_fault_def.get_communication_channel() == changing_fault_def->get_communication_channel())
      {
        if ((faults & (0x01 << other_com_fault_def.get_fault_bit())) == 0)
        {
          one_device_not_faulted = true;
          break;
        }
      }
    }

    elijah_state_framework::internal::FaultDefinition<FaultKeyEnumType>& com_entry = fault_map.at(
      static_cast<uint8_t>(changing_fault_def->get_communication_channel()));
    if (one_device_not_faulted)
    {
      faults &= ~(0x01 << com_entry.get_fault_bit());
    }
    else
    {
      faults |= 0x01 << com_entry.get_fault_bit();
    }
  }

  shared_mutex_exit_exclusive(&faults_smtx);
  restore_interrupts_from_disabled(saved_ints);
  return faults;
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
bool FaultManager<FaultKeyType>::is_faulted(FaultKeyType key)
{
  shared_mutex_enter_blocking_shared(&faults_smtx);

  elijah_state_framework::internal::FaultDefinition<FaultKeyType>* def = find_fault(key);
  assert(def != nullptr);

  const bool is_faulted = (faults & 0x01 << def->get_fault_bit()) > 0;;

  shared_mutex_exit_shared(&faults_smtx);
  return is_faulted;
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
bool FaultManager<FaultKeyType>::is_faulted(CommunicationChannel communication_channel)
{
  shared_mutex_enter_blocking_shared(&faults_smtx);

  const bool is_faulted = (faults & 0x01 << static_cast<uint8_t>(communication_channel)) > 0;

  shared_mutex_exit_shared(&faults_smtx);
  return is_faulted;
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
void FaultManager<FaultKeyType>::register_fault(CommunicationChannel communication_channel, std::string fault_name)
{
  fault_map.emplace(static_cast<uint8_t>(communication_channel),
                    elijah_state_framework::internal::FaultDefinition<FaultKeyType>(
                      communication_channel, fault_name, static_cast<uint8_t>(communication_channel),
                      communication_channel, true)
  );
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
elijah_state_framework::internal::FaultDefinition<FaultKeyType>* FaultManager<FaultKeyType>::find_fault(
  FaultKeyType fault_key)
{
  for (elijah_state_framework::internal::FaultDefinition<FaultKeyType>& fault_def : std::views::values(fault_map))
  {
    typename elijah_state_framework::internal::FaultDefinition<FaultKeyType>::fault_key_t check_fault_key = fault_def.
      get_fault_key();
    if (std::holds_alternative<FaultKeyType>(check_fault_key) && std::get<FaultKeyType>(check_fault_key) == fault_key)
    {
      return &fault_def;
    }
  }

  return nullptr;
}
