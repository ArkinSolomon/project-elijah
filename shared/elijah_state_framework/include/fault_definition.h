#pragma once

#include <cstring>
#include <utility>

#pragma once

namespace elijah_state_framework::internal
{
  template <EnumType FaultKeyType>
  class FaultDefinition
  {
  public:
    using fault_key_t = std::variant<FaultKeyType, CommunicationChannel>;

    FaultDefinition(fault_key_t fault_key, std::string fault_name, uint8_t fault_bit,
                    CommunicationChannel communication_channel, bool is_communication_channel);

    [[nodiscard]] fault_key_t get_fault_key() const;
    [[nodiscard]] const std::string& get_fault_name() const;
    [[nodiscard]] uint8_t get_fault_bit() const;
    [[nodiscard]] CommunicationChannel get_communication_channel() const;
    [[nodiscard]] bool is_communication_channel() const;

    void set_last_fault_message(const std::string& message);
    [[nodiscard]] const std::string& get_last_fault_message() const;

    [[nodiscard]] size_t get_encoded_definition_size() const;
    void encode_definition(uint8_t* dest) const;

  private:
    fault_key_t fault_key;
    uint8_t fault_bit;
    CommunicationChannel communication_channel;
    bool is_com_channel;
    std::string last_fault_message;

    std::string fault_name;
  };
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
elijah_state_framework::internal::FaultDefinition<FaultKeyType>::FaultDefinition(fault_key_t fault_key, std::string fault_name, const uint8_t fault_bit,
                                               const CommunicationChannel communication_channel,
                                               const bool is_communication_channel) :
  fault_key(fault_key),
  fault_bit(fault_bit),
  communication_channel(communication_channel),
  is_com_channel(is_communication_channel),
  fault_name(std::move(fault_name))
{
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
typename elijah_state_framework::internal::FaultDefinition<FaultKeyType>::fault_key_t elijah_state_framework::internal::FaultDefinition<FaultKeyType>::get_fault_key() const
{
  return fault_key;
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
const std::string& elijah_state_framework::internal::FaultDefinition<FaultKeyType>::get_fault_name() const
{
  return fault_name;
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
uint8_t elijah_state_framework::internal::FaultDefinition<FaultKeyType>::get_fault_bit() const
{
  return fault_bit;
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
CommunicationChannel elijah_state_framework::internal::FaultDefinition<FaultKeyType>::get_communication_channel() const
{
  return communication_channel;
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
bool elijah_state_framework::internal::FaultDefinition<FaultKeyType>::is_communication_channel() const
{
  return is_com_channel;
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
void elijah_state_framework::internal::FaultDefinition<FaultKeyType>::set_last_fault_message(const std::string& message)
{
  last_fault_message = message;
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
const std::string& elijah_state_framework::internal::FaultDefinition<FaultKeyType>::get_last_fault_message() const
{
  return last_fault_message;
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
size_t elijah_state_framework::internal::FaultDefinition<FaultKeyType>::get_encoded_definition_size() const
{
  return 2 * sizeof(uint8_t) /* is_com_channel | bit#, channel */ + fault_name.size() + 1 + last_fault_message.size() + 1;
}

template <elijah_state_framework::internal::EnumType FaultKeyType>
void elijah_state_framework::internal::FaultDefinition<FaultKeyType>::encode_definition(uint8_t* dest) const
{
  dest[0] = fault_bit;

  if (is_com_channel)
  {
    dest[0] |= 0x01 << 7;
  }

  dest[1] = static_cast<uint8_t>(communication_channel);

  memcpy(dest + 2, fault_name.c_str(), fault_name.size() + 1);
  memcpy(dest + 2 + fault_name.size() + 1, last_fault_message.c_str(), last_fault_message.size() + 1);
}
