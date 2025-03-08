#pragma once

#include <cstring>
#include <utility>

#pragma once

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

  [[nodiscard]] size_t get_encoded_definition_size() const;
  void encode_definition(uint8_t* dest) const;

private:
  fault_key_t fault_key;
  uint8_t fault_bit;
  CommunicationChannel communication_channel;
  bool is_com_channel;

  std::string fault_name;
};

template <EnumType FaultKeyType>
FaultDefinition<FaultKeyType>::FaultDefinition(fault_key_t fault_key, std::string fault_name, const uint8_t fault_bit,
                                               const CommunicationChannel communication_channel,
                                               const bool is_communication_channel) :
  fault_key(fault_key),
  fault_bit(fault_bit),
  communication_channel(communication_channel),
  is_com_channel(is_communication_channel),
  fault_name(std::move(fault_name))
{
}

template <EnumType FaultKeyType>
typename FaultDefinition<FaultKeyType>::fault_key_t FaultDefinition<FaultKeyType>::get_fault_key() const
{
  return fault_key;
}

template <EnumType FaultKeyType>
const std::string& FaultDefinition<FaultKeyType>::get_fault_name() const
{
  return fault_name;
}

template <EnumType FaultKeyType>
uint8_t FaultDefinition<FaultKeyType>::get_fault_bit() const
{
  return fault_bit;
}

template <EnumType FaultKeyType>
CommunicationChannel FaultDefinition<FaultKeyType>::get_communication_channel() const
{
  return communication_channel;
}

template <EnumType FaultKeyType>
bool FaultDefinition<FaultKeyType>::is_communication_channel() const
{
  return is_com_channel;
}

template <EnumType FaultKeyType>
size_t FaultDefinition<FaultKeyType>::get_encoded_definition_size() const
{
  return 2 * sizeof(uint8_t) /* is_com_channel | bit#, channel */ + fault_name.size() + 1;
}

template <EnumType FaultKeyType>
void FaultDefinition<FaultKeyType>::encode_definition(uint8_t* dest) const
{
  dest[0] = fault_bit;

  if (is_com_channel)
  {
    dest[0] |= 0x01 << 7;
  }

  dest[1] = static_cast<uint8_t>(communication_channel);

  memcpy(dest + 2, fault_name.c_str(), fault_name.size() + 1);
}
