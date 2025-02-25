#pragma once

#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <variant>
#include <pico/critical_section.h>
#include <pico/stdio_usb.h>

#include "data_type.h"
#include "persistent_data_storage.h"
#include "registered_command.h"
#include "variable_definition.h"

#define MAX_STDIO_WRITE_BUFF 512

#define START_DATA_ENCODER(COLLECTION_DATA_TYPE) void encode_data(void* encode_dest, const COLLECTION_DATA_TYPE& encode_data, bool register_data) override \
  { \
  size_t data_len = 0; \
  size_t curr_data_size_len; \
  assert(!is_size_calculated);

#define ENCODE_DATA(MEMBER_NAME, DATA_TYPE, DISP_NAME, DISP_UNIT) \
  assert(!done_with_static); \
  static_assert(!std::is_same<decltype(encode_data.MEMBER_NAME), std::string>::value, "Property (" #MEMBER_NAME ") must be not be of type string"); \
  curr_data_size_len = data_type_helpers::get_size_for_data_type(DATA_TYPE); \
  if (register_data) { \
    register_data_variable(DISP_NAME, DISP_UNIT, data_len, DATA_TYPE); \
  } else { \
    memcpy(static_cast<uint8_t*>(encode_dest) + data_len, &encode_data.MEMBER_NAME, curr_data_size_len); \
  } \
  data_len += curr_data_size_len; \

#define END_DATA_ENCODER() \
    if (register_data) { \
      set_encoded_collection_data_size(data_len); \
    } \
  }

enum class LogLevel : uint8_t
{
  DEBUGGING = 0,
  DEFAULT = 1,
  WARN = 2,
  ERROR = 3
};

enum class OutputPacketId : uint8_t
{
  LOG_MESSAGE = 1,
  COLUMN_DEFS = 2
};

template <typename CollectionDataType, EnumType PersistentKeyType>
class ElijahStateFramework
{
public:
  ElijahStateFramework() = default;
  virtual ~ElijahStateFramework() = default;

  static void initialize_communication();

  PersistentDataStorage<PersistentKeyType>& get_persistent_data_storage() const;

  static void log_message(const std::string& message, LogLevel log_level = LogLevel::DEFAULT);

  void check_for_commands();
  void data_collected(const CollectionDataType& new_data);

protected:
  void register_command(const std::string& command, std::function<void()> callback);
  void register_command(const std::string& command, std::function<void(double)> callback);
  void register_command(const std::string& command, bool is_alphanumeric,
                        std::function<void(std::string)> callback);
  void register_command(const std::string& command, std::function<void(tm)> callback);

  void register_data_variable(const std::string& display_name, const std::string& display_unit, size_t offset,
                              DataType data_type);

  void set_encoded_collection_data_size(size_t encoded_data_size);
  void calculate_data_size();
  void encode_data(void* encode_dest, const CollectionDataType& encode_data);
  virtual void encode_data(void* encode_dest, const CollectionDataType& encode_data,
                           bool register_data) = 0;

private:
  template <class... Ts>
  struct overloaded : Ts...
  {
    using Ts::operator()...;
  };

  template <class... Ts>
  overloaded(Ts...) -> overloaded<Ts...>;

  static inline critical_section_t usb_cs;

  uint8_t command_id_counter = 0;
  uint8_t variable_id_counter = 0;

  std::map<uint8_t, RegisteredCommand> registered_commands;
  std::map<uint8_t, VariableDefinition> variable_definitions;

  bool is_size_calculated = false;
  size_t encoded_collection_data_size = 0;

  PersistentDataStorage<PersistentKeyType>* persistent_data_storage = new PersistentDataStorage<PersistentKeyType>();

  static void write_packet(const uint8_t* packet_data, size_t packet_len);
  void register_command(const std::string& command, CommandInputType command_input, command_callback_t callback);
};

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::initialize_communication()
{
  stdio_init_all();
  critical_section_init(&usb_cs);
}

template <typename CollectionDataType, EnumType PersistentKeyType>
PersistentDataStorage<PersistentKeyType>& ElijahStateFramework<CollectionDataType, PersistentKeyType>::
get_persistent_data_storage() const
{
  return *persistent_data_storage;
}

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::check_for_commands()
{
  if (!stdio_usb_connected())
  {
    return;
  }

  critical_section_enter_blocking(&usb_cs);

  uint8_t command_id = 0xFF;
  int bytes_read = stdio_get_until(reinterpret_cast<char*>(&command_id), 1, delayed_by_ms(get_absolute_time(), 10));
  if (bytes_read != 1)
  {
    critical_section_exit(&usb_cs);
    return;
  }

  if (!registered_commands.contains(command_id))
  {
    critical_section_exit(&usb_cs);
    return;
  }
  const RegisteredCommand* command = &registered_commands[command_id];

  double double_arg;
  std::string str_arg;
  tm time_arg{};

  switch (command->command_input)
  {
  case CommandInputType::DOUBLE:
    {
      double value;
      bytes_read = stdio_get_until(reinterpret_cast<char*>(&value), sizeof(value),
                                   delayed_by_ms(get_absolute_time(), 10));
      critical_section_exit(&usb_cs);
      if (bytes_read != sizeof(value))
      {
        return;
      }
      break;
    }
  case CommandInputType::ALPHANUMERIC:
  case CommandInputType::STRING:
    {
      uint8_t str_size_buf[2];
      bytes_read = stdio_get_until(reinterpret_cast<char*>(str_size_buf), 2, delayed_by_ms(get_absolute_time(), 10));
      if (bytes_read != 2)
      {
        critical_section_exit(&usb_cs);
        return;
      }

      const uint16_t str_size = *reinterpret_cast<uint16_t*>(str_size_buf);
      char str_buff[str_size + 1];
      bytes_read = stdio_get_until(str_buff, str_size, delayed_by_ms(get_absolute_time(), 10));
      critical_section_exit(&usb_cs);

      if (bytes_read != str_size)
      {
        return;
      }
      str_buff[str_size] = '\0';
      str_arg = std::string(str_buff);

      break;
    }
  case CommandInputType::TIME:
    critical_section_exit(&usb_cs);
  // TODO time input
    break;
  case CommandInputType::NONE:
  default:
    critical_section_exit(&usb_cs);
    break;
  }

  std::visit(overloaded{
               [](const std::function<void()>& cb) { cb(); },
               [&double_arg](const std::function<void(double)>& cb) { cb(double_arg); },
               [&str_arg](const std::function<void(std::string)>& cb) { cb(str_arg); },
               [&time_arg](const std::function<void(tm)>& cb) { cb(time_arg); },
             }, command->callback);
}

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::data_collected(const CollectionDataType& new_data)
{

}

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::encode_data(void* encode_dest,
  const CollectionDataType& encode_data)
{
  encode_data(encode_dest, encode_data, false);
}

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::log_message(const std::string& message,
  LogLevel log_level)
{
  if (!stdio_usb_connected())
  {
    return;
  }

  std::string send_message = message;

  if (message.size() > std::numeric_limits<uint16_t>::max())
  {
    send_message = "Message too large... ";
    send_message += message.substr(0, std::numeric_limits<uint16_t>::max() - send_message.size());
  }

  const size_t total_packet_size = 1 /* packet id */ + 1 /* log level */ + 2 /* message length */ + send_message.
    size();
  uint8_t packet_data[total_packet_size] = {
    static_cast<uint8_t>(OutputPacketId::LOG_MESSAGE), static_cast<uint8_t>(log_level)
  };

  *reinterpret_cast<uint16_t*>(packet_data + 2) = static_cast<uint16_t>(send_message.size());
  memcpy(packet_data + 4, send_message.c_str(), send_message.size());

  write_packet(packet_data, total_packet_size);
}

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::register_command(const std::string& command,
  std::function<void()> callback)
{
  register_command(command, CommandInputType::NONE, callback);
}

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::register_command(const std::string& command,
  std::function<void(double)> callback)
{
  register_command(command, CommandInputType::DOUBLE, callback);
}

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::register_command(const std::string& command,
  const bool is_alphanumeric, std::function<void(std::string)> callback)
{
  register_command(command, is_alphanumeric ? CommandInputType::ALPHANUMERIC : CommandInputType::STRING, callback);
}

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::register_command(const std::string& command,
  std::function<void(tm)> callback)
{
  register_command(command, CommandInputType::TIME, callback);
}

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::register_data_variable(
  const std::string& display_name, const std::string& display_unit, const size_t offset, const DataType data_type)
{
  const uint8_t variable_id = variable_id_counter;
  variable_id_counter++;

  variable_definitions[variable_id] = VariableDefinition(VariableDefCategory::DATA, variable_id, display_name, display_unit, offset, data_type);
}

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::set_encoded_collection_data_size(
  const size_t encoded_data_size)
{
  is_size_calculated = true;
  encoded_collection_data_size = encoded_data_size;
}

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::calculate_data_size()
{
  CollectionDataType collection_data;
  encode_data(nullptr, collection_data, true);
}

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::write_packet(const uint8_t* packet_data,
  const size_t packet_len)
{
  if (!stdio_usb_connected())
  {
    return;
  }

  critical_section_enter_blocking(&usb_cs);
  size_t remaining_len = packet_len;
  size_t offset = 0;
  while (remaining_len > 0)
  {
    size_t write_size = 0;
    if (remaining_len >= MAX_STDIO_WRITE_BUFF)
    {
      remaining_len -= MAX_STDIO_WRITE_BUFF;
      write_size = MAX_STDIO_WRITE_BUFF;
    }
    else
    {
      write_size = remaining_len;
      remaining_len = 0;
    }

    stdio_put_string(reinterpret_cast<const char*>(packet_data + offset), static_cast<int>(write_size), false, false);
    stdio_flush();
    offset += write_size;
  }

  critical_section_exit(&usb_cs);
}

template <typename CollectionDataType, EnumType PersistentKeyType>
void ElijahStateFramework<CollectionDataType, PersistentKeyType>::register_command(const std::string& command,
  const CommandInputType command_input, command_callback_t callback)
{
  const uint8_t new_id = command_id_counter++;
  registered_commands[new_id] = RegisteredCommand(new_id, command, command_input, std::move(callback));
}
