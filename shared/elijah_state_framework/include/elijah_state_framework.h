#pragma once

#include <cstring>
#include <functional>
#include <ranges>
#include <map>
#include <string>
#include <variant>
#include <pico/critical_section.h>
#include <pico/stdio_usb.h>

#include "data_type.h"
#include "persistent_data_storage.h"
#include "registered_command.h"
#include "variable_definition.h"

#define STDIO_WRITE_SIZE 512
constexpr uint64_t FRAMEWORK_TAG = 0xBC7AA65201C73901;

#define START_STATE_ENCODER(STATE_DATA_TYPE) void encode_state(void* encode_dest, const STATE_DATA_TYPE& state, bool register_data) override \
  { \
  size_t data_len = 0; \
  size_t curr_data_size_len; \
  if (register_data) { \
    assert(!is_size_calculated); \
  }

#define ENCODE_STATE(MEMBER_NAME, DATA_TYPE, DISP_NAME, DISP_UNIT) \
  assert(!done_with_static); \
  static_assert(!std::is_same<decltype(state.MEMBER_NAME), std::string>::value, "Property (" #MEMBER_NAME ") must be not be of type string"); \
  curr_data_size_len = data_type_helpers::get_size_for_data_type(DATA_TYPE); \
  if (register_data) { \
    register_data_variable(DISP_NAME, DISP_UNIT, data_len, DATA_TYPE); \
  } else { \
    memcpy(static_cast<uint8_t*>(encode_dest) + data_len, &state.MEMBER_NAME, curr_data_size_len); \
  } \
  data_len += curr_data_size_len;
#define END_STATE_ENCODER() \
    if (register_data) { \
      set_encoded_state_size(data_len); \
    } \
  }

enum class LogLevel : uint8_t
{
  Debug = 0,
  Default = 1,
  Warning = 2,
  Error = 3
};

template <typename StateDataType, EnumType PersistentKeyType>
class ElijahStateFramework
{
public:
  explicit ElijahStateFramework(std::string application_name);
  virtual ~ElijahStateFramework() = default;

  PersistentDataStorage<PersistentKeyType>& get_persistent_data_storage() const;

  static void log_message(const std::string& message, LogLevel log_level = LogLevel::Default);

  [[nodiscard]] const std::string& get_application_name() const;

  void check_for_commands();
  void state_changed(const StateDataType& new_state);

  void send_framework_metadata();

protected:
  void register_command(const std::string& command, std::function<void()> callback);
  void register_command(const std::string& command, std::function<void(double)> callback);
  void register_command(const std::string& command, bool is_alphanumeric,
                        std::function<void(std::string)> callback);
  void register_command(const std::string& command, std::function<void(tm)> callback);

  void register_data_variable(const std::string& display_name, const std::string& display_unit, size_t offset,
                              DataType data_type);

  void set_encoded_state_size(size_t encoded_data_size);
  void finish_registration();
  void encode_state(void* encode_dest, const StateDataType& state);
  virtual void encode_state(void* encode_dest, const StateDataType& encode_data,
                           bool register_data) = 0;

private:
  template <class... Ts>
  struct overloaded : Ts...
  {
    using Ts::operator()...;
  };

  template <class... Ts>
  overloaded(Ts...) -> overloaded<Ts...>;

  enum class OutputPacketId : uint8_t
  {
    Metadata = 1,
    LogMessage = 2,
    CollectionData = 3,
    PersistentState = 4
  };

  enum class MetadataSegmentId : uint8_t
  {
    ApplicationName = 1,
    Commands = 2,
    VariableDefinitions = 3
  };

  static inline critical_section_t usb_cs;

  std::string application_name;

  uint8_t command_id_counter = 0;
  uint8_t variable_id_counter = 0;

  std::map<uint8_t, RegisteredCommand> registered_commands;
  std::map<uint8_t, VariableDefinition> variable_definitions;

  bool is_size_calculated = false;
  size_t encoded_collection_data_size = 0;

  PersistentDataStorage<PersistentKeyType>* persistent_data_storage = new PersistentDataStorage<PersistentKeyType>();

  static void initialize_communication();
  static void write_to_serial(const uint8_t* packet_data, size_t packet_len);

  void register_command(const std::string& command, CommandInputType command_input, command_callback_t callback);
};

template <typename StateDataType, EnumType PersistentKeyType>
ElijahStateFramework<StateDataType, PersistentKeyType>::ElijahStateFramework(std::string application_name) :
  application_name(std::move(application_name))
{
  initialize_communication();

  persistent_data_storage->on_commit([this](const void* data, const size_t data_len)
  {
    critical_section_enter_blocking(&usb_cs);
    const auto packet_id = static_cast<uint8_t>(OutputPacketId::PersistentState);

    write_to_serial(&packet_id, 1);
    write_to_serial(static_cast<const uint8_t*>(data), data_len);

    critical_section_exit(&usb_cs);
  });
}

template <typename StateDataType, EnumType PersistentKeyType>
PersistentDataStorage<PersistentKeyType>& ElijahStateFramework<StateDataType, PersistentKeyType>::
get_persistent_data_storage() const
{
  return *persistent_data_storage;
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::check_for_commands()
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

  switch (command->get_input_type())
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
             }, command->get_callback());
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::state_changed(const StateDataType& new_state)
{
  const size_t total_packet_size = encoded_collection_data_size + 1;
  const auto packet_data = new uint8_t[total_packet_size];
  packet_data[0] = static_cast<uint8_t>(OutputPacketId::CollectionData);
  encode_state(packet_data + 1, new_state);

  if (stdio_usb_connected())
  {
    critical_section_enter_blocking(&usb_cs);
    write_to_serial(packet_data, total_packet_size);
    critical_section_exit(&usb_cs);
  }

  delete [] packet_data;
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::send_framework_metadata()
{
  if (!stdio_usb_connected())
  {
    return;
  }

  critical_section_enter_blocking(&usb_cs);

  write_to_serial(reinterpret_cast<const uint8_t*>(&FRAMEWORK_TAG), sizeof(FRAMEWORK_TAG));
  //
  // auto segment_id = static_cast<uint8_t>(MetadataSegmentId::ApplicationName);
  // write_to_serial(&segment_id, 1);
  // write_to_serial(reinterpret_cast<const uint8_t*>(application_name.c_str()), application_name.size());
  //
  // const uint8_t command_count = registered_commands.size();
  // if (command_count > 0)
  // {
  //   segment_id = static_cast<uint8_t>(MetadataSegmentId::Commands);
  //   const uint8_t segment_header[2] = {segment_id, command_count};
  //   write_to_serial(segment_header, 2);
  //
  //   for (const auto& command : std::views::values(registered_commands))
  //   {
  //     size_t encoded_size;
  //     std::unique_ptr<uint8_t> encoded = command.encode_command(encoded_size);
  //     write_to_serial(encoded.get(), encoded_size);
  //   }
  // }
  //
  // const uint8_t var_count = variable_definitions.size();
  // if (var_count > 0)
  // {
  //   segment_id = static_cast<uint8_t>(MetadataSegmentId::VariableDefinitions);
  //   const uint8_t segment_header[2] = {segment_id, var_count};
  //   write_to_serial(segment_header, 2);
  //
  //   for (const auto& var_def : std::views::values(variable_definitions))
  //   {
  //     size_t encoded_size;
  //     std::unique_ptr<uint8_t> encoded = var_def.encode_var(encoded_size);
  //     write_to_serial(encoded.get(), encoded_size);
  //   }
  // }

  critical_section_exit(&usb_cs);
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::encode_state(void* encode_dest,
                                                                              const StateDataType& state)
{
  encode_state(encode_dest, state, false);
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::log_message(const std::string& message,
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
    static_cast<uint8_t>(OutputPacketId::LogMessage), static_cast<uint8_t>(log_level)
  };

  *reinterpret_cast<uint16_t*>(packet_data + 2) = static_cast<uint16_t>(send_message.size());
  memcpy(packet_data + 4, send_message.c_str(), send_message.size());

  critical_section_enter_blocking(&usb_cs);
  write_to_serial(packet_data, total_packet_size);
  critical_section_exit(&usb_cs);
}

template <typename StateDataType, EnumType PersistentKeyType>
const std::string& ElijahStateFramework<StateDataType, PersistentKeyType>::get_application_name() const
{
  return application_name;
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::register_command(const std::string& command,
  std::function<void()> callback)
{
  register_command(command, CommandInputType::NONE, callback);
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::register_command(const std::string& command,
  std::function<void(double)> callback)
{
  register_command(command, CommandInputType::DOUBLE, callback);
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::register_command(const std::string& command,
  const bool is_alphanumeric, std::function<void(std::string)> callback)
{
  register_command(command, is_alphanumeric ? CommandInputType::ALPHANUMERIC : CommandInputType::STRING, callback);
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::register_command(const std::string& command,
  std::function<void(tm)> callback)
{
  register_command(command, CommandInputType::TIME, callback);
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::register_data_variable(
  const std::string& display_name, const std::string& display_unit, const size_t offset, const DataType data_type)
{
  const uint8_t variable_id = variable_id_counter;
  variable_id_counter++;

  variable_definitions[variable_id] = VariableDefinition(variable_id, display_name,
                                                         display_unit, offset, data_type);
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::set_encoded_state_size(
  const size_t encoded_data_size)
{
  is_size_calculated = true;
  encoded_collection_data_size = encoded_data_size;
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::finish_registration()
{
  StateDataType collection_data;
  encode_state(nullptr, collection_data, true);
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::initialize_communication()
{
  stdio_init_all();
  critical_section_init(&usb_cs);
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::write_to_serial(const uint8_t* packet_data,
  const size_t packet_len)
{
  for (size_t buff_idx = 0; buff_idx < packet_len; buff_idx += STDIO_WRITE_SIZE)
  {
    const size_t write_size = std::min(static_cast<size_t>(STDIO_WRITE_SIZE), packet_len - buff_idx);
    stdio_put_string(reinterpret_cast<const char*>(packet_data) + buff_idx, static_cast<int>(write_size), false, false);
    stdio_flush();
  }
}

template <typename StateDataType, EnumType PersistentKeyType>
void ElijahStateFramework<StateDataType, PersistentKeyType>::register_command(const std::string& command,
  const CommandInputType command_input, command_callback_t callback)
{
  const uint8_t new_id = command_id_counter++;
  registered_commands[new_id] = RegisteredCommand(new_id, command, command_input, std::move(callback));
}
