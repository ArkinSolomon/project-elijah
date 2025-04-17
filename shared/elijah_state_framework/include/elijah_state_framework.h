#pragma once

#include <cstring>
#include <deque>
#include <functional>
#include <ranges>
#include <map>
#include <string>
#include <format>
#include <variant>
#include <hardware/watchdog.h>
#include <hardware/gpio.h>
#include <pico/critical_section.h>
#include <pico/rand.h>
#include <pico/stdio_usb.h>

#include "data_type.h"
#include "fault_manager.h"
#include "flight_phase_controller.h"
#include "log_level.h"
#include "metadata_segment.h"
#include "output_packet.h"
#include "persistent_data_storage.h"
#include "registered_command.h"
#include "speaker_controller.h"
#include "usb_comm.h"
#include "state_framework_logger.h"
#include "variable_definition.h"

#define START_STATE_ENCODER(STATE_DATA_TYPE) void encode_state(void* encode_dest, const STATE_DATA_TYPE& state, const uint64_t seq, bool register_data) override \
  { \
  size_t data_len = 0; \
  size_t curr_data_size_len; \
  if (register_data) { \
    assert(!is_state_size_calculated); \
  } \
  curr_data_size_len = data_type_helpers::get_size_for_data_type(DataType::UInt64); \
  if (register_data) { \
    register_data_variable("_sequence", "", data_len, DataType::UInt64); \
  } else { \
    memcpy(static_cast<uint8_t*>(encode_dest) + data_len, &seq, curr_data_size_len); \
  } \
  data_len += curr_data_size_len; \
  if (register_data) { \
    register_data_variable("_us_since_boot", "", data_len, DataType::UInt64); \
  } else { \
    uint64_t us = to_us_since_boot(get_absolute_time()); \
    memcpy(static_cast<uint8_t*>(encode_dest) + data_len, &us, curr_data_size_len); \
  } \
  data_len += curr_data_size_len;
#define ENCODE_STATE(MEMBER_NAME, DATA_TYPE, DISP_NAME, DISP_UNIT) \
  static_assert(DataType::String != (DATA_TYPE) && !std::is_same<decltype(state.MEMBER_NAME), std::string>::value, "Property (" #MEMBER_NAME ") must be not be of type string"); \
  static_assert(DataType::Time != (DATA_TYPE) && !std::is_same<decltype(state.MEMBER_NAME), tm>::value, "Property (" #MEMBER_NAME ") must not be a time type, use ENCODE_TIME_STATE(" #MEMBER_NAME ", " #DISP_NAME ") instead"); \
  curr_data_size_len = data_type_helpers::get_size_for_data_type(DATA_TYPE); \
  if (register_data) { \
    register_data_variable(DISP_NAME, DISP_UNIT, data_len, DATA_TYPE); \
  } else { \
    memcpy(static_cast<uint8_t*>(encode_dest) + data_len, &state.MEMBER_NAME, curr_data_size_len); \
  } \
  data_len += curr_data_size_len;
#define ENCODE_TIME_STATE(MEMBER_NAME, DISP_NAME) \
  static_assert(std::is_same<decltype(state.MEMBER_NAME), tm>::value, "Using ENCODE_TIME_STATE requires encoding a time variable"); \
  curr_data_size_len = data_type_helpers::get_size_for_data_type(DataType::Time); \
  if (register_data) { \
    register_data_variable(DISP_NAME, "_time_unit", data_len, DataType::Time); \
  } else { \
    elijah_state_framework::internal::encode_time(static_cast<uint8_t*>(encode_dest) + data_len, state.MEMBER_NAME); \
  } \
  data_len += curr_data_size_len;
#define END_STATE_ENCODER() \
    if (register_data) { \
      set_encoded_state_size(data_len); \
    } \
  }

#define FRAMEWORK_TEMPLATE_DECL \
template <typename TStateData, \
  elijah_state_framework::internal::EnumType EPersistentStorageKey, \
  elijah_state_framework::internal::EnumType EFaultKey, \
  elijah_state_framework::internal::EnumType EFlightPhase, \
  std::derived_from<elijah_state_framework::FlightPhaseController<TStateData, EFlightPhase>> TFlightPhaseController \
>
#define FRAMEWORK_TEMPLATE_TYPES \
  TStateData, \
  EPersistentStorageKey, \
  EFaultKey, \
  EFlightPhase, \
  TFlightPhaseController

#define MAX_MICRO_SD_METADATA_REMOUNT_ATTEMPTS 10
#define MAX_MICRO_SD_LOG_WRITE_REMOUNT_ATTEMPTS 100

constexpr uint64_t FRAMEWORK_TAG = 0x11335577AAEEFF33;

namespace elijah_state_framework
{
  namespace std_helpers
  {
    class StdCommandRegistrationHelpers;
  }

  FRAMEWORK_TEMPLATE_DECL
  class ElijahStateFramework
  {
    friend class std_helpers::StdCommandRegistrationHelpers;

  public:
    ElijahStateFramework(std::string application_name, size_t state_history_size);
    virtual ~ElijahStateFramework();

    PersistentDataStorage<EPersistentStorageKey>* get_persistent_storage() const;

    [[nodiscard]] const std::string& get_application_name() const;

    void check_for_log_write();

    void log_message(const std::string& message,
                     LogLevel log_level = LogLevel::Default);
    void check_for_commands();

    void state_changed(const TStateData& new_state);
    void lock_state_history();
    void release_state_history();
    [[nodiscard]] const std::deque<TStateData>& get_state_history() const;

    [[nodiscard]] TFlightPhaseController* get_flight_phase_controller() const;
    [[nodiscard]] EFlightPhase get_current_flight_phase();
    void set_flight_phase(EFlightPhase new_phase);

    void set_fault(EFaultKey fault_key, bool fault_state);
    void set_fault(EFaultKey fault_key, bool fault_state, const std::string& message);
    [[nodiscard]] bool is_faulted(EFaultKey fault_key);

  protected:
    using StdCommandRegistrationHelpers = std_helpers::StdCommandRegistrationHelpers;

    bool is_state_size_calculated = false;

    void register_command(const std::string& command, std::function<void()> callback);
    void register_command(const std::string& command, const std::string& input_prompt,
                          std::function<void(double)> callback);
    void register_command(const std::string& command, const std::string& input_prompt, bool is_alphanumeric,
                          std::function<void(std::string)> callback);
    void register_command(const std::string& command, const std::string& input_prompt,
                          std::function<void(tm)> callback);

    void register_data_variable(const std::string& display_name, const std::string& display_unit, size_t offset,
                                DataType data_type);

    void register_fault(EFaultKey key, std::string fault_name, CommunicationChannel communication_channel);

    void set_encoded_state_size(size_t encoded_data_size);
    void finish_construction();

    void encode_state(void* encode_dest, const TStateData& state);
    virtual void encode_state(void* encode_dest, const TStateData& encode_data, uint64_t seq,
                              bool register_data) = 0;

  private:
    std::string application_name;
    static constexpr EPersistentStorageKey launch_key = EPersistentStorageKey::LaunchKey;
    static constexpr EPersistentStorageKey flight_phase_key = EPersistentStorageKey::FlightPhaseKey;

    uint8_t command_id_counter = 1;
    uint8_t variable_id_counter = 1;
    uint64_t state_seq = 1;

    std::map<uint8_t, RegisteredCommand> registered_commands;
    std::map<uint8_t, VariableDefinition> variable_definitions;

    size_t encoded_state_size = 0;

    shared_mutex_t state_history_smtx;
    size_t state_history_size;
    std::deque<TStateData> state_history;

    PersistentDataStorage<EPersistentStorageKey>* persistent_data_storage = new PersistentDataStorage<
      EPersistentStorageKey>();

    StateFrameworkLogger* logger = nullptr;
    shared_mutex_t logger_smtx;
    static constexpr EFaultKey micro_sd_fault_key = EFaultKey::MicroSD;
    bool did_write_metadata = false, require_writing_restart_marker = false;
    absolute_time_t last_mount_attempt_time = nil_time;
    uint8_t remount_attempts = 0;

    FaultManager<EFaultKey>* fault_manager = new FaultManager<EFaultKey>(0x0000);

    TFlightPhaseController* flight_phase_controller;
    EFlightPhase current_phase;
    mutex_t current_phase_mtx;

    constexpr static size_t mount_fail_freq_len = 3;
    constexpr static uint16_t mount_fail_freqs[mount_fail_freq_len] = {1000, 100, 300};
    constexpr static uint16_t mount_fail_timings[mount_fail_freq_len] = {1000, 1000, 1000};

    constexpr static size_t fault_freqs_len = 5;
    constexpr static uint16_t fault_freqs[fault_freqs_len] = {3000, 200, 500, 1000, 0};
    constexpr static uint16_t fault_timings[fault_freqs_len] = {100, 190, 200, 300, 3000};

    void register_command(const std::string& command, const std::string& input_prompt, CommandInputType command_input,
                          command_callback_t callback);

    void send_framework_metadata(bool write_to_file, bool write_to_serial);
    void send_persistent_state(const void* data, size_t data_len);
    void set_fault(EFaultKey fault_key, bool fault_state, const std::string& message, bool write_to_logger);

    void set_flight_phase(EFlightPhase new_phase, bool lock_curr_phase_mtx);
    [[nodiscard]] uint8_t get_saved_phase_value() const;
    void set_curr_phase_speaker_pattern();
  };
}

FRAMEWORK_TEMPLATE_DECL
elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::ElijahStateFramework(
  std::string application_name, const size_t state_history_size) :
  application_name(std::move(application_name)),
  state_history_size(state_history_size)
{
  critical_section_init(&internal::usb_cs);
  shared_mutex_init(&logger_smtx);
  shared_mutex_init(&state_history_smtx);

  flight_phase_controller = new TFlightPhaseController();
  current_phase = flight_phase_controller->initial_flight_phase();
  mutex_init(&current_phase_mtx);

  persistent_data_storage->on_commit([this](const void* data, const size_t data_len)
  {
    send_persistent_state(data, data_len);
  });

  register_command("_", [this]
  {
    send_framework_metadata(false, true);
  });

  // ReSharper disable once CppPassValueParameterByConstReference
  register_command("New launch", "Launch name", true, [this](std::string launch_name)
  {
    shared_mutex_enter_blocking_exclusive(&logger_smtx);

    logger->flush_log();
    delete logger;
    did_write_metadata = false;

    logger = new StateFrameworkLogger(launch_name);
    remount_attempts = 0;
    shared_mutex_exit_exclusive(&logger_smtx);

    send_framework_metadata(true, false);

    persistent_data_storage->set_string(this->launch_key, launch_name);
    persistent_data_storage->commit_data();

    log_message(std::format("New launch: {}", launch_name));
  });

  register_command("Next flight phase", [this]
  {
    lock_state_history();
    if (state_history.empty())
    {
      log_serial_message("No state history, can not go to next phase");
      release_state_history();
    }
    const TStateData last_state = state_history.front();
    release_state_history();

    mutex_enter_blocking(&current_phase_mtx);
    const EFlightPhase next_phase = flight_phase_controller->force_next_phase(current_phase, last_state);
    set_flight_phase(next_phase, false);
    mutex_exit(&current_phase_mtx);
  });

  register_command("Restart", [this]
  {
    log_message("Restarting...", LogLevel::Warning);

    shared_mutex_enter_blocking_exclusive(&logger_smtx);
    logger->flush_log();
    delete logger;

    watchdog_enable(100, true);
    sleep_ms(500);
  });

  register_command("Toggle speaker", [this]
  {
    if (speaker_controller::toggle_speaker())
    {
      log_serial_message("Speaker is muted");
    }
    else
    {
      log_serial_message("Speaker is unmuted");
    }
  });
  std::string rand_launch_name = std::format("launch-{:016x}", get_rand_64());

  persistent_data_storage->register_key(launch_key, "Launch key", rand_launch_name);
  persistent_data_storage->register_key(flight_phase_key, "Flight phase", static_cast<uint8_t>(current_phase));

  set_curr_phase_speaker_pattern();
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::finish_construction()
{
  const uint8_t stored_phase = persistent_data_storage->get_uint8(flight_phase_key);
  if ((stored_phase & 0x80) > 0)
  {
    current_phase = static_cast<EFlightPhase>(stored_phase & 0x3F);
  }

  if ((stored_phase & 0x40) > 0)
  {
    did_write_metadata = true;
  }
  else
  {
    did_write_metadata = false;
  }

  logger = new StateFrameworkLogger(persistent_data_storage->get_string(launch_key));

  TStateData collection_data;
  encode_state(nullptr, collection_data, 0, true);

  speaker_controller::internal::play_current_freq();

  persistent_data_storage->lock_active_data();
  send_persistent_state(persistent_data_storage->get_active_data_loc(), persistent_data_storage->get_total_byte_size());
  persistent_data_storage->release_active_data();
}

FRAMEWORK_TEMPLATE_DECL
elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::~ElijahStateFramework()
{
  delete persistent_data_storage;

  shared_mutex_enter_blocking_exclusive(&logger_smtx);
  delete logger;
  logger = nullptr;
  shared_mutex_exit_exclusive(&logger_smtx);

  delete fault_manager;
  delete flight_phase_controller;
}

FRAMEWORK_TEMPLATE_DECL
elijah_state_framework::PersistentDataStorage<EPersistentStorageKey>* elijah_state_framework::ElijahStateFramework<
  FRAMEWORK_TEMPLATE_TYPES>::get_persistent_storage() const
{
  return persistent_data_storage;
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::check_for_commands()
{
  if (!stdio_usb_connected())
  {
    return;
  }

  critical_section_enter_blocking(&internal::usb_cs);

  uint8_t command_input[3];
  int bytes_read = stdio_get_until(reinterpret_cast<char*>(command_input), 3, delayed_by_ms(get_absolute_time(), 50));
  if (bytes_read != 3)
  {
    critical_section_exit(&internal::usb_cs);
    return;
  }

  if (command_input[0] != 0xAA || command_input[1] != 0x8E)
  {
    internal::log_serial_message_with_lock_opt(std::format("Invalid command prefix bytes received: 0x{:02X} and 0x{:02X}", command_input[0], command_input[1]), false);
    critical_section_exit(&internal::usb_cs);
    return;
  }

  uint8_t command_id = command_input[2];
  internal::log_serial_message_with_lock_opt(std::format("Received command: 0x{:02X}", command_id), false);

  if (!registered_commands.contains(command_id))
  {
    critical_section_exit(&internal::usb_cs);
    return;
  }
  const RegisteredCommand* command = &registered_commands[command_id];

  switch (command->get_input_type())
  {
  case CommandInputType::Double:
    {
      double double_arg;
      bytes_read = stdio_get_until(reinterpret_cast<char*>(&double_arg), sizeof(double_arg),
                                   delayed_by_ms(get_absolute_time(), 50));
      if (bytes_read != sizeof(double_arg))
      {
        internal::log_serial_message_with_lock_opt(
          std::format("Could not read double for command, only read {}/{} bytes", bytes_read, sizeof(double_arg)),
          false);
        critical_section_exit(&internal::usb_cs);
        return;
      }

      internal::log_serial_message_with_lock_opt(
        std::format("Executing double command \"{}\" (0x{:02X}) with argument: {:.5f}", command->get_command_name(),
                    command->get_command_id(), double_arg),
        false);
      critical_section_exit(&internal::usb_cs);

      const auto cb = std::get<std::function<void(double)>>(command->get_callback());
      cb(double_arg);
      break;
    }
  case CommandInputType::AlphaNumeric:
  case CommandInputType::String:
    {
      uint8_t str_size_buf[2];
      bytes_read = stdio_get_until(reinterpret_cast<char*>(str_size_buf), 2, delayed_by_ms(get_absolute_time(), 5000));
      if (bytes_read != 2)
      {
        internal::log_serial_message_with_lock_opt(
          std::format("Could not read string size for string command, only read {}/2 bytes", bytes_read), false);
        critical_section_exit(&internal::usb_cs);
        return;
      }

      const uint16_t str_size = *reinterpret_cast<uint16_t*>(str_size_buf);
      char str_buff[str_size + 1];
      bytes_read = stdio_get_until(str_buff, str_size, delayed_by_ms(get_absolute_time(), str_size * 1000));

      if (bytes_read != str_size)
      {
        internal::log_serial_message_with_lock_opt(
          std::format("Could not read string after reading size, only read {}/{} bytes", bytes_read, str_size), false);
        critical_section_exit(&internal::usb_cs);
        return;
      }
      str_buff[str_size] = '\0';
      const auto str_arg = std::string(str_buff);

      internal::log_serial_message_with_lock_opt(
        std::format("Executing string command \"{}\" (0x{:02X}) with argument: {}", command->get_command_name(),
                    command->get_command_id(), str_arg), false);
      critical_section_exit(&internal::usb_cs);

      const auto cb = std::get<std::function<void(std::string)>>(command->get_callback());
      cb(str_arg);
      break;
    }
  case CommandInputType::Time:
    {
      uint8_t tm_buff[data_type_helpers::get_size_for_data_type(DataType::Time)];
      bytes_read = stdio_get_until(reinterpret_cast<char*>(tm_buff),
                                   static_cast<int>(data_type_helpers::get_size_for_data_type(DataType::Time)),
                                   delayed_by_ms(get_absolute_time(), 1000));
      if (bytes_read != data_type_helpers::get_size_for_data_type(DataType::Time))
      {
        internal::log_serial_message_with_lock_opt(
          std::format("Failed to read data for time command, only read {}/{} bytes", bytes_read,
                      data_type_helpers::get_size_for_data_type(DataType::Time)), false);
        critical_section_exit(&internal::usb_cs);
        return;
      }

      const tm time_arg = internal::decode_time(tm_buff);

      internal::log_serial_message_with_lock_opt(std::format("Executing time command \"{}\" (0x{:02X})",
                                                             command->get_command_name(), command->get_command_id()),
                                                 false);
      critical_section_exit(&internal::usb_cs);

      const auto cb = std::get<std::function<void(tm)>>(command->get_callback());
      cb(time_arg);
      break;
    }
  case CommandInputType::None:
  default:
    internal::log_serial_message_with_lock_opt(std::format("Executing command \"{}\" (0x{:02X})",
                                                           command->get_command_name(), command->get_command_id()),
                                               false);
    critical_section_exit(&internal::usb_cs);

    const auto cb = std::get<std::function<void()>>(command->get_callback());
    cb();
    break;
  }
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::state_changed(const TStateData& new_state)
{
  if (!did_write_metadata)
  {
    send_framework_metadata(true, false);
  }

  const size_t total_encoded_packet_size = encoded_state_size + 1;
  uint8_t encoded_output_packet[total_encoded_packet_size] = {
    static_cast<uint8_t>(internal::OutputPacket::StateUpdate)
  };
  encode_state(encoded_output_packet + 1, new_state);

  shared_mutex_enter_blocking_exclusive(&state_history_smtx);
  if (state_history.size() >= state_history_size)
  {
    state_history.pop_back();
  }
  state_history.push_front(new_state);
  shared_mutex_exit_exclusive(&state_history_smtx);

  shared_mutex_enter_blocking_shared(&state_history_smtx);
  EFlightPhase new_phase = flight_phase_controller->update_phase(current_phase, state_history);
  shared_mutex_exit_shared(&state_history_smtx);

  bool phase_changed = false;
  size_t phase_change_packet_size = sizeof(uint8_t) /* output packet */ + sizeof(uint8_t) /* new state */;
  uint8_t* phase_change_packet = nullptr;
  if (new_phase != current_phase)
  {
    phase_changed = true;

    mutex_enter_blocking(&current_phase_mtx);
    current_phase = new_phase;
    mutex_exit(&current_phase_mtx);

    const std::string phase_name = flight_phase_controller->get_phase_name(current_phase);
    phase_change_packet_size += phase_name.length() + 1;
    phase_change_packet = new uint8_t[phase_change_packet_size]{
      static_cast<uint8_t>(internal::OutputPacket::PhaseChanged)
    };
    phase_change_packet[1] = static_cast<uint8_t>(current_phase);
    memcpy(phase_change_packet + 2, phase_name.c_str(), phase_name.size() + 1);
  }

  if (stdio_usb_connected())
  {
    critical_section_enter_blocking(&internal::usb_cs);
    internal::write_to_serial(encoded_output_packet, total_encoded_packet_size, !phase_changed);
    if (phase_changed)
    {
      internal::write_to_serial(phase_change_packet, phase_change_packet_size);
    }
    critical_section_exit(&internal::usb_cs);
  }

  gpio_put(25, true);
  shared_mutex_enter_blocking_shared(&logger_smtx);
  if (logger && did_write_metadata)
  {
    if (require_writing_restart_marker)
    {
      require_writing_restart_marker = false;
      constexpr auto device_restart_marker = static_cast<uint8_t>(internal::OutputPacket::DeviceRestartMarker);
      logger->log_data(&device_restart_marker, 1);
    }

    logger->log_data(encoded_output_packet, total_encoded_packet_size);
    if (phase_changed)
    {
      logger->log_data(phase_change_packet, phase_change_packet_size);
    }
  }
  shared_mutex_exit_shared(&logger_smtx);
  gpio_put(25, false);

  if (phase_changed || persistent_data_storage->get_uint8(flight_phase_key) != get_saved_phase_value())
  {
    persistent_data_storage->set_uint8(flight_phase_key, get_saved_phase_value());
    persistent_data_storage->commit_data();
  }

  delete [] phase_change_packet;

  if (fault_manager->get_all_faults() > 0)
  {
    speaker_controller::internal::play_status_freqs(fault_freqs, fault_timings, fault_freqs_len);
  }
  else
  {
    set_curr_phase_speaker_pattern();
  }
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::lock_state_history()
{
  shared_mutex_enter_blocking_shared(&state_history_smtx);
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::release_state_history()
{
  shared_mutex_exit_shared(&state_history_smtx);
}

FRAMEWORK_TEMPLATE_DECL
const std::deque<TStateData>& elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::
get_state_history() const
{
  return state_history;
}

FRAMEWORK_TEMPLATE_DECL
TFlightPhaseController* elijah_state_framework::ElijahStateFramework<
  FRAMEWORK_TEMPLATE_TYPES>::get_flight_phase_controller() const
{
  return flight_phase_controller;
}

FRAMEWORK_TEMPLATE_DECL
EFlightPhase elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::get_current_flight_phase()
{
  mutex_enter_blocking(&current_phase_mtx);
  EFlightPhase p = current_phase;
  mutex_exit(&current_phase_mtx);
  return p;
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::set_flight_phase(EFlightPhase new_phase)
{
  set_flight_phase(new_phase, true);
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::set_fault(
  EFaultKey fault_key,
  const bool fault_state)
{
  set_fault(fault_key, fault_state, "");
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::set_fault(
  EFaultKey fault_key, bool fault_state, const std::string& message)
{
  set_fault(fault_key, fault_state, message, true);
}

FRAMEWORK_TEMPLATE_DECL
bool elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::is_faulted(
  EFaultKey fault_key)
{
  return fault_manager->is_faulted(fault_key);
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::encode_state(
  void* encode_dest,
  const TStateData& state)
{
  encode_state(encode_dest, state, state_seq, false);
  state_seq++;
}

FRAMEWORK_TEMPLATE_DECL
const std::string& elijah_state_framework::ElijahStateFramework<TStateData, EPersistentStorageKey, EFaultKey,
                                                                EFlightPhase,
                                                                TFlightPhaseController>::get_application_name() const
{
  return application_name;
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::log_message(
  const std::string& message,
  const LogLevel log_level)
{
  size_t encoded_len;
  const std::unique_ptr<uint8_t[]> encoded_message = internal::encode_log_message(
    message, log_level, encoded_len);

  if (stdio_usb_connected())
  {
    critical_section_enter_blocking(&internal::usb_cs);
    internal::write_to_serial(encoded_message.get(), encoded_len);
    critical_section_exit(&internal::usb_cs);
  }

  if (log_level != LogLevel::Debug && logger)
  {
    shared_mutex_enter_blocking_shared(&logger_smtx);
    logger->log_data(encoded_message.get(), encoded_len);
    shared_mutex_exit_shared(&logger_smtx);
  }
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::check_for_log_write()
{
  shared_mutex_enter_blocking_shared(&logger_smtx);
  if (logger->is_mounted() || remount_attempts < MAX_MICRO_SD_LOG_WRITE_REMOUNT_ATTEMPTS)
  {
    bool did_try_mount, did_mount;
    const bool did_succeed = logger->flush_write_buff(did_try_mount, did_mount);

    if (did_try_mount)
    {
      remount_attempts++;
      if (did_mount)
      {
        set_fault(micro_sd_fault_key, false, std::format("Logger mounted while checking for log write in {}/{}",
                                                         remount_attempts, MAX_MICRO_SD_LOG_WRITE_REMOUNT_ATTEMPTS));
        remount_attempts = 0;
      }
      else
      {
        log_message(
          std::format(
            "Logger failed to mount while checking for log write ({}/{} remount attempts)",
            remount_attempts, MAX_MICRO_SD_LOG_WRITE_REMOUNT_ATTEMPTS),
          LogLevel::Debug);
        set_fault(micro_sd_fault_key, true, "Logger failed to mount while checking for log write", false);
      }
    }
    else if (!did_succeed)
    {
      set_fault(micro_sd_fault_key, true, "Failed to write data to MicroSD while checking for log writing", false);
    }
  }
  shared_mutex_exit_shared(&logger_smtx);
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::register_command(
  const std::string& command, std::function<void()> callback)
{
  register_command(command, "", CommandInputType::None, std::move(callback));
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::register_command(
  const std::string& command, const std::string& input_prompt, std::function<void(double)> callback)
{
  register_command(command, input_prompt, CommandInputType::Double, std::move(callback));
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::register_command(
  const std::string& command, const std::string& input_prompt, const bool is_alphanumeric,
  std::function<void(std::string)> callback)
{
  register_command(command, input_prompt, is_alphanumeric ? CommandInputType::AlphaNumeric : CommandInputType::String,
                   std::move(callback));
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::register_command(
  const std::string& command, const std::string& input_prompt, std::function<void(tm)> callback)
{
  register_command(command, input_prompt, CommandInputType::Time, std::move(callback));
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::register_data_variable(
  const std::string& display_name, const std::string& display_unit, const size_t offset, const DataType data_type)
{
  const uint8_t variable_id = variable_id_counter;
  variable_id_counter++;

  variable_definitions[variable_id] = VariableDefinition(variable_id, display_name,
                                                         display_unit, offset, data_type);
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::register_fault(
  EFaultKey key,
  std::string fault_name,
  CommunicationChannel communication_channel)
{
  fault_manager->register_fault(key, fault_name, communication_channel);
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::set_encoded_state_size(
  const size_t encoded_data_size)
{
  is_state_size_calculated = true;
  encoded_state_size = encoded_data_size;
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::register_command(
  const std::string& command, const std::string& input_prompt, const CommandInputType command_input,
  command_callback_t callback)
{
  const uint8_t new_id = command_id_counter++;
  registered_commands[new_id] = RegisteredCommand(new_id, command, input_prompt, command_input, std::move(callback));
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::send_framework_metadata(
  bool write_to_file, const bool write_to_serial)
{
  if (!write_to_file && !write_to_serial)
  {
    return;
  }

  if (write_to_serial)
  {
    critical_section_enter_blocking(&internal::usb_cs);
  }
  else if (!write_to_file)
  {
    return;
  }

  shared_mutex_enter_blocking_shared(&logger_smtx);

  if (write_to_file && logger)
  {
    if (!logger->is_mounted() && remount_attempts < MAX_MICRO_SD_METADATA_REMOUNT_ATTEMPTS && absolute_time_diff_us(
      last_mount_attempt_time, get_absolute_time()) / 1000 > 1500)
    {
      // If it has been more than 1.5s since we last tried to mount, try again
      last_mount_attempt_time = get_absolute_time();
      remount_attempts++;
      if (logger->mount_card())
      {
        last_mount_attempt_time = nil_time;
        remount_attempts = 0;
      }
      else
      {
        write_to_file = false;
        shared_mutex_exit_shared(&logger_smtx);

        set_fault(micro_sd_fault_key, true, "Failed to mount card while sending framework metadata", false);
        if (!write_to_serial)
        {
          log_message(
            std::format(
              "Logger failed to mount (and not writing to serial), can not write framework metadata ({}/{} remount attempts)",
              remount_attempts, MAX_MICRO_SD_METADATA_REMOUNT_ATTEMPTS),
            LogLevel::Debug);
          return;
        }
      }
    }

    if (logger->is_mounted())
    {
      if (!write_to_serial)
      {
        log_message("Logger mounted (and not writing to serial), writing framework metadata", LogLevel::Debug);
      }

      if (logger->did_load_file() && !logger->is_new_log_file())
      {
        did_write_metadata = true;
        shared_mutex_exit_shared(&logger_smtx);
        write_to_file = false;
      }

      if (write_to_file)
      {
        constexpr auto packet_id = static_cast<uint8_t>(internal::OutputPacket::Metadata);
        logger->log_data(&packet_id, sizeof(packet_id));
      }
    }
    else if (!write_to_serial)
    {
      shared_mutex_exit_shared(&logger_smtx);
      return;
    }
  }
  else
  {
    shared_mutex_exit_shared(&logger_smtx);

    if (!write_to_serial)
    {
      critical_section_enter_blocking(&internal::usb_cs);
      return;
    }
  }

  auto segment_id = static_cast<uint8_t>(internal::MetadataSegment::ApplicationName);
  const size_t initial_size = sizeof(FRAMEWORK_TAG) + sizeof(uint8_t) + application_name.size() + 1;
  const auto initial_data = new uint8_t[initial_size];

  *reinterpret_cast<uint64_t*>(initial_data) = FRAMEWORK_TAG;
  initial_data[sizeof(FRAMEWORK_TAG)] = segment_id;
  memcpy(initial_data + sizeof(FRAMEWORK_TAG) + sizeof(segment_id), application_name.c_str(),
         application_name.size() + 1);

  if (write_to_serial)
  {
    internal::write_to_serial(initial_data, initial_size, false);
  }
  if (write_to_file && logger)
  {
    logger->log_data(initial_data + sizeof(FRAMEWORK_TAG), sizeof(uint8_t) + application_name.size() + 1);
  }

  const uint8_t command_count = registered_commands.size();
  if (write_to_serial && command_count > 0)
  {
    segment_id = static_cast<uint8_t>(internal::MetadataSegment::Commands);
    const uint8_t segment_header[2] = {segment_id, command_count};

    internal::write_to_serial(segment_header, 2, false);
    for (const auto& command : std::views::values(registered_commands))
    {
      size_t encoded_size;
      std::unique_ptr<uint8_t[]> encoded = command.encode_command(encoded_size);
      internal::write_to_serial(encoded.get(), encoded_size, false);
    }
  }

  const uint8_t var_count = variable_definitions.size();
  if (var_count > 0)
  {
    segment_id = static_cast<uint8_t>(internal::MetadataSegment::VariableDefinitions);
    const uint8_t segment_header[3] = {segment_id, var_count, static_cast<uint8_t>(sizeof(size_t))};
    if (write_to_serial)
    {
      internal::write_to_serial(segment_header, 3, false);
    }
    if (write_to_file && logger)
    {
      logger->log_data(segment_header, 3);
    }

    for (const auto& var_def : std::views::values(variable_definitions))
    {
      size_t encoded_size;
      std::unique_ptr<uint8_t[]> encoded = var_def.encode_var(encoded_size);
      if (write_to_serial)
      {
        internal::write_to_serial(encoded.get(), encoded_size, false);
      }
      if (write_to_file && logger)
      {
        logger->log_data(encoded.get(), encoded_size);
      }
    }
  }

  // Always has launch key
  segment_id = static_cast<uint8_t>(internal::MetadataSegment::PersistentStorageEntries);
  const uint8_t persistent_data_segment_header[3] = {
    segment_id, static_cast<uint8_t>(persistent_data_storage->get_entry_count()), static_cast<uint8_t>(sizeof(size_t))
  };
  if (write_to_serial)
  {
    internal::write_to_serial(persistent_data_segment_header, 3, false);
  }
  if (write_to_file && logger)
  {
    logger->log_data(persistent_data_segment_header, 3);
  }

  size_t encoded_size;
  std::unique_ptr<uint8_t[]> encoded_data = persistent_data_storage->encode_all_entries(encoded_size);

  // We need to flush here so that we can dump persistent storage
  if (write_to_serial)
  {
    internal::write_to_serial(encoded_data.get(), encoded_size);
  }
  if (write_to_file && logger)
  {
    logger->log_data(encoded_data.get(), encoded_size);
    logger->flush_log();
  }

  persistent_data_storage->lock_active_data();
  if (write_to_serial)
  {
    internal::write_to_serial(
      static_cast<uint8_t*>(persistent_data_storage->get_active_data_loc()),
      persistent_data_storage->get_total_byte_size());
  }
  if (write_to_file && logger)
  {
    logger->log_data(static_cast<uint8_t*>(persistent_data_storage->get_active_data_loc()),
                     persistent_data_storage->get_total_byte_size());
  }
  persistent_data_storage->release_active_data();

  // Always has communication channels, so we can skip size checks
  segment_id = static_cast<uint8_t>(internal::MetadataSegment::FaultInformation);
  const uint32_t all_faults = fault_manager->get_all_faults();

  constexpr size_t header_len = 2 * sizeof(uint8_t) + sizeof(uint32_t);
  uint8_t fault_segment_header[header_len] = {
    segment_id, static_cast<uint8_t>(fault_manager->get_fault_count())
  };
  memcpy(fault_segment_header + 2, &all_faults, sizeof(uint32_t));

  if (write_to_serial)
  {
    internal::write_to_serial(fault_segment_header, header_len, false);
  }
  if (write_to_file && logger)
  {
    logger->log_data(fault_segment_header, header_len);
  }

  encoded_data = fault_manager->encode_all_faults(encoded_size);
  if (write_to_serial)
  {
    internal::write_to_serial(encoded_data.get(), encoded_size, false);
  }
  if (write_to_file && logger)
  {
    logger->log_data(encoded_data.get(), encoded_size);
  }

  segment_id = static_cast<uint8_t>(internal::MetadataSegment::InitialPhase);
  const std::string curr_phase_name = flight_phase_controller->get_phase_name(current_phase);
  const size_t phase_change_size = 2 * sizeof(uint8_t) + curr_phase_name.size() + 1;
  uint8_t phase_change_packet[encoded_size] = {segment_id, static_cast<uint8_t>(current_phase)};
  memcpy(phase_change_packet + 2, curr_phase_name.c_str(), curr_phase_name.size() + 1);
  if (write_to_serial)
  {
    internal::write_to_serial(phase_change_packet, phase_change_size, false);
  }

  if (write_to_file && logger)
  {
    logger->log_data(phase_change_packet, phase_change_size);
  }

  segment_id = static_cast<uint8_t>(internal::MetadataSegment::MetadataEnd);


  if (write_to_file && logger)
  {
    logger->log_data(&segment_id, 1);

    if (!write_to_serial)
    {
      log_message("Will flush before exiting send_framework_metadata()", LogLevel::Debug);
    }
    did_write_metadata = logger->flush_log();
    shared_mutex_exit_shared(&logger_smtx);
  }

  if (write_to_serial)
  {
    internal::write_to_serial(&segment_id, 1);
    critical_section_exit(&internal::usb_cs);
  }

  if (write_to_file)
  {
    set_fault(micro_sd_fault_key, !did_write_metadata,
              did_write_metadata
                ? "Successfully wrote state framework metadata"
                : "Failed to write state framework metadata", did_write_metadata);
  }
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::send_persistent_state(
  const void* data, const size_t data_len)
{
  critical_section_enter_blocking(&internal::usb_cs);
  constexpr auto packet_id = static_cast<uint8_t>(internal::OutputPacket::PersistentStateUpdate);

  // Data length will be parsed by pre-existing persistent data segments
  internal::write_to_serial(&packet_id, 1, false);
  internal::write_to_serial(static_cast<const uint8_t*>(data), data_len);

  critical_section_exit(&internal::usb_cs);

  shared_mutex_enter_blocking_shared(&logger_smtx);
  if (logger)
  {
    if (!did_write_metadata)
    {
      send_framework_metadata(true, false);
    }

    logger->flush_log();
    if (did_write_metadata)
    {
      logger->log_data(&packet_id, 1);
      logger->log_data(static_cast<const uint8_t*>(data), data_len);
      logger->flush_log();
    }
  }
  shared_mutex_exit_shared(&logger_smtx);
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::set_fault(
  EFaultKey fault_key, bool fault_state, const std::string& message, bool write_to_logger)
{
  uint8_t fault_bit;
  bool did_fault_change;
  const uint32_t faults = fault_manager->set_fault_status(fault_key, fault_state, fault_bit, did_fault_change, message);

  if (!did_fault_change)
  {
    return;
  }

  const size_t encoded_size = 2 * sizeof(uint8_t) /* output packet, fault_bit */ + sizeof(uint32_t) + message.size() +
    1;
  uint8_t encoded_data[encoded_size];

  encoded_data[0] = static_cast<uint8_t>(internal::OutputPacket::FaultsChanged);
  encoded_data[1] = fault_bit;
  memcpy(encoded_data + 2 * sizeof(uint8_t), &faults, sizeof(uint32_t));
  memcpy(encoded_data + 2 * sizeof(uint8_t) + sizeof(uint32_t), message.c_str(), message.size() + 1);

  critical_section_enter_blocking(&internal::usb_cs);
  internal::write_to_serial(encoded_data, encoded_size);
  critical_section_exit(&internal::usb_cs);

  if (write_to_logger)
  {
    shared_mutex_enter_blocking_shared(&logger_smtx);
    if (logger)
    {
      if (!did_write_metadata)
      {
        send_framework_metadata(true, false);
      }

      if (did_write_metadata)
      {
        logger->log_data(encoded_data, encoded_size);
      }
    }
    shared_mutex_exit_shared(&logger_smtx);
  }
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::set_flight_phase(
  EFlightPhase new_phase, bool lock_curr_phase_mtx)
{
  if (lock_curr_phase_mtx)
  {
    mutex_enter_blocking(&current_phase_mtx);
  }
  if (current_phase == new_phase)
  {
    if (lock_curr_phase_mtx)
    {
      mutex_exit(&current_phase_mtx);
    }
    return;
  }
  current_phase = new_phase;

  size_t phase_change_packet_size = sizeof(uint8_t) /* output packet */ + sizeof(uint8_t) /* new state */;
  const std::string phase_name = flight_phase_controller->get_phase_name(current_phase);
  phase_change_packet_size += phase_name.length() + 1;
  uint8_t phase_change_packet[phase_change_packet_size] = {
    static_cast<uint8_t>(internal::OutputPacket::PhaseChanged)
  };

  phase_change_packet[1] = static_cast<uint8_t>(current_phase);
  memcpy(phase_change_packet + 2, phase_name.c_str(), phase_name.size() + 1);

  persistent_data_storage->set_uint8(flight_phase_key, get_saved_phase_value());
  persistent_data_storage->commit_data();

  if (lock_curr_phase_mtx)
  {
    mutex_exit(&current_phase_mtx);
  }

  if (stdio_usb_connected())
  {
    critical_section_enter_blocking(&internal::usb_cs);
    internal::write_to_serial(phase_change_packet, phase_change_packet_size);
    critical_section_exit(&internal::usb_cs);
  }

  shared_mutex_enter_blocking_shared(&logger_smtx);
  if (logger && did_write_metadata)
  {
    logger->log_data(phase_change_packet, phase_change_packet_size);
  }
  shared_mutex_exit_shared(&logger_smtx);
}

FRAMEWORK_TEMPLATE_DECL
uint8_t elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::get_saved_phase_value() const
{
  return (0x80 | (did_write_metadata ? 0x40 : 0x00) | static_cast<uint8_t>(current_phase));
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>::set_curr_phase_speaker_pattern()
{
  const uint16_t* pattern;
  const uint16_t* pattern_timings;

  mutex_enter_blocking(&current_phase_mtx);
  const size_t pattern_len = flight_phase_controller->get_speaker_pattern(current_phase, pattern, pattern_timings);
  mutex_exit(&current_phase_mtx);

  speaker_controller::internal::play_status_freqs(pattern, pattern_timings, pattern_len);
}
