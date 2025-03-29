#include "registered_command.h"

#include <cstdint>
#include <string>
#include <utility>

#include "elijah_state_framework.h"

RegisteredCommand::RegisteredCommand() : command_id(0xFF),
                                         command_input(CommandInputType::None),
                                         callback({})
{
}

RegisteredCommand::RegisteredCommand(const uint8_t command_id, std::string command_name, std::string input_prompt,
                                     const CommandInputType command_input,
                                     command_callback_t callback) : command_id(command_id),
                                                                    command_name(std::move(command_name)),
                                                                    input_prompt(std::move(input_prompt)),
                                                                    command_input(command_input),
                                                                    callback(std::move(callback))
{
}

uint8_t RegisteredCommand::get_command_id() const
{
  return command_id;
}

const std::string& RegisteredCommand::get_command_name() const
{
  return command_name;
}

const std::string& RegisteredCommand::get_input_prompt() const
{
  return input_prompt;
}

CommandInputType RegisteredCommand::get_input_type() const
{
  return command_input;
}

const command_callback_t& RegisteredCommand::get_callback() const
{
  return callback;
}

std::unique_ptr<uint8_t[]> RegisteredCommand::encode_command(size_t& encoded_size) const
{
  encoded_size = sizeof(command_id) + sizeof(command_input) + command_name.size() + 1 + input_prompt.size() + 1;
  std::unique_ptr<uint8_t[]> encoded_command(new uint8_t[encoded_size]);

  encoded_command.get()[0] = command_id;
  encoded_command.get()[sizeof(command_id)] = static_cast<uint8_t>(command_input);
  memcpy(encoded_command.get() + sizeof(command_id) + sizeof(command_input), command_name.c_str(),
         command_name.size() + 1);
  memcpy(encoded_command.get() + sizeof(command_id) + sizeof(command_input) + command_name.size() + 1,
         input_prompt.c_str(), input_prompt.size() + 1);

  return encoded_command;
}
