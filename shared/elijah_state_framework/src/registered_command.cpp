#include "registered_command.h"

#include <cstdint>
#include <string>
#include <utility>

#include "elijah_state_framework.h"

RegisteredCommand::RegisteredCommand()  : command_id(0xFF),
                          command_input(CommandInputType::NONE),
                          callback({})
{
}

RegisteredCommand::RegisteredCommand(const uint8_t command_id, std::string command, const CommandInputType command_input,
  command_callback_t callback) : command_id(command_id),
                                                                           command(std::move(command)),
                                                                           command_input(command_input),
                                                                           callback(std::move(callback))
{
}
