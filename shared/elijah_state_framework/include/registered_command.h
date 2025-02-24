#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <variant>

using command_callback_t = std::variant<std::function<void()>, std::function<void(double)>, std::function<void
                                          (std::string)>, std::function<void(tm)>>;

enum class CommandInputType : uint8_t
{
  NONE = 0,
  DOUBLE = 1,
  ALPHANUMERIC = 2,
  STRING = 3,
  TIME = 4
};

struct RegisteredCommand
{
  uint8_t command_id;
  std::string command;
  CommandInputType command_input;
  command_callback_t callback;

  RegisteredCommand();
  RegisteredCommand(uint8_t command_id, std::string command, CommandInputType command_input,
                    command_callback_t callback);
};
