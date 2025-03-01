#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <variant>

using command_callback_t = std::variant<std::function<void()>, std::function<void(double)>, std::function<void
                                          (std::string)>, std::function<void(tm)>>;

enum class CommandInputType : uint8_t
{
  None = 0,
  Double = 1,
  AlphaNumeric = 2,
  String = 3,
  Time = 4
};

class RegisteredCommand
{
public:
  RegisteredCommand();
  RegisteredCommand(uint8_t command_id, std::string command_name, CommandInputType command_input,
                    command_callback_t callback);

  [[nodiscard]] uint8_t get_command_id() const;
  [[nodiscard]] const std::string& get_command_name() const;
  [[nodiscard]] CommandInputType get_input_type() const;
  [[nodiscard]] command_callback_t get_callback() const;

  [[nodiscard]] std::unique_ptr<uint8_t> encode_command(size_t& encoded_size) const;

private:
  uint8_t command_id;
  std::string command_name;
  CommandInputType command_input;
  command_callback_t callback;
};
