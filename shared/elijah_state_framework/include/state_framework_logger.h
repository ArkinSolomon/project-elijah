#pragma once

#define LOG_BUFF_SIZE 1024
#include <cstdint>
#include <memory>
#include <string>
#include <pico/mutex.h>

class StateFrameworkLogger
{
public:
  explicit StateFrameworkLogger(std::string file_name);

  static bool init_driver_on_core();

  [[nodiscard]] bool is_new_file();

  void log_data(const uint8_t* data, size_t len);
  void flush_log();
  bool flush_write_buff();

private:
  mutex_t log_buff_mtx;
  recursive_mutex_t write_buff_rmtx;

  std::string file_name;
  bool was_file_created = false;
  uint64_t next_log_pos = sizeof(uint64_t);

  std::unique_ptr<uint8_t[]> log_buff = std::unique_ptr<uint8_t[]>(new uint8_t[LOG_BUFF_SIZE]);
  size_t log_size = 0;

  std::unique_ptr<uint8_t[]> write_buff = std::unique_ptr<uint8_t[]>(nullptr);
  size_t write_size = 0;

  void move_to_write_buff();
  void load_old_data();
};
