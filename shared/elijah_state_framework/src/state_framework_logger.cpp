#include "state_framework_logger.h"

#include <utility>
#include <sd_card.h>

#include "usb_comm.h"

elijah_state_framework::StateFrameworkLogger::StateFrameworkLogger(std::string file_name) : file_name(
  std::move(file_name))
{
  mutex_init(&log_buff_mtx);
  recursive_mutex_init(&write_buff_rmtx);

  load_old_data();
}

bool elijah_state_framework::StateFrameworkLogger::init_driver_on_core()
{
  return sd_init_driver();
}

bool elijah_state_framework::StateFrameworkLogger::is_new_file() const
{
  return !was_file_existing;
}

void elijah_state_framework::StateFrameworkLogger::log_data(const uint8_t* data, const size_t len)
{
  assert(len <= LOG_BUFF_SIZE);

  mutex_enter_blocking(&log_buff_mtx);
  if (log_size + len > LOG_BUFF_SIZE)
  {
    move_to_write_buff();
  }

  memcpy(log_buff.get() + log_size, data, len);
  log_size += len;

  mutex_exit(&log_buff_mtx);
}

bool elijah_state_framework::StateFrameworkLogger::flush_log()
{
  recursive_mutex_enter_blocking(&write_buff_rmtx);
  mutex_enter_blocking(&log_buff_mtx);
  bool did_flush = flush_write_buff();
  move_to_write_buff();
  did_flush = did_flush && flush_write_buff();
  mutex_exit(&log_buff_mtx);
  recursive_mutex_exit(&write_buff_rmtx);

  return did_flush;
}

bool elijah_state_framework::StateFrameworkLogger::flush_write_buff()
{
  recursive_mutex_enter_blocking(&write_buff_rmtx);

  if (write_size == 0)
  {
    recursive_mutex_exit(&write_buff_rmtx);
    return true;
  }

  FATFS fs;
  FIL fil;
  FRESULT fr = f_mount(&fs, "0:", 1);
  if (fr != FR_OK)
  {
    recursive_mutex_exit(&write_buff_rmtx);
    log_serial_message("Will not flush write buffer, failed to mount file system");
    return false;
  }

  fr = f_open(&fil, file_name.c_str(), FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
  if (fr != FR_OK)
  {
    f_unmount("");
    recursive_mutex_exit(&write_buff_rmtx);
    log_serial_message("Will not flush write buffer, failed to open file");
    return false;
  }

  size_t bytes_written;
  if (f_size(&fil) == 0)
  {
    constexpr decltype(next_log_pos) log_pos = 0;
    fr = f_write(&fil, &log_pos, sizeof(next_log_pos), &bytes_written);
  }
  else
  {
    fr = f_lseek(&fil, next_log_pos);
  }

  if (fr != FR_OK)
  {
    f_close(&fil);
    f_unmount("");
    recursive_mutex_exit(&write_buff_rmtx);
    log_serial_message("Will not flush write buffer, failed to seek to or write next log position");
    return false;
  }

  fr = f_write(&fil, write_buff.get(), write_size, &bytes_written);
  if (fr != FR_OK || bytes_written != write_size)
  {
    f_close(&fil);
    f_unmount("");
    recursive_mutex_exit(&write_buff_rmtx);
    log_serial_message("Will not flush write buffer, failed to write data");
    return false;
  }
  next_log_pos += write_size;

  fr = f_lseek(&fil, 0);
  if (fr != FR_OK || bytes_written != write_size)
  {
    f_close(&fil);
    f_unmount("");
    recursive_mutex_exit(&write_buff_rmtx);
    log_serial_message("Failed to update log position, could not seek to beginning of file to write next log position");
    return false;
  }

  fr = f_write(&fil, &next_log_pos, sizeof(next_log_pos), &bytes_written);
  f_close(&fil);
  f_unmount("");

  if (fr != FR_OK)
  {
    recursive_mutex_exit(&write_buff_rmtx);
    log_serial_message("Failed to update log position, unable to write at beginning of file");
    return false;
  }

  write_buff.reset();
  write_size = 0;

  recursive_mutex_exit(&write_buff_rmtx);
  return true;
}

void elijah_state_framework::StateFrameworkLogger::move_to_write_buff()
{
  recursive_mutex_enter_blocking(&write_buff_rmtx);
  write_buff = std::move(log_buff);
  write_size = log_size;
  recursive_mutex_exit(&write_buff_rmtx);

  log_buff = std::unique_ptr<uint8_t[]>(new uint8_t[LOG_BUFF_SIZE]);
  log_size = 0;
}

void elijah_state_framework::StateFrameworkLogger::load_old_data()
{
  FATFS fs;
  FIL fil;
  FRESULT fr = f_mount(&fs, "0:", 1);
  if (fr != FR_OK)
  {
    return;
  }

  fr = f_stat(file_name.c_str(), nullptr);
  was_file_existing = fr != FR_NO_FILE;
  if (was_file_existing)
  {
    return;
  }

  fr = f_open(&fil, file_name.c_str(), FA_READ);
  if (fr != FR_OK)
  {
    f_unmount("");
    return;
  }

  fr = f_lseek(&fil, 0);
  if (fr != FR_OK)
  {
    f_close(&fil);
    f_unmount("");
    return;
  }

  size_t bytes_read;
  decltype(next_log_pos) read_log_pos;
  fr = f_read(&fil, &read_log_pos, sizeof(next_log_pos), &bytes_read);

  f_close(&fil);
  f_unmount("");

  if (fr == FR_OK)
  {
    next_log_pos = read_log_pos;
  }
}
