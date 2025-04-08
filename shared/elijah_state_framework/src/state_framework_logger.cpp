#include "state_framework_logger.h"

#include <utility>
#include <sd_card.h>

#include "usb_comm.h"

elijah_state_framework::StateFrameworkLogger::StateFrameworkLogger(std::string file_name) : file_name(
  std::move(file_name))
{
  if (!mutex_is_initialized(&sd_card_mtx))
  {
    mutex_init(&sd_card_mtx);
  }

  mutex_init(&log_buff_mtx);
  recursive_mutex_init(&write_buff_rmtx);

  mutex_enter_blocking(&sd_card_mtx);
  if (mount_card(false))
  {
    load_old_data();
  }
  mutex_exit(&sd_card_mtx);
}

elijah_state_framework::StateFrameworkLogger::~StateFrameworkLogger()
{
  f_unmount("");
}

bool elijah_state_framework::StateFrameworkLogger::init_driver_on_core()
{
  return sd_init_driver();
}

void elijah_state_framework::StateFrameworkLogger::log_data(const uint8_t* data, const size_t len)
{
  if (len > STATE_FRAMEWORK_LOG_BUFF_SIZE)
  {
    return;
  }

  mutex_enter_blocking(&log_buff_mtx);
  if (log_size + len > STATE_FRAMEWORK_LOG_BUFF_SIZE)
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

  bool _;
  bool did_flush = flush_write_buff(_, _);
  move_to_write_buff();
  did_flush = did_flush && flush_write_buff(_, _);

  mutex_exit(&log_buff_mtx);
  recursive_mutex_exit(&write_buff_rmtx);

  return did_flush;
}

bool elijah_state_framework::StateFrameworkLogger::flush_write_buff(bool& did_try_remount, bool& did_mount)
{
  recursive_mutex_enter_blocking(&write_buff_rmtx);

  did_try_remount = false;
  did_mount = false;
  if (write_size == 0)
  {
    recursive_mutex_exit(&write_buff_rmtx);
    return true;
  }

  mutex_enter_blocking(&sd_card_mtx);
  if (!mounted)
  {
    did_try_remount = true;
    if (mount_card(false))
    {
      did_mount = true;
    }
    else
    {
      did_mount = false;
      mutex_exit(&sd_card_mtx);
      recursive_mutex_exit(&write_buff_rmtx);
      return false;
    }
  }

  FIL fil;
  FRESULT fr = f_open(&fil, file_name.c_str(), FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
  if (fr != FR_OK)
  {
    f_unmount("");
    mounted = false;
    mutex_exit(&sd_card_mtx);
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
    mounted = false;
    mutex_exit(&sd_card_mtx);
    recursive_mutex_exit(&write_buff_rmtx);
    log_serial_message("Will not flush write buffer, failed to seek to or write next log position");
    return false;
  }

  fr = f_write(&fil, write_buff.get(), write_size, &bytes_written);
  if (fr != FR_OK || bytes_written != write_size)
  {
    f_close(&fil);
    f_unmount("");
    mounted = false;
    mutex_exit(&sd_card_mtx);
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
    mounted = false;
    mutex_exit(&sd_card_mtx);
    recursive_mutex_exit(&write_buff_rmtx);
    log_serial_message("Failed to update log position, could not seek to beginning of file to write next log position");
    return false;
  }

  fr = f_write(&fil, &next_log_pos, sizeof(next_log_pos), &bytes_written);
  f_close(&fil);

  if (fr != FR_OK)
  {
    f_unmount("");
    mounted = false;
    mutex_exit(&sd_card_mtx);
    recursive_mutex_exit(&write_buff_rmtx);
    log_serial_message("Failed to update log position, unable to write at beginning of file");
    return false;
  }

  write_buff.reset();
  write_size = 0;

  mutex_exit(&sd_card_mtx);
  recursive_mutex_exit(&write_buff_rmtx);
  return true;
}

bool elijah_state_framework::StateFrameworkLogger::is_mounted() const
{
  return mounted;
}

bool elijah_state_framework::StateFrameworkLogger::did_load_file() const
{
  return did_load_data;
}

bool elijah_state_framework::StateFrameworkLogger::is_new_log_file() const
{
  return is_new_file;
}

bool elijah_state_framework::StateFrameworkLogger::mount_card()
{
  return mount_card(true);
}

bool elijah_state_framework::StateFrameworkLogger::mount_card(const bool lock)
{
  if (lock)
  {
    mutex_enter_blocking(&sd_card_mtx);
  }
  const FRESULT fr = f_mount(&fs, "0:", 1);
  if (fr != FR_OK)
  {
    if (lock)
    {
      mutex_exit(&sd_card_mtx);
    }
    mounted = false;
    return false;
  }

  mounted = true;

  if (!did_load_data)
  {
    load_old_data();
    log_serial_message("Loaded old data");
    did_load_data = true;
  }

  if (lock)
  {
    mutex_exit(&sd_card_mtx);
  }

  return true;
}

void elijah_state_framework::StateFrameworkLogger::move_to_write_buff()
{
  recursive_mutex_enter_blocking(&write_buff_rmtx);
  write_buff = std::move(log_buff);
  write_size = log_size;
  recursive_mutex_exit(&write_buff_rmtx);

  log_buff = std::unique_ptr<uint8_t[]>(new uint8_t[STATE_FRAMEWORK_LOG_BUFF_SIZE]);
  log_size = 0;
}

void elijah_state_framework::StateFrameworkLogger::load_old_data()
{
  if (!mounted)
  {
    return;
  }

  FIL fil;
  FRESULT fr = f_stat(file_name.c_str(), nullptr);
  is_new_file = fr == FR_NO_FILE;
  if (is_new_file)
  {
    return;
  }

  fr = f_open(&fil, file_name.c_str(), FA_READ);
  if (fr != FR_OK)
  {
    f_unmount("");
    mounted = false;
    return;
  }

  fr = f_lseek(&fil, 0);
  if (fr != FR_OK)
  {
    f_unmount("");
    mounted = false;
    f_close(&fil);
    return;
  }

  size_t bytes_read;
  decltype(next_log_pos) read_log_pos;
  fr = f_read(&fil, &read_log_pos, sizeof(next_log_pos), &bytes_read);

  f_close(&fil);

  if (fr == FR_OK)
  {
    next_log_pos = read_log_pos;
  }
  else
  {
    f_unmount("");
    mounted = false;
  }
}
