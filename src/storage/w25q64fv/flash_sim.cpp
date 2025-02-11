#include <format>
#include <f_util.h>
#include <sd_card.h>
#include <string>
#include <pico/rand.h>

#include "lock_nums.h"
#include "usb_communication.h"
#include "w25q64fv.h"
#include "status_manager.h"

#ifdef W25Q64FV_FLASH_SIM

inline critical_section_t flash_sim_cs;
inline const auto filename = "flash_data.binary";

bool mount_card(FATFS* fs)
{
  const FRESULT fr = f_mount(fs, "0:", 1);
  if (fr != FR_OK)
  {
    usb_communication::send_string(std::format("Fault: microSD card failed to mount, error {} ({})", FRESULT_str(fr),
                                               static_cast<uint8_t>(fr)));
    set_fault(status_manager::MICRO_SD, true);
    return false;
  }

  set_fault(status_manager::MICRO_SD, false);
  return true;
}

bool open_file(FATFS* fs, FIL* fil, const BYTE mode)
{
  if (!mount_card(fs))
  {
    return false;
  }

  const FRESULT fr = f_open(fil, filename, mode);
  if (fr != FR_OK && fr != FR_EXIST)
  {
    usb_communication::send_string(std::format("Fault: could not open {} on microSD card, error: {} ({})",
                                               filename, FRESULT_str(fr), static_cast<uint8_t>(fr)));
    set_fault(status_manager::MICRO_SD, true);
    f_unmount("");
    return false;
  }

  return true;
}

bool try_close_and_unmount(FIL* fil)
{
  const FRESULT fr = f_close(fil);

  bool success = true;
  if (fr != FR_OK)
  {
    usb_communication::send_string(std::format(
      "Fault: could not close open file on microSD card, error: {} ({})",
      FRESULT_str(fr), static_cast<uint8_t>(fr)));
    set_fault(status_manager::MICRO_SD, true);
    success = false;
  }

  success = success && f_unmount("") == FR_OK;
  return success;
}

bool try_seek(FIL* fil, const FSIZE_t seek_pos)
{
  const FRESULT fr = f_lseek(fil, seek_pos);
  if (fr != FR_OK)
  {
    usb_communication::send_string(std::format("Fault: failed to seek to position {}, error: {} ({})",
                                               seek_pos, FRESULT_str(fr), static_cast<uint8_t>(fr)));
    set_fault(status_manager::MICRO_SD, true);
    try_close_and_unmount(fil);
    return false;
  }
  return true;
}

bool w25q64fv::init()
{
  usb_communication::send_string("Initializing flash simulator");
  critical_section_init_with_lock_num(&flash_sim_cs, LOCK_NUM_FLASH_SIM_CS);

  if (!sd_init_driver())
  {
    usb_communication::send_string("Fault: could not initialize SD card driver");
    set_fault(status_manager::MICRO_SD, true);
    return false;
  }

  manufacturer_id = WINBOND_MANUFACTURER_ID;
  manufacturer_device_id = 0xAA;
  memory_type = 0xBB;
  capacity = 0xCC;
  unique_id = get_rand_64();

  set_fault(status_manager::MICRO_SD, false);
  return true;
}

bool w25q64fv::write_enable()
{
  return true;
}

bool w25q64fv::write_disable()
{
  return true;
}

bool w25q64fv::write_sector(uint32_t sector_addr, const uint8_t* data, uint16_t data_len)
{
  if (sector_addr > LARGEST_SECTOR_ADDR)
  {
    // usb_communication::send_string(std::format("Sector address too large, can not simulate write to 0x{:06X}", sector_addr));
    // return false;
  }

  critical_section_enter_blocking(&flash_sim_cs);

  FATFS fs;
  FIL fil;
  FRESULT fr;

  if (!open_file(&fs, &fil, FA_OPEN_APPEND | FA_WRITE))
  {
    critical_section_exit(&flash_sim_cs);
    return false;
  }

  uint bytes_written;
  uint8_t sector_buffer[SECTOR_SIZE];
  std::fill_n(sector_buffer, SECTOR_SIZE, 0x01);

  if (fil.fptr < sector_addr)
  {
    size_t bytes_remaining = sector_addr - fil.fptr;
    size_t blank_bytes_written = 0;

    while (bytes_remaining > 0)
    {
      size_t write_size;
      if (bytes_remaining >= SECTOR_SIZE)
      {
        bytes_remaining -= SECTOR_SIZE;
        write_size = SECTOR_SIZE;
      }
      else
      {
        write_size = bytes_remaining;
        bytes_remaining = 0;
      }

      fr = f_write(&fil, sector_buffer, write_size, &bytes_written);
      if (fr != FR_OK || bytes_written != write_size)
      {
        usb_communication::send_string(std::format(
          "Fault: failed to write {} blank bytes (only wrote {} blank bytes) to microSD, error: {} ({}) f_ptr={}",
          write_size, bytes_written,
          FRESULT_str(fr), static_cast<uint8_t>(fr), fil.fptr));

        try_close_and_unmount(&fil);
        critical_section_exit(&flash_sim_cs);
        set_fault(status_manager::MICRO_SD, true);
        return false;
      }

      blank_bytes_written += bytes_written;
    }
  }

  if (!try_seek(&fil, sector_addr))
  {
    critical_section_exit(&flash_sim_cs);
    return false;
  }

  memcpy(sector_buffer, data, data_len);
  fr = f_write(&fil, sector_buffer, SECTOR_SIZE, &bytes_written);
  if (fr != FR_OK || bytes_written != SECTOR_SIZE)
  {
    usb_communication::send_string(std::format(
      "Fault: failed to write {} bytes (only wrote {}) (simulating sector address 0x{:06X}) to microSD, error: {} ({})",
      SECTOR_SIZE, sector_addr, bytes_written,
      FRESULT_str(fr), static_cast<uint8_t>(fr)));
    try_close_and_unmount(&fil);
    critical_section_exit(&flash_sim_cs);
    set_fault(status_manager::MICRO_SD, true);
    return false;
  }

  const bool did_unmount_succeed = try_close_and_unmount(&fil);
  critical_section_exit(&flash_sim_cs);
  set_fault(status_manager::MICRO_SD, !did_unmount_succeed);
  return did_unmount_succeed;
}

bool w25q64fv::page_program(uint32_t page_addr, const uint8_t* data, size_t data_len)
{
  usb_communication::send_string("Do not call w25q64fv::page_program when using the microSD card as a flash simulator");
  return false;
}

bool w25q64fv::read_data(uint32_t data_addr, uint8_t* data_buff, size_t data_len)
{
  const uint32_t end_addr = data_addr + data_len;
  if (end_addr > LARGEST_READABLE_ADDR)
  {
    usb_communication::send_string(std::format("Can not simulate read {} bytes from 0x{:06X}, would overflow flash", data_len,
                                               data_addr));
    return false;
  }

  FATFS fs;
  FIL fil;

  critical_section_enter_blocking(&flash_sim_cs);
  if (!open_file(&fs, &fil, FA_OPEN_APPEND | FA_READ))
  {
    critical_section_exit(&flash_sim_cs);
    return false;
  }

  if (fil.fptr >= end_addr)
  {
    if (!try_seek(&fil, data_addr))
    {
      critical_section_exit(&flash_sim_cs);
      return false;
    }

    size_t bytes_read;
    const FRESULT fr = f_read(&fil, data_buff, data_len, &bytes_read);
    if (fr != FR_OK)
    {
      usb_communication::send_string(std::format(
        "Fault: failed to read {} bytes (only read {}) (simulating address 0x{:06X}) from microSD, error: {} ({})",
        data_len, bytes_read, data_addr, FRESULT_str(fr), static_cast<uint8_t>(fr)));
      try_close_and_unmount(&fil);
      critical_section_exit(&flash_sim_cs);
      set_fault(status_manager::MICRO_SD, true);
      return false;
    }
  }
  else if (fil.fptr < data_addr)
  {
    usb_communication::send_string(std::format("No data to read, filling {} 0x01s", data_len));
    std::fill_n(data_buff, data_len, 0x01);
  }
  else
  {
    // data_addr <= fptr < end_addr

    const size_t avail_size = fil.fptr - data_addr + 1;
    const size_t fill_size = data_len - avail_size;
    std::fill_n(data_buff + avail_size, fill_size, 0x01);

    size_t bytes_read;
    const FRESULT fr = f_read(&fil, data_buff, avail_size, &bytes_read);
    if (fr != FR_OK)
    {
      usb_communication::send_string(std::format(
        "Fault: failed to read {} bytes (partial read) (only read {}) (simulating address 0x{:06X}) from microSD, error: {} ({})",
        data_len, bytes_read, data_addr, FRESULT_str(fr), static_cast<uint8_t>(fr)));
      try_close_and_unmount(&fil);
      critical_section_exit(&flash_sim_cs);
      set_fault(status_manager::MICRO_SD, true);
      return false;
    }
  }


  const bool did_unmount_succeed = try_close_and_unmount(&fil);
  critical_section_exit(&flash_sim_cs);
  set_fault(status_manager::MICRO_SD, !did_unmount_succeed);
  return did_unmount_succeed;
}

bool w25q64fv::chip_erase()
{
  FATFS fs;
  critical_section_enter_blocking(&flash_sim_cs);
  if (!mount_card(&fs))
  {
    critical_section_exit(&flash_sim_cs);
    return false;
  }

  FRESULT fr = f_stat(filename, nullptr);
  if (fr == FR_NO_FILE)
  {
    set_fault(status_manager::MICRO_SD, false);
    f_unmount("");
    critical_section_exit(&flash_sim_cs);
    return true;
  }

  if (fr != FR_OK)
  {
    usb_communication::send_string(std::format(
      "Fault: failed to determine if the flash simulator data file existed on the microSD card, error: {} ({})",
      FRESULT_str(fr), static_cast<uint8_t>(fr)));
    set_fault(status_manager::MICRO_SD, true);
    f_unmount("");
    critical_section_exit(&flash_sim_cs);
    return false;
  }

  fr = f_unlink(filename);
  f_unmount("");
  critical_section_exit(&flash_sim_cs);

  if (fr != FR_OK)
  {
    usb_communication::send_string(std::format("Fault, failed to delete microSD card file error: {} ({})",
                                               FRESULT_str(fr), static_cast<uint8_t>(fr)));
    set_fault(status_manager::MICRO_SD, true);
    return false;
  }

  set_fault(status_manager::MICRO_SD, false);
  return true;
}

bool w25q64fv::open_status_reg_1()
{
  return true;
}

bool w25q64fv::wait_for_wel(bool can_write)
{
  return true;
}

bool w25q64fv::wait_for_not_busy()
{
  return true;
}

#endif
