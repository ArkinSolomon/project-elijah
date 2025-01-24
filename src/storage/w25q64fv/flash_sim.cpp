#include <format>
#include <f_util.h>
#include <sd_card.h>
#include <string>
#include <pico/rand.h>

#include "pin_outs.h"
#include "usb_communication.h"
#include "w25q64fv.h"
#include "status_manager.h"

#ifdef W25Q64FV_FLASH_SIM

inline critical_section_t flash_sim_cs;
inline const auto filename = "flash_data.binary";

bool mount_card(FATFS* fs)
{
  const FRESULT fr = f_mount(fs, "", 1);
  if (fr != FR_OK)
  {
    usb_communication::send_string(std::format("Fault: microSD card failed to mount, error {} ({})", FRESULT_str(fr), static_cast<uint8_t>(fr)));
    set_fault(status_manager::MICRO_SD, true);
    return false;
  }

  set_fault(status_manager::MICRO_SD, false);
  return true;
}

bool open_file(FIL* fil)
{
  FATFS fs;
  if (!mount_card(&fs))
  {
    return false;
  }

  const FRESULT fr = f_open(fil, filename, FA_OPEN_ALWAYS | FA_WRITE);
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
  if (fr != FR_OK)
  {
    usb_communication::send_string(std::format(
      "Fault: could not close open file on microSD card, error: {} ({})",
      FRESULT_str(fr), static_cast<uint8_t>(fr)));
    set_fault(status_manager::MICRO_SD, true);
  }

  f_unmount("");
  return true;
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
  critical_section_init(&flash_sim_cs);

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
  usb_communication::send_string(std::format("Writing sector 0x{:06X}", sector_addr));

  critical_section_enter_blocking(&flash_sim_cs);
  FIL fil;
  FRESULT fr;
  if (!open_file(&fil))
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

      // usb_communication::send_string(std::format("Filling {} blank bytes", write_size));

      gpio_put(CORE_0_LED_PIN, true);

      fr = f_write(&fil, sector_buffer, write_size, &bytes_written);
      gpio_put(CORE_1_LED_PIN, false);

      if (fr != FR_OK || bytes_written != write_size)
      {
        usb_communication::send_string(std::format(
          "Fault: failed to write {} blank bytes (only wrote {} blank bytes) to microSD, error: {} ({}) [[{}]]",
          write_size, bytes_written,
          FRESULT_str(fr), static_cast<uint8_t>(fr), fil.fptr));

        set_fault(status_manager::MICRO_SD, true);
        try_close_and_unmount(&fil);
        critical_section_exit(&flash_sim_cs);
        return false;
      }

      blank_bytes_written += bytes_written;
    }
  }

  // usb_communication::send_string(std::format("Seeking to sector 0x{:06X}", sector_addr));
  if (!try_seek(&fil, sector_addr))
  {
    try_close_and_unmount(&fil);
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
    set_fault(status_manager::MICRO_SD, true);
    try_close_and_unmount(&fil);
    critical_section_exit(&flash_sim_cs);
    return false;
  }

  const bool unmount_success = try_close_and_unmount(&fil);
  critical_section_exit(&flash_sim_cs);
  return unmount_success;
}

bool w25q64fv::page_program(uint32_t page_addr, const uint8_t* data, size_t data_len)
{
  usb_communication::send_string("Do not call w25q64fv::page_program when using the microSD card as a flash simulator");
  return false;
}

bool w25q64fv::read_data(uint32_t data_addr, uint8_t* data_buff, size_t data_len)
{
  return false;
}

bool w25q64fv::chip_erase()
{
  FATFS fs;
  if (!mount_card(&fs))
  {
    return false;
  }

  FRESULT fr = f_stat(filename, nullptr);
  if (fr == FR_NO_FILE)
  {
    set_fault(status_manager::MICRO_SD, false);
    f_unmount("");
    return true;
  }

  if (fr != FR_OK)
  {
    usb_communication::send_string(std::format(
      "Fault: failed to determine if the flash simulator data file existed on the microSD card, error: {} ({})",
      FRESULT_str(fr), static_cast<uint8_t>(fr)));
    set_fault(status_manager::MICRO_SD, true);
    f_unmount("");
    return false;
  }

  fr = f_unlink(filename);
  f_unmount("");

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
