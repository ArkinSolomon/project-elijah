#pragma once
#include <hardware/pio.h>
#include <pico/stdlib.h>

namespace status_manager
{
  inline PIO pio;
  inline uint sm;
  inline uint offset;

  constexpr uint8_t END_OF_FAULT_LIST = 0xAB;
  inline uint8_t faults[3] = {false, false, END_OF_FAULT_LIST};

  enum fault_id
  {
    DEVICE_BMP_180 = 0,
    DEVICE_DS_1307 = 1
  };

  enum device_status
  {
    STATUS_NOT_SET = 0,
    BOOTING = 0xFFFF0000,
    NORMAL = 0xFFFFFFFF,
    FAULT = 0xAAAAAAAA,
    DONE = 0xF0000A00
  };

  inline device_status current_status = STATUS_NOT_SET;

  void status_manager_pio_init();
  void set_status(device_status status);
  device_status get_current_status();
  void set_fault(fault_id fault_id, bool fault_state);
  device_status check_faults();
}
