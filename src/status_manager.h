#pragma once
#include <hardware/pio.h>
#include <pico/stdlib.h>

#define END_OF_FAULT_LIST 0xAB
#define I2C_MAX_CONSISTENT_FAILS 20

namespace status_manager
{
  inline PIO pio;
  inline uint sm;
  inline uint offset;

  inline uint8_t i2c_fault_detect_cycles = 0;

  inline uint8_t faults[6] = {false, false, false, false, false, END_OF_FAULT_LIST};

  enum fault_id
  {
    DEVICE_BMP_180 = 0,
    DEVICE_DS_1307 = 1,
    _i2c_bus = 2,
    DEVICE_BMP_280 = 3,
    DEVICE_HMC5883L = 4,
    _end_of_device_list = END_OF_FAULT_LIST
  };

  constexpr fault_id i2c_fault_ids[4] = {/*DEVICE_BMP_180,*/ DEVICE_DS_1307, DEVICE_BMP_280, DEVICE_HMC5883L, _end_of_device_list};

  enum device_status
  {
    STATUS_NOT_SET = 0,
    BOOTING = 0xFFFF0000,
    NORMAL = 0xFFFFFFFF,
    FAULT = 0xAAAAAAAA,
    DONE = 0xF0000A00,
    USB = 0xFFFFFFB
  };

  inline device_status current_status = STATUS_NOT_SET;

  void status_manager_pio_init();
  void set_status(device_status status);
  device_status get_current_status();
  void set_fault(fault_id fault_id, bool fault_state);
  device_status check_faults();
  bool is_i2c_fault_id(fault_id id);
  bool check_i2c_bus_fault();
}
