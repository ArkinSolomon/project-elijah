#include "status_manager.h"

#include <cstdio>
#include <format>
#include <hardware/clocks.h>
#include <hardware/pio.h>

#include "byte_util.h"
#include "lock_nums.h"
#include "pin_outs.h"
#include "elijah_state_framework.h"
#include "payload_state_manager.h"
#include "i2c_util.h"

#ifdef STATUS_MANAGER_LED_ENABLE
#include "status_led_controller.pio.h"
#endif

void status_manager::status_manager_init()
{
  if (!critical_section_is_initialized(&status_cs))
  {
    critical_section_init_with_lock_num(&status_cs, LOCK_NUM_STATUS_CS);
  }

#ifdef STATUS_MANAGER_LED_ENABLE
  // This freq math is technically wrong, but it's a good speed so who cares ig
  constexpr float pio_freq = 10;
  uint mem_offset;
  const float div = static_cast<float>(clock_get_hz(clk_sys)) / pio_freq;
  const bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&status_led_controller_program, &sm_pio, &sm_state_machine,
                                                                        &mem_offset, STATUS_LED_PIN, 1, true);

  if (!success)
  {
    uint8_t err_pattern_count = 0;
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, true);
    while (err_pattern_count <= 10)
    {
      PayloadStateManager::log_message("Fault: status manager PIO not started, no free state machines");
      gpio_put(STATUS_LED_PIN, true);
      sleep_ms(500);
      gpio_put(STATUS_LED_PIN, false);
      sleep_ms(200);
      gpio_put(STATUS_LED_PIN, true);
      sleep_ms(50);
      gpio_put(STATUS_LED_PIN, false);
      sleep_ms(250);
      err_pattern_count++;
    }

    gpio_deinit(STATUS_LED_PIN);
    return;
  }

  status_led_controller_program_init(sm_pio, sm_state_machine, mem_offset, STATUS_LED_PIN, div);
#endif
  set_status(BOOTING);
}

void status_manager::set_status(const device_status status)
{
  if (current_status == status)
  {
    return;
  }

  PayloadStateManager::log_message(std::format("Device status changed: 0x{:08X} ({})", static_cast<uint32_t>(status),
                                             status_name_map.contains(status)
                                               ? status_name_map.at(status)
                                               : "Unknown status")
  );
  current_status = status;
#ifdef STATUS_MANAGER_LED_ENABLE
  pio_sm_put_blocking(sm_pio, sm_state_machine, status);
#endif
}

status_manager::device_status status_manager::get_current_status()
{
  return current_status;
}

void status_manager::set_fault(const fault_id fault_id, const bool fault_state)
{
  critical_section_enter_blocking(&status_cs);

  const bool original_fault_state =  faults[fault_id] ;
  faults[fault_id] = fault_state;
  const bool did_bus_fault = detect_i2c_bus_fault(fault_id);

  if (original_fault_state != fault_state || did_bus_fault)
  {
    set_status(check_faults());
    send_status();
  }

  critical_section_exit(&status_cs);
}

bool status_manager::is_faulted(const fault_id fault_id)
{
  return faults[fault_id];
}

bool status_manager::detect_i2c_bus_fault(const fault_id fault_id)
{
  static uint8_t fault_detect_cycles_bus0 = 0;
  static uint8_t fault_detect_cycles_bus1 = 0;

  const bool is_faulted = faults[fault_id];
  auto fault_on_bus = i2c_bus::NONE;
  if (is_i2c_fault_id(i2c_fault_ids_bus0, fault_id))
  {
    fault_on_bus = i2c_bus::BUS0;
  }
  else if (is_i2c_fault_id(i2c_fault_ids_bus1, fault_id))
  {
    fault_on_bus = i2c_bus::BUS1;
  }

  uint8_t* fault_detect_cycles = fault_on_bus == i2c_bus::BUS0
                                   ? &fault_detect_cycles_bus0
                                   : &fault_detect_cycles_bus1;

  if (fault_on_bus == i2c_bus::NONE)
  {
    return false;
  }

  if (!is_faulted)
  {
    *fault_detect_cycles = 0;
    faults[static_cast<uint8_t>(fault_on_bus)] = false;
    return false;
  }

  (*fault_detect_cycles)++;


  if (*fault_detect_cycles < I2C_MAX_FAULT_CYCLES)
  {
    return false;
  }
  *fault_detect_cycles = I2C_MAX_FAULT_CYCLES;

  if (check_i2c_bus_fault(fault_on_bus == i2c_bus::BUS0 ? i2c_fault_ids_bus0 : i2c_fault_ids_bus1))
  {
    faults[static_cast<uint8_t>(fault_on_bus)] = true;
    set_status(FAULT);

    PayloadStateManager::log_message(std::format("Fault: I2C Bus fail, bus {}... attempting to recover", fault_on_bus == i2c_bus::BUS0 ? 0 : 1));
    if (fault_on_bus == i2c_bus::BUS0)
    {
      i2c_util::recover_i2c(I2C_BUS0, I2C0_SDA_PIN, I2C0_SCL_PIN);
    }
    else
    {
      i2c_util::recover_i2c(I2C_BUS1, I2C1_SDA_PIN, I2C1_SCL_PIN);
    }
    busy_wait_ms(75);
    return true;
  }
  return false;
}

status_manager::device_status status_manager::check_faults()
{
  size_t i = 0;
  device_status new_state = NORMAL;
  do
  {
    if (faults[i])
    {
      new_state = FAULT;
      break;
    }
  }
  while (faults[++i] != END_OF_FAULT_LIST);
  return new_state;
}

bool status_manager::is_i2c_fault_id(const fault_id i2c_fault_ids[], const fault_id id)
{
  size_t i = 0;
  do
  {
    if (i2c_fault_ids[i] == id)
    {
      return true;
    }
  }
  while (i2c_fault_ids[++i] != _end_of_device_list);
  return false;
}


bool status_manager::check_i2c_bus_fault(const fault_id i2c_fault_ids[])
{
  size_t i = 0;

  do
  {
    if (!faults[i2c_fault_ids[i]])
    {
      return false;
    }
  }
  while (i2c_fault_ids[++i] != _end_of_device_list);
  return true;
}

void status_manager::send_status()
{
  if (!stdio_usb_connected())
  {
    return;
  }

  uint8_t send_data[6];
  size_t i = 0;
  size_t byte_offset = 0;
  do
  {
    if (i > 0 && i % 8 == 0)
    {
      byte_offset += 1;
    }
    send_data[4 + byte_offset] <<= 1;
    send_data[4 + byte_offset] |= faults[i] ? 0x1 : 0x0;
  }
  while (faults[++i] != END_OF_FAULT_LIST);

  byte_util::encode_uint32(current_status, send_data);
  // send_packet(usb_communication::FAULT_DATA, send_data);
}
