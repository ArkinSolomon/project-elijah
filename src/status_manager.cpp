#include "status_manager.h"

#include <cstdio>
#include <iostream>
#include <hardware/clocks.h>
#include <hardware/pio.h>

#include "pin_outs.h"
#include "status_led_controller.pio.h"

void status_manager::status_manager_pio_init()
{
  static constexpr float pio_freq = 2000;
  const float div = static_cast<float>(clock_get_hz(clk_sys)) / pio_freq;

  const bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&status_led_controller_program, &pio, &sm,
                                                                        &offset, STATUS_LED_PIN, 1, true);
  if (!success)
  {
    uint8_t err_pattern_count = 0;
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, true);
    while (err_pattern_count <= 10)
    {
      printf("Fault: PIO not started [NO_FREE]");
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

  status_led_controller_program_init(pio, sm, offset, STATUS_LED_PIN, div);
  set_status(BOOTING);
}

void status_manager::set_status(device_status status)
{
  if (current_status == status || current_status == DONE)
  {
    return;
  }

  if (current_status == FAULT || current_status == NORMAL)
  {
    status = check_faults();
  }

  printf("Device status: %#010X\n", status);
  current_status = status;
  pio_sm_put_blocking(pio, sm, status);
}

status_manager::device_status status_manager::get_current_status()
{
  return current_status;
}

void status_manager::set_fault(const fault_id fault_id, const bool fault_state)
{
  const bool current_fault = faults[fault_id];
  if (current_fault == fault_state)
  {
    return;
  }

  faults[fault_id] = fault_state;
  if (current_status == FAULT || current_status == NORMAL)
  {
    set_status(check_faults());
  }
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