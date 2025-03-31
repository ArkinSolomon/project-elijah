#include "reliable_clock.h"

#include <pico/aon_timer.h>

#include "payload_state_manager.h"

#define DS_1307_ADDR 0b1101000

ReliableClock::ReliableClock() : ReliableComponentHelper(payload_state_manager, PayloadFaultKey::OnboardClock),
                                 ds_1307(DS1307(i2c0, DS_1307_ADDR))
{
}

DS1307& ReliableClock::get_ds_1307()
{
  return ds_1307;
}

void ReliableClock::set_clock(const tm& time_inst)
{
  const bool clock_set = ds_1307.set_clock(time_inst);
  if (!clock_set)
  {
    get_framework()->set_fault(PayloadFaultKey::DS1307, true,
                           "Failed to read DS1307 (or not set) while initializing onboard clock");
    return;
  }

  if (is_connected())
  {
    aon_timer_set_time_calendar(&time_inst);
  }
}

std::string ReliableClock::on_init(PayloadState& state)
{
  tm time_inst{};
  if (!ds_1307.check_and_read_clock(time_inst))
  {
    return "Failed to initialize onboard clock with DS1307";
  }

  if (aon_timer_is_running())
  {
    aon_timer_set_time_calendar(&time_inst);
  }
  else
  {
    aon_timer_start_calendar(&time_inst);
  }

  busy_wait_ms(1);

  if (!aon_timer_is_running())
  {
    return "Failed to initialize onboard clock";
  }
  return "";
}

std::string ReliableClock::on_update(PayloadState& state)
{
  const bool got_time = aon_timer_get_time_calendar(&state.time_inst);

  if (!got_time)
  {
    return "Failed to get time from onboard clock";
  }

  if (cycles_since_last_check >= DS_1307_FUNCTIONAL_CHECK_CYCLES)
  {
    if (ds_1307.functional_check(state.time_inst))
    {
      cycles_since_last_check = 0;
    }
  }
  else
  {
    cycles_since_last_check++;
  }
  return "";
}
