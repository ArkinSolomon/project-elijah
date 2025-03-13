#include "onboard_clock.h"

#include <cstdint>
#include <pico/aon_timer.h>

#include "payload_state_manager.h"
#include "sensors/ds_1307/ds_1307.h"

void onboard_clock::clock_loop(PayloadState& collection_data)
{
  static uint8_t loops_since_ds_1307_check = 0;

  if (!aon_timer_is_running())
  {
    if (ds_1307::check_and_read_clock(collection_data.time_inst))
    {
      if (aon_timer_start_calendar(&collection_data.time_inst))
      {
        payload_state_manager->set_fault(FaultKey::OnboardClock, false);
      }
      else
      {
        payload_state_manager->set_fault(FaultKey::OnboardClock, false, "Onboard clock not running");
      }
    }
    else
    {
      payload_state_manager->set_fault(FaultKey::OnboardClock, true, "Clock requires DS 1307 to init");
    }
    return;
  }

  const bool got_time = aon_timer_get_time_calendar(&collection_data.time_inst);
  // PayloadStateManager::log_message(std::format("Onboard RTC: {} {} {} {} {} {} {}", collection_data.time_inst.tm_year,
  //                                            collection_data.time_inst.tm_mon, collection_data.time_inst.tm_mday,
  //                                            collection_data.time_inst.tm_wday, collection_data.time_inst.tm_hour,
  //                                            collection_data.time_inst.tm_min, collection_data.time_inst.tm_sec));

  if (!got_time)
  {
    payload_state_manager->set_fault(FaultKey::OnboardClock, true, "Failed to get time from onboard clock");
    return;
  }

  if (loops_since_ds_1307_check > 32)
  {
    if (ds_1307::functional_check(collection_data.time_inst))
    {
      loops_since_ds_1307_check = 0;
    }
  }
  else
  {
    loops_since_ds_1307_check++;
  }

  payload_state_manager->set_fault(FaultKey::OnboardClock, false);
}
