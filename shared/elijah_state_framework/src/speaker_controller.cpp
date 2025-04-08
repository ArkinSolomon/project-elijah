#include "speaker_controller.h"

#include <format>

#include <hardware/gpio.h>
#include <hardware/clocks.h>
#include <hardware/pwm.h>

#include "usb_comm.h"

void elijah_state_framework::speaker_controller::init(uint8_t speaker_gpio)
{
  gpio_init(speaker_gpio);
  gpio_set_function(speaker_gpio, GPIO_FUNC_PWM);
  internal::status_pwm_slice = pwm_gpio_to_slice_num(speaker_gpio);
  internal::status_pwm_chan = speaker_gpio & 0x01 > 0 ? PWM_CHAN_B : PWM_CHAN_A;

  critical_section_init(&internal::speaker_cs);
  internal::alarm_pool = alarm_pool_create_with_unused_hardware_alarm(MAX_AIRBRAKES_CONTROL_TIMERS);

  internal::play_status_freqs(nullptr, nullptr, 0);
}

void elijah_state_framework::speaker_controller::mute()
{
  critical_section_enter_blocking(&internal::speaker_cs);
  internal::is_audio_muted = true;
  pwm_set_chan_level(internal::status_pwm_slice, internal::status_pwm_chan, 0);
  critical_section_exit(&internal::speaker_cs);
}

void elijah_state_framework::speaker_controller::unmute()
{
  critical_section_enter_blocking(&internal::speaker_cs);
  internal::is_audio_muted = false;
  pwm_set_chan_level(internal::status_pwm_slice, internal::status_pwm_chan, internal::duty_cycle_top);
  critical_section_exit(&internal::speaker_cs);
}

bool elijah_state_framework::speaker_controller::toggle_speaker()
{
  critical_section_enter_blocking(&internal::speaker_cs);
  if (internal::is_audio_muted)
  {
    internal::is_audio_muted = false;
    pwm_set_chan_level(internal::status_pwm_slice, internal::status_pwm_chan, internal::duty_cycle_top);
  }
  else
  {
    internal::is_audio_muted = true;
    pwm_set_chan_level(internal::status_pwm_slice, internal::status_pwm_chan, 0);
  }
  const bool is_muted = internal::is_audio_muted;
  critical_section_exit(&internal::speaker_cs);
  return is_muted;
}

bool elijah_state_framework::speaker_controller::is_muted()
{
  critical_section_enter_blocking(&internal::speaker_cs);
  const bool is_muted = internal::is_audio_muted;
  critical_section_exit(&internal::speaker_cs);
  return is_muted;
}

void elijah_state_framework::speaker_controller::internal::play_status_freqs(
  const uint16_t* freqs, const uint16_t* timings_ms, const size_t freq_len)
{
  critical_section_enter_blocking(&speaker_cs);

  if (freqs == nullptr || timings_ms == nullptr || freq_len == 0)
  {
    pwm_set_enabled(status_pwm_slice, false);
    cancel_current_alarm();
    critical_section_exit(&speaker_cs);
    return;
  }

  const bool has_changed = freq_list != freqs;
  if (!has_changed)
  {
    critical_section_exit(&speaker_cs);
    return;
  }

  curr_freq_idx = 0;
  freq_list = freqs;
  timings_ms_list = timings_ms;
  freq_list_len = freq_len;
  duty_cycle_top = 0;

  cancel_current_alarm();

  const uint64_t next_freq_time_us = play_current_freq();
  pwm_set_enabled(status_pwm_slice, true);
  next_freq_alarm = alarm_pool_add_alarm_in_us(alarm_pool, next_freq_time_us, &on_next_freq, nullptr, false);
  if (next_freq_alarm < 0)
  {
    pwm_set_enabled(status_pwm_slice, false);
  }
  critical_section_exit(&speaker_cs);
}

uint64_t elijah_state_framework::speaker_controller::internal::play_current_freq()
{
  const uint16_t target_freq = freq_list[curr_freq_idx];
  const uint16_t required_timing_ms = timings_ms_list[curr_freq_idx];

  const uint32_t f_sys = clock_get_hz(clk_sys);
  const float divider = static_cast<float>(f_sys) / SPEAKER_PWM_FREQ_HZ;
  pwm_set_clkdiv(status_pwm_slice, divider);
  const uint32_t top = SPEAKER_PWM_FREQ_HZ / target_freq - 1;
  pwm_set_wrap(status_pwm_slice, top);

#ifdef SPEAKER_ENABLE
  duty_cycle_top = !is_audio_muted && target_freq ? top / 2 : 0;
#else
  constexpr uint32_t duty_cycle_top = 0;
#endif
  pwm_set_chan_level(status_pwm_slice, status_pwm_chan, duty_cycle_top);

  return required_timing_ms * 1000;
}

int64_t elijah_state_framework::speaker_controller::internal::on_next_freq(alarm_id_t, void*)
{
  critical_section_enter_blocking(&speaker_cs);
  curr_freq_idx++;
  if (curr_freq_idx == freq_list_len)
  {
    curr_freq_idx = 0;
  }

  const auto next_freq_time = static_cast<int64_t>(play_current_freq());
  critical_section_exit(&speaker_cs);
  return next_freq_time;
}

void elijah_state_framework::speaker_controller::internal::cancel_current_alarm()
{
  if (next_freq_alarm <= 0)
  {
    return;
  }

  if (alarm_pool_cancel_alarm(alarm_pool, next_freq_alarm))
  {
    next_freq_alarm = std::numeric_limits<int32_t>::min();
  }
}
