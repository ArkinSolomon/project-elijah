#include "speaker_controller.h"

#include <format>

#include "usb_comm.h"
#include "reliable_sensors/reliable_clock/reliable_clock.h"

void elijah_state_framework::speaker_controller::init(uint8_t speaker_gpio)
{
  gpio_init(speaker_gpio);
  gpio_set_function(speaker_gpio, GPIO_FUNC_PWM);
  internal::status_pwm_slice = pwm_gpio_to_slice_num(speaker_gpio);
  internal::status_pwm_chan = speaker_gpio & 0x01 > 0 ? PWM_CHAN_B : PWM_CHAN_A;

  pwm_clear_irq(internal::status_pwm_slice);
  pwm_set_irq_enabled(internal::status_pwm_slice, true);
  irq_set_exclusive_handler(PWM_DEFAULT_IRQ_NUM(), &internal::on_pwm_wrap);
  irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true);

  internal::play_status_freqs(nullptr, 0);
}

void elijah_state_framework::speaker_controller::internal::play_status_freqs(const uint16_t* freqs, const size_t freq_len)
{
  bool has_changed = freq_list != freqs;

  freq_list = freqs;
  curr_freq_idx = 0;
  freq_list_len = freq_len;

  if (freq_list == nullptr || freq_len == 0)
  {
    gpio_put(25, true);
    // pwm_set_enabled(status_pwm_slice, false);
    return;
  }
  //
  // if (!has_changed)
  // {
  //   return;
  // }

  // pwm_set_enabled(status_pwm_slice, false);
  play_current_status_freq();
  pwm_set_enabled(status_pwm_slice, true);
}

float divider;
uint32_t top;
void elijah_state_framework::speaker_controller::internal::play_current_status_freq()
{
  const uint16_t target_freq = freq_list[curr_freq_idx];

  const uint32_t f_sys = clock_get_hz(clk_sys);
  /*const float*/ divider = static_cast<float>(f_sys) / SPEAKER_PWM_FREQ_HZ;
  pwm_set_clkdiv(status_pwm_slice, divider);
  /*const uint32_t*/ top = SPEAKER_PWM_FREQ_HZ / target_freq - 1;
  pwm_set_wrap(status_pwm_slice, top);
  pwm_set_chan_level(status_pwm_slice, status_pwm_chan, top / 2);

  curr_wrap_cycles = 0;
  const float ms_per_wrap = static_cast<float>(top) / SPEAKER_PWM_FREQ_HZ * 1000;
  wrap_cycles_req = SPEAKER_SWITCH_LEN_MS / ms_per_wrap;

  log_serial_message(std::format("Updating PWM frequency to {}, top: {}, level: {}, div: {}, {}, {}ms",
                                 target_freq, top, top / 2, divider, wrap_cycles_req, ms_per_wrap));
}

void elijah_state_framework::speaker_controller::internal::on_pwm_wrap()
{
  pwm_clear_irq(status_pwm_slice);
  curr_wrap_cycles++;
  if (curr_wrap_cycles >= wrap_cycles_req)
  {
    log_serial_message(std::format("wrap {} at freq {} at top {} div {}, {}/{}", to_us_since_boot(get_absolute_time()), freq_list[curr_freq_idx],top, divider, curr_wrap_cycles, wrap_cycles_req));
    curr_wrap_cycles = 0;
    curr_freq_idx++;
    if (curr_freq_idx == freq_list_len)
    {
      curr_freq_idx = 0;
    }
    play_current_status_freq();
  }
}
