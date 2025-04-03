#pragma once
#include <hardware/pwm.h>

// #define SPEAKER_PWM_FREQ_HZ 65535
#define SPEAKER_PWM_FREQ_HZ 1000000ul
#define SPEAKER_SWITCH_LEN_MS 1000

namespace elijah_state_framework::speaker_controller
{
  void init(uint8_t speaker_gpio);

  namespace internal
  {
    inline uint status_pwm_slice, status_pwm_chan;

    inline const uint16_t* freq_list = nullptr;
    inline size_t curr_freq_idx = 0;
    inline size_t freq_list_len = 0;

    inline uint wrap_cycles_req;
    inline uint curr_wrap_cycles;

    void play_status_freqs(const uint16_t* freqs, size_t freq_len);
    void play_current_status_freq();
    void on_pwm_wrap();
  }
}
