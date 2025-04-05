#pragma once
#include <pico/critical_section.h>

#define SPEAKER_PWM_FREQ_HZ 1000000ul
#define MAX_ALARM_TIMERS 4

namespace elijah_state_framework::speaker_controller
{
  void init(uint8_t speaker_gpio);

  void mute();
  void unmute();
  bool toggle_speaker();
  [[nodiscard]] bool is_muted();

  namespace internal
  {
    inline critical_section_t speaker_cs;
    inline uint status_pwm_slice, status_pwm_chan;

    inline bool is_audio_muted = false;

    inline const uint16_t* freq_list = nullptr;
    inline const uint16_t* timings_ms_list = nullptr;
    inline size_t curr_freq_idx = 0;
    inline size_t freq_list_len = 0;

    inline uint32_t duty_cycle_top = 0;

    inline alarm_pool_t* alarm_pool = nullptr;
    inline alarm_id_t next_freq_alarm;

    void play_status_freqs(const uint16_t* freqs, const uint16_t* timings_ms, size_t freq_len);
    uint64_t play_current_freq();

    int64_t on_next_freq(alarm_id_t, void*);
    void cancel_current_alarm();
  }
}
