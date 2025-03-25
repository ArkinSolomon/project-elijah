#pragma once

#include <pico/critical_section.h>
#include <pico/util/queue.h>

#define MAX_ENCODER_POS 171
#define MIN_ENCODER_POS 0

namespace core1 {
    inline queue_t core0_ready_queue, core1_ready_queue;
    inline queue_t encoder_target_queue;

    inline critical_section_t target_access_cs, encoder_pos_cs;
    inline bool angle_override_active = false;
    inline int32_t current_encoder_pos, target_encoder_pos;

    void launch_core1();
    void core1_main();

    void read_encoder();
    void encoder_zero(uint gpio, uint32_t events);
}
