#pragma once

#include <pico/critical_section.h>
#include <pico/util/queue.h>

#define MAX_ENCODER_POS 171
#define MIN_ENCODER_POS 0
#define ZERO_BUTTON_THRESHOLD 15

#define MIN_BUTTON_PRESS_MS 250

#define MAX_ALARMS 10

namespace core1 {
    inline queue_t core0_ready_queue, core1_ready_queue;
    inline queue_t encoder_target_queue;

    inline alarm_pool_t* ab_ctrl_pool;

    inline critical_section_t target_access_cs, encoder_pos_cs;
    inline bool angle_override_active = false;
    inline int32_t current_encoder_pos = MAX_ENCODER_POS, target_encoder_pos;
    inline bool require_zero_threshold = true;

    void launch_core1();
    void core1_main();

    void handle_int(uint gpio, uint32_t events);
}
