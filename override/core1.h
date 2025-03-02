#pragma once

#include <pico/util/queue.h>

namespace core1 {
    inline queue_t core0_ready_queue, core1_ready_queue;

    void launch_core1();
    void core1_main();
}
