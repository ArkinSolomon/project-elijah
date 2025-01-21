#pragma once

#define CORE_1_READY_FLAG 0xC7

#include "storage/payload_data_manager/payload_data_manager.h"

namespace core_1_stats
{
  inline mutex_t loop_time_mtx;
  inline uint64_t loop_time;
}

void launch_core_1();
void core_1_main();
