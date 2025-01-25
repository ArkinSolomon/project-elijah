#pragma once

#define CORE_1_READY_FLAG 0xC7

#define MAX_CORE_1_CMD_Q_LEN 32
#define NEW_LAUNCH_CMD 0xC4CB8D0335ED5B2D
#define FLUSH_DATA_COMMAND 0x9BE29325B55701F3

#include <pico/util/queue.h>

#include "storage/payload_data_manager/payload_data_manager.h"

namespace core_1
{
  namespace stats
  {
    inline mutex_t loop_time_mtx;
    inline uint64_t loop_time;
  }

  inline queue_t command_queue;

  void launch_core_1();
  void core_1_main();

  void handle_new_launch_cmd();
  void handle_flush_data_cmd();
}
