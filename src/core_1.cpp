#include <sd_card.h>
#include <pico/flash.h>
#include <pico/multicore.h>

#include "core_1.h"

#include "lock_nums.h"
#include "pin_outs.h"
#include "status_manager.h"
#include "usb_communication.h"
#include "storage/w25q64fv/w25q64fv.h"

void core_1::launch_core_1()
{
  mutex_init(&stats::loop_time_mtx);
  payload_data_manager::init_data_manager();

  queue_init_with_spinlock(&command_queue, sizeof(uint64_t), MAX_CORE_1_CMD_Q_LEN, CORE_1_CMD_Q_SL_NUM);

  multicore_reset_core1();
  sleep_ms(100);
  multicore_launch_core1(core_1_main);
}

inline bool tmp_stop_data_writing = false;
void core_1::core_1_main()
{
  flash_safe_execute_core_init();

  bool did_w25q64fv_init = w25q64fv::init();
  if (!did_w25q64fv_init)
  {
    usb_communication::send_string("Fault: W25Q64FV, device failed to initialize");
    set_fault(status_manager::fault_id::DEVICE_W25Q64FV, true);
  }

  payload_data_manager::init_launch_data();

  // uint8_t test_data[32];
  // for (size_t i = 0; i < 32; i++)
  // {
  //   test_data[i] = get_rand_32() & 0xFF;
  // }
  //
  // w25q64fv::write_sector(0, test_data, 32);
  // uint8_t test_data_out[32];
  // w25q64fv::read_data(0, test_data_out, 32);
  // for (size_t i = 0; i < 32; i++)
  // {
  //   usb_communication::send_string(std::format("data {:02X} == {:02X}", test_data[i],
  //                                              test_data_out[i]));
  // }

  gpio_put(CORE_1_LED_PIN, false);
  multicore_fifo_push_blocking(CORE_1_READY_FLAG);

  bool led_on = false;
  while (true)
  {
    const absolute_time_t start_time = get_absolute_time();
    gpio_put(CORE_1_LED_PIN, led_on = !led_on);

    if (!queue_is_empty(&command_queue))
    {
      uint64_t command;
      queue_remove_blocking(&command_queue, &command);
      switch (command)
      {
      case NEW_LAUNCH_CMD:
        handle_new_launch_cmd();
        break;
      case FLUSH_DATA_COMMAND:
        handle_flush_data_cmd();
        break;
      default:
        usb_communication::send_string(std::format("Unhandled multicore command: 0x{:016X}", command));
        break;
      }
    }

    if (!did_w25q64fv_init)
    {
      did_w25q64fv_init = w25q64fv::init();
    }

    if (!tmp_stop_data_writing && did_w25q64fv_init)
    {
      payload_data_manager::try_write_active_data();
    }

    mutex_enter_blocking(&stats::loop_time_mtx);
    const uint64_t elapsed_time = absolute_time_diff_us(start_time, get_absolute_time());
    stats::loop_time = elapsed_time;
    mutex_exit(&stats::loop_time_mtx);
  }
}

void core_1::handle_new_launch_cmd()
{
  uint64_t name_addr_64;
  queue_remove_blocking(&command_queue, &name_addr_64);
  const auto name_ptr = reinterpret_cast<const char*>(name_addr_64);

  const std::string launch_name = name_ptr;
  payload_data_manager::new_launch(launch_name);
  delete [] name_ptr;
}

void core_1::handle_flush_data_cmd()
{
  tmp_stop_data_writing = !tmp_stop_data_writing;
  // usb_communication::send_string("Flush data not implemented");
}
