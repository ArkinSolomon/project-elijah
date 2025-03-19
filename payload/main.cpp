
#include <format>
#include <sd_card.h>

#include <pico/flash.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <pico/util/queue.h>

#include "aprs.h"
#include "core1.h"
#include "pin_outs.h"
#include "payload_state_manager.h"
#include "elijah_state_framework.h"
#include "sensors/onboard_clock/onboard_clock.h"

#define APRS_FLAG 0x7E

int main()
{
  flash_safe_execute_core_init();
  multicore_lockout_victim_init();

  pin_init();

  elijah_state_framework::StateFrameworkLogger::init_driver_on_core();

  core1::launch_core1();

  uint8_t core_data;
  queue_remove_blocking(&core1::core1_ready_queue, &core_data);
  payload_state_manager = new PayloadStateManager();

  core_data = 0xBB;
  queue_add_blocking(&core1::core0_ready_queue, &core_data);

  sensors_init();

  PayloadState state{};

  gpio_put(LED_3_PIN, true);
  while (true)
  {
    payload_state_manager->check_for_commands();

    bmp280->update(state);
    mpu6050->update(state);
    onboard_clock::clock_loop(state);

    state.bat_voltage = battery->get_voltage();
    state.bat_percent = battery->calc_charge_percent(state.bat_voltage);

    payload_state_manager->state_changed(state);

    payload_state_manager->check_for_log_write();

    sleep_ms(50);
  }

  // status_manager::status_manager_init();
  // mpu_6050::init();
  //
  // gpio_put(CORE_0_LED_PIN, true);
  // sleep_ms(200);
  // gpio_put(CORE_1_LED_PIN, true);
  // sleep_ms(200);
  //
  // if (watchdog_caused_reboot())
  // {
  //   PayloadStateManager::log_message("Reboot caused by watchdog");
  // }
  // watchdog_enable(5000, true);
  //
  // sleep_ms(1000);
  // gpio_put(CORE_0_LED_PIN, false);
  //
  // core_1::launch_core_1();
  //
  // while (multicore_fifo_get_status() & 0x1 == 0)
  // {
  //   if (multicore_fifo_pop_blocking() == CORE_1_READY_FLAG)
  //   {
  //     PayloadStateManager::log_message("Core 1 ready");
  //     break;
  //   }
  // }
  //
  // if (status_manager::get_current_status() == status_manager::BOOTING)
  // {
  //   set_status(status_manager::NORMAL);
  // }
  //
  // constexpr uint64_t us_between_loops = 1000000 / MAX_UPDATES_PER_SECOND;
  //
  // bool led_on = false;
  // bool usb_connected = false;
  //
  // CollectionData collection_data{{}};
  // absolute_time_t last_loop_end_time = nil_time;
  // while (true)
  // {
  //   const absolute_time_t start_time = get_absolute_time();
  //
  //   watchdog_update();
  //   gpio_put(CORE_0_LED_PIN, led_on = !led_on);
  //
  //   const absolute_time_t main_loop_time = absolute_time_diff_us(start_time, get_absolute_time());
  //   if (stdio_usb_connected())
  //   {
  //     const absolute_time_t usb_start_time = get_absolute_time();
  //     if (!usb_connected)
  //     {
  //       // usb_communication::say_hello();
  //       bmp_280::send_calibration_data();
  //       mpu_6050::send_calibration_data();
  //       status_manager::send_status();
  //       payload_data_manager::send_current_launch_data();
  //       usb_connected = true;
  //     }
  //
  //     status_manager::check_faults();
  //
  //     state_com.check_for_commands();
  //     // usb_communication::send_collection_data(collection_data);
  //
  //     // uint8_t loop_time_data[usb_communication::packet_type_lens.at(usb_communication::LOOP_TIME)];
  //     // byte_util::encode_uint64(main_loop_time, loop_time_data);
  //
  //     mutex_enter_blocking(&core_1::stats::loop_time_mtx);
  //     // byte_util::encode_uint64(core_1::stats::loop_time, loop_time_data + 8);
  //     mutex_exit(&core_1::stats::loop_time_mtx);
  //
  //     const absolute_time_t usb_loop_time = absolute_time_diff_us(usb_start_time, get_absolute_time());
  //     // byte_util::encode_uint64(usb_loop_time, loop_time_data + 16);
  //
  //     // send_packet(usb_communication::LOOP_TIME, loop_time_data);
  //   }
  //   else if (usb_connected)
  //   {
  //     usb_connected = false;
  //   }
  //
  //   sleep_until(delayed_by_us(last_loop_end_time, us_between_loops));
  //   last_loop_end_time = get_absolute_time();
  // }
  //
  // watchdog_disable();
}
