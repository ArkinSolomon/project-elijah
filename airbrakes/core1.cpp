#include "core1.h"

#include <pico/flash.h>
#include <pico/multicore.h>

#include "airbrakes_state_manager.h"
#include "airbrake_controls.h"
#include "pin_outs.h"
#include "airbrakes_sensors.h"
#include "state_framework_logger.h"

void core1::launch_core1()
{
  multicore_reset_core1();

  queue_init(&core0_ready_queue, 1, 1);
  queue_init(&core1_ready_queue, 1, 1);
  queue_init(&encoder_target_queue, sizeof(int32_t), 16);

  critical_section_init(&target_access_cs);
  critical_section_init(&encoder_pos_cs);

  if (gpio_get(ZERO_BUTTON_PIN))
  {
    current_encoder_pos = 0;
  }

  sleep_ms(100);
  multicore_launch_core1(core1_main);
}

void core1::core1_main()
{
  flash_safe_execute_core_init();
  multicore_lockout_victim_init();

  elijah_state_framework::StateFrameworkLogger::init_driver_on_core();
  ab_ctrl_pool = alarm_pool_create_with_unused_hardware_alarm(MAX_AIRBRAKES_CONTROL_TIMERS);

  uint8_t core_ready = 0xAA;
  queue_add_blocking(&core1_ready_queue, &core_ready);
  queue_remove_blocking(&core0_ready_queue, &core_ready);

  gpio_set_irq_enabled_with_callback(ROTARY_CLK_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &handle_int);
  gpio_set_irq_enabled(ZERO_BUTTON_PIN, GPIO_IRQ_EDGE_RISE, true);

  gpio_put(LED_3_PIN, true);
  bool led_on = true;

  while (true)
  {
    critical_section_enter_blocking(&target_access_cs);
    if (!angle_override_active && airbrakes_state_manager->get_current_flight_phase() ==
      elijah_state_framework::std_helpers::StandardFlightPhase::COAST)
    {
      if (!queue_is_empty(&encoder_target_queue))
      {
        do
        {
          queue_remove_blocking(&encoder_target_queue, &target_encoder_pos);
        }
        while (!queue_is_empty(&encoder_target_queue));
      }
    }
    else if (!angle_override_active)
    {
      target_encoder_pos = 0;
    }

    target_encoder_pos = std::clamp(target_encoder_pos,
                                    static_cast<int32_t>(MIN_ENCODER_POS), static_cast<int32_t>(MAX_ENCODER_POS));
    const int32_t static_targ_pos = target_encoder_pos;
    critical_section_exit(&target_access_cs);

    // airbrakes_state_manager->log_message(std::format("target: {}, {}", static_targ_pos, angle_override_active));

    critical_section_enter_blocking(&encoder_pos_cs);
    const int32_t static_encoder_pos = current_encoder_pos;
    critical_section_exit(&encoder_pos_cs);

    if (abs(static_encoder_pos - static_targ_pos) <= 2)
    {
      airbrakes_freeze();
    }
    else if (static_encoder_pos < static_targ_pos)
    {
      // airbrakes_state_manager->log_message("Opening", elijah_state_framework::LogLevel::Debug);
      airbrakes_open();
    }
    else
    {
      // airbrakes_state_manager->log_message("Closing", elijah_state_framework::LogLevel::Debug);
      airbrakes_close();
    }

    gpio_put(LED_3_PIN, led_on = !led_on);
    sleep_ms(10);
  }
}

// See https://docs.cirkitdesigner.com/component/fb77c67b-258e-42e8-840b-9cf445a0d2f2/hw-040-rotary-encoder
void core1::handle_int(const uint gpio, const uint32_t events)
{
  critical_section_enter_blocking(&encoder_pos_cs);
  if (gpio == ROTARY_CLK_PIN)
  {
    static bool last_clk = gpio_get(ROTARY_CLK_PIN);
    const bool current_clk = gpio_get(ROTARY_CLK_PIN);

    if (last_clk != current_clk)
    {
      if (gpio_get(ROTARY_DT_PIN) != current_clk)
      {
        current_encoder_pos--;
      }
      else
      {
        current_encoder_pos++;
      }
    }
    last_clk = current_clk;
  }
  else if (gpio == ZERO_BUTTON_PIN)
  {
    if (gpio_get(ZERO_BUTTON_PIN))
    {
      current_encoder_pos = 0;
    }
  }

  critical_section_exit(&encoder_pos_cs);
}