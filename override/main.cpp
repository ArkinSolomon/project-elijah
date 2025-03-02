#include <format>
#include <hardware/gpio.h>
#include <pico/flash.h>
#include <pico/multicore.h>

#include "bmp_280.h"
#include "core1.h"
#include "i2c_util.h"
#include "mpu_6050.h"
#include "override_state_manager.h"
#include "pin_outs.h"
#include "sensors.h"

int main()
{
  flash_safe_execute_core_init();
  multicore_lockout_victim_init();

  core1::launch_core1();

  uint8_t core_data;
  queue_remove_blocking(&core1::core1_ready_queue, &core_data);
  override_state_manager = new OverrideStateManager();
  core_data = 0xBB;
  queue_add_blocking(&core1::core0_ready_queue, &core_data);

  StateFrameworkLogger::init_driver_on_core();

  sensors_init();

  OverrideState state{};

  while (true)
  {
    auto s = bmp280->change_settings(BMP280::DeviceMode::NormalMode, BMP280::StandbyTimeSetting::Standby500us,
                         BMP280::FilterCoefficientSetting::FilterOff, BMP280::OssSettingPressure::PressureOss2,
                         BMP280::OssSettingTemperature::TemperatureOss1);

    override_state_manager->check_for_commands();
    const bool mpus = mpu6050->get_data(state.accel_x, state.accel_y, state.accel_z, state.gyro_x, state.gyro_y, state.gyro_z);
    const bool bmps = bmp280->read_press_temp_alt(state.pressure, state.temperature, state.altitude);

    state.bat_voltage = battery->get_voltage();
    state.bat_percent = battery->calc_charge_percent(state.bat_voltage);

    OverrideStateManager::log_message(std::format("[{}] p: {} t: {} alt: {} a: {} {} {}", s,state.pressure, state.temperature, state.altitude, state.accel_x, state.accel_y, state.accel_z));
    override_state_manager->state_changed(state);

    gpio_put(LED_3_PIN, true);
    override_state_manager->lock_logger();
    override_state_manager->get_logger()->write_full_buff();
    override_state_manager->release_logger();
    gpio_put(LED_3_PIN, false);

    sleep_ms(100);
  }
}
