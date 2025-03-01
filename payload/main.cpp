#include "main.h"

#include <format>
#include <sd_card.h>

#include <hardware/adc.h>
#include <hardware/clocks.h>
#include <hardware/i2c.h>
#include <hardware/pwm.h>
#include <hardware/watchdog.h>
#include <pico/aon_timer.h>
#include <pico/flash.h>
#include <pico/multicore.h>
#include <pico/time.h>

#include "aprs.h"
#include "bmp_280.h"
#include "mpu_6050.h"
#include "byte_util.h"
#include "core_1.h"
#include "pin_outs.h"
#include "payload_state_manager.h"
#include "elijah_state_framework.h"
#include "status_manager.h"
#include "sensors/battery/battery.h"
#include "sensors/onboard_clock/onboard_clock.h"
#include "i2c_util.h"

#define APRS_FLAG 0x7E

int main()
{
#ifdef PICO_RP2040
#define CLOCK_SPEED_KHZ 133000
  set_sys_clock_khz(CLOCK_SPEED_KHZ, true);
#elifdef PICO_RP2350
#define CLOCK_SPEED_KHZ 150000
    set_sys_clock_khz(150000, true);
#endif

  // pin_init();
  flash_safe_execute_core_init();
  multicore_lockout_victim_init();

  PayloadStateManager::log_message("Initializing...");

  // i2c_util::i2c_bus_init(i2c0, I2C0_SDA_PIN, I2C0_SCL_PIN, 400 * 1000);
  // i2c_util::i2c_bus_init(i2c1, I2C1_SDA_PIN, I2C1_SCL_PIN, 400 * 1000);

  CollectionData data{};

  // BMP280 bmp280(i2c0, BMP_280_ADDR);
  // MPU6050 mpu6050(i2c1, 0x68, MPU6050::GyroFullScaleRange::Range250, MPU6050::AccelFullScaleRange::Range4g);
  // bmp280.read_calibration_data();
  // bmp280.change_settings(BMP280::DeviceMode::NormalMode, BMP280::StandbyTimeSetting::Standby500us,
  //                        BMP280::FilterCoefficientSetting::Filter4x, BMP280::OssSettingPressure::PressureOss2,
  //                        BMP280::OssSettingTemperature::TemperatureOss2);
  //
  // bmp280.get_calibration_data();
  // mpu6050.get_calibration_data();
  //
  // mpu6050.calibrate(50);
  // mpu6050.check_chip_id();
  while (true)
  {
    // bmp280.read_press_temp_alt(data.pressure, data.temperature, data.altitude);
    // mpu6050.get_data(data.accel_x, data.accel_y, data.accel_z, data.gyro_x, data.gyro_y, data.gyro_z);
    // payload_state_manager.data_collected(data);
    // PayloadStateManager::log_message(std::format("Pressure: {} Temperature: {:.03f} Alt: {:.03f}, xa = {}, ya ={}, za = {}, xg ={}, yg ={} , zg = {}", data.pressure, data.temperature, data.altitude, data.accel_x, data.accel_y, data.accel_z, data.gyro_x, data.gyro_y, data.gyro_z));
    // sleep_ms(10);
    payload_state_manager.send_framework_metadata();
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

void pin_init()
{
  i2c_util::i2c_bus_init(I2C_BUS0, I2C0_SDA_PIN, I2C0_SCL_PIN, 400 * 1000);
  i2c_util::i2c_bus_init(I2C_BUS1, I2C1_SDA_PIN, I2C1_SCL_PIN, 400 * 1000);

  gpio_init(CORE_0_LED_PIN);
  gpio_set_dir(CORE_0_LED_PIN, GPIO_OUT);
  gpio_init(CORE_1_LED_PIN);
  gpio_set_dir(CORE_1_LED_PIN, GPIO_OUT);

  // See sd_hw_config.c for microSD card SPI setup

  // SPI at 33MHz for W25Q64FV
  gpio_set_function(SPI1_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI1_TX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI1_RX_PIN, GPIO_FUNC_SPI);

  gpio_init(SPI1_CSN_PIN);
  gpio_set_dir(SPI1_CSN_PIN, GPIO_OUT);
  gpio_put(SPI1_CSN_PIN, true);

  spi_set_slave(spi1, false);
  spi_init(spi1, 33 * 1000 * 1000);

  // Battery ADC
  adc_init();
  adc_gpio_init(BAT_VOLTAGE_PIN);
  adc_select_input(BAT_ADC_INPUT);

  aprs::init_aprs_system(CLOCK_SPEED_KHZ, RADIO_SEL_PIN, PWM_CHAN_B);
  // gpio_init(RADIO_SEL_PIN);
  // gpio_set_dir(RADIO_SEL_PIN, GPIO_OUT);
}

void flight_loop(CollectionData& collection_data, absolute_time_t last_loop_start_time)
{
  onboard_clock::clock_loop(collection_data);
  // mpu_6050::data_loop(collection_data);
  battery::collect_bat_information(collection_data);

  // const absolute_time_t time_since_last_collection = absolute_time_diff_us(last_loop_start_time, get_absolute_time());
  // payload_data_manager::DataInstance data_inst(collection_data, time_since_last_collection);
  payload_state_manager.state_changed(collection_data);
}
