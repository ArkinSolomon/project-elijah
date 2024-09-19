#include <cstdio>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

#include "pin_outs.h"
#include "sensors/bmp_180/bmp_180.h"
#include "sensors/ds_1307/ds_1307.h"
#include "core_1.h"

int main()
{
  stdio_init_all();

  // i2c@400kHz
  i2c_init(I2C_BUS, 400 * 1000);
  adc_init();

  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);

  gpio_init(TEMP_LED_PIN);
  gpio_set_dir(TEMP_LED_PIN, true);

  bool power_led_on = false;
  gpio_init(POWER_PIN);
  gpio_set_dir(POWER_PIN, true);

  adc_select_input(ONBOARD_TEMP_PIN);

  multicore_launch_core1(core_1_main);
  multicore_fifo_push_blocking(MC_FLAG_VALUE);

  bool calib_recieved = false;
  bool clock_ready = false;

  bmp_180::CalibrationData bmp_180_calib_data{};
  while (true)
  {
    if (clock_ready && calib_recieved && multicore_fifo_rvalid() && multicore_fifo_pop_blocking() == MC_FLAG_VALUE)
    {
      power_led_on = true;
      gpio_put(POWER_PIN, power_led_on);
      break;
    }

    if (!calib_recieved)
    {
      calib_recieved = bmp_180::check_device_id() && bmp_180::read_calibration_data(bmp_180_calib_data);
    }

    if (!clock_ready)
    {
      clock_ready = ds_1307::verify_clock();
      if (!clock_ready)
      {
        clock_ready = ds_1307::set_clock(2024, ds_1307::month_of_year::SEPTEMBER, ds_1307::day_of_week::FRIDAY,
                                         13, 5, 21, 30);
      }
    }

    gpio_put(POWER_PIN, power_led_on);
    power_led_on = !power_led_on;
    sleep_ms(100);
  }

  while (true)
  {
    // const float conversion_factor = 3.3f / 0x1000;
    // uint16_t adc_raw = adc_read();
    // float voltage = adc_raw * conversion_factor;
    // float t = 27 - (voltage - 0.706) / 0.001721;
    // printf("Raw value: 0x%03x, voltage: %f V, temperature %f C\n", adc_raw, voltage, t);

    // bmp_180_calib_data.AC1 = 408;
    // bmp_180_calib_data.AC2 = -72;
    // bmp_180_calib_data.AC3 = -14383;
    // bmp_180_calib_data.AC4 = 32741;
    // bmp_180_calib_data.AC5 = 32757;
    // bmp_180_calib_data.AC6 = 23153;
    // bmp_180_calib_data.B1 = 6190;
    // bmp_180_calib_data.B2 = 4;
    // bmp_180_calib_data.MB = -32768;
    // bmp_180_calib_data.MC = -8711;
    // bmp_180_calib_data.MD = 2868;

    int32_t press;
    double temp, altitude;
    bmp_180::read_press_temp_alt(bmp_180::oss_setting::ULTRA_HIGH, bmp_180_calib_data, temp, press, altitude);
    // bmp_180::print_calib_data(bmp_180_calib_data);
    const double alt_ft = altitude * 3.2808;
    const double temp_f = temp * ((double)9 / 5) + 32;

    ds_1307::TimeInstance time_inst{};
    ds_1307::get_time_instance(time_inst);

    printf(
      "[%d/%d/%d %d:%d:%d] temp (C): %.1f temp: (F): %.2f press (Pa): %d, altitude (m): %.3f altitude (ft): %.3f\n",
      time_inst.month, time_inst.date, time_inst.year, time_inst.hours, time_inst.minutes, time_inst.seconds,
      temp, temp_f, press, altitude, alt_ft);
  }
}
