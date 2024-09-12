#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

#include "pin_outs.h"
#include "bmp_180.h"
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

  bool status_ok = true;

  bmp_180::CalibrationData bmp_180_calib_data;
  if (!bmp_180::check_device_id() || !bmp_180::read_calibration_data(bmp_180_calib_data))
  {
    status_ok = false;
  }

  if (status_ok)
  {
    multicore_launch_core1(core_1_main);
    multicore_fifo_push_blocking(MC_FLAG_VALUE);
  }

  while (true)
  {
    if (multicore_fifo_rvalid() && multicore_fifo_pop_blocking() == MC_FLAG_VALUE)
    {
      power_led_on = true;
      gpio_put(POWER_PIN, power_led_on);
      break;
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
    // float temp = 27 - (voltage - 0.706) / 0.001721;
    // printf("Raw value: 0x%03x, voltage: %f V, temperature %f C\n", adc_raw, voltage, temp);

    long temp, press;
    bmp_180::read_press_temp(bmp_180::oss_setting::STANDARD, bmp_180_calib_data, temp, press);
    printf("temp : %d press : %d\n", temp, press);
  }
}