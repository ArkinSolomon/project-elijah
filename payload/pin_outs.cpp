#include "pin_outs.h"

#include "i2c_util.h"
#include <hardware/adc.h>
#include <hardware/pwm.h>

#include "aprs.h"

void pin_init()
{
  i2c_util::i2c_bus_init(i2c0, I2C0_SDA_PIN, I2C0_SCL_PIN, 400 * 1000);
  i2c_util::i2c_bus_init(i2c1, I2C1_SDA_PIN, I2C1_SCL_PIN, 400 * 1000);

  gpio_init(CORE_0_LED_PIN);
  gpio_set_dir(CORE_0_LED_PIN, GPIO_OUT);
  gpio_init(CORE_1_LED_PIN);
  gpio_set_dir(CORE_1_LED_PIN, GPIO_OUT);

  // See sd_hw_config.c for microSD card SPI setup

  // SPI at 33MHz for W25Q64FV
  // gpio_set_function(SPI1_SCK_PIN, GPIO_FUNC_SPI);
  // gpio_set_function(SPI1_TX_PIN, GPIO_FUNC_SPI);
  // gpio_set_function(SPI1_RX_PIN, GPIO_FUNC_SPI);
  //
  // gpio_init(SPI1_CSN_PIN);
  // gpio_set_dir(SPI1_CSN_PIN, GPIO_OUT);
  // gpio_put(SPI1_CSN_PIN, true);
  //
  // spi_set_slave(spi1, false);
  // spi_init(spi1, 33 * 1000 * 1000);

  // Battery ADC
  adc_init();
  adc_gpio_init(BAT_VOLTAGE_PIN);

  gpio_init(RADIO_PTT_PIN);
  gpio_set_dir(RADIO_PTT_PIN, GPIO_OUT);
  aprs::init_aprs_system(SYS_CLK_KHZ, RADIO_PTT_PIN, PWM_CHAN_B);
}
