#include "pin_outs.h"

#include "i2c_util.h"
#include <hardware/adc.h>
#include <hardware/pwm.h>

#include "aprs.h"

void pin_init()
{
  i2c_util::i2c_bus_init(i2c0, I2C0_SDA_PIN, I2C0_SCL_PIN, 400 * 1000);
  i2c_util::i2c_bus_init(i2c1, I2C1_SDA_PIN, I2C1_SCL_PIN, 400 * 1000);

  gpio_init(LED_2_PIN);
  gpio_set_dir(LED_2_PIN, GPIO_OUT);
  gpio_init(LED_3_PIN);
  gpio_set_dir(LED_3_PIN, GPIO_OUT);
  gpio_init(ONBOARD_LED_PIN);
  gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);

  gpio_init(BACKUP_BMP_280_CSN_PIN);
  gpio_set_dir(BACKUP_BMP_280_CSN_PIN, GPIO_OUT);

  gpio_init(SPI1_SCK_PIN);
  gpio_set_function(SPI1_SCK_PIN, GPIO_FUNC_SPI);
  gpio_init(SPI1_TX_PIN);
  gpio_set_function(SPI1_TX_PIN, GPIO_FUNC_SPI);
  gpio_init(SPI1_RX_PIN);
  gpio_set_function(SPI1_RX_PIN, GPIO_FUNC_SPI);
  spi_init(spi1, 10 * 1000 *    1000);

  gpio_init(SPEAKER_GND_PIN);
  gpio_set_dir(SPEAKER_GND_PIN, GPIO_OUT);
  gpio_put(SPEAKER_GND_PIN, true);
  elijah_state_framework::speaker_controller::init(SPEAKER_PIN);

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
  gpio_put(RADIO_PTT_PIN, true);
}
