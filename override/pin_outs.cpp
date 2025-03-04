#include "pin_outs.h"

#include "i2c_util.h"
#include <hardware/adc.h>
#include <hardware/spi.h>

void pin_init()
{
  gpio_init(LED_2_PIN);
  gpio_set_dir(LED_2_PIN, GPIO_OUT);
  gpio_init(LED_3_PIN);
  gpio_set_dir(LED_3_PIN, GPIO_OUT);
  gpio_init(ONBOARD_LED_PIN);
  gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);

  gpio_init(BMP_CS_PIN);
  gpio_set_dir(BMP_CS_PIN, GPIO_OUT);

  // SPI0 initialized by sd_hw_config.c
  i2c_util::i2c_bus_init(i2c0, I2C0_SDA_PIN, I2C0_SCL_PIN, 400 * 1000);

  adc_init();
  adc_gpio_init(BAT_VOLTAGE_PIN);
}
