#include "sensors.h"

#include <hardware/adc.h>
#include <hardware/gpio.h>

#include "i2c_util.h"
#include "pin_outs.h"

void sensors_init()
{
  gpio_init(LED_2_PIN);
  gpio_set_dir(LED_2_PIN, GPIO_OUT);
  gpio_init(LED_3_PIN);
  gpio_set_dir(LED_3_PIN, GPIO_OUT);
  gpio_init(ONBOARD_LED_PIN);
  gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);

  gpio_set_function(SPI0_RX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI0_TX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI0_SCK_PIN, GPIO_FUNC_SPI);
  gpio_init(BMP_CS_PIN);
  gpio_set_dir(BMP_CS_PIN, GPIO_OUT);
  spi_init(spi0, 10 * 1000 * 1000);

  adc_init();

  i2c_util::i2c_bus_init(i2c0, I2C0_SDA_PIN, I2C0_SCL_PIN, 400 * 1000);

  bmp280 = new BMP280(spi0, BMP_CS_PIN);
  bmp280->read_calibration_data();
  bmp280->change_settings(BMP280::DeviceMode::NormalMode, BMP280::StandbyTimeSetting::Standby500us,
                       BMP280::FilterCoefficientSetting::FilterOff, BMP280::OssSettingPressure::PressureOss2,
                       BMP280::OssSettingTemperature::TemperatureOss1);

  mpu6050 = new MPU6050(i2c0, MPU_6050_ADDR, MPU6050::GyroFullScaleRange::Range500,
                        MPU6050::AccelFullScaleRange::Range4g);
  mpu6050->configure_default();

  battery = new Battery(BAT_VOLTAGE_PIN, 32, 3.125);
}
