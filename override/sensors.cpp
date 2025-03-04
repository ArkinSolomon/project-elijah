#include "sensors.h"

#include <hardware/adc.h>

#include "i2c_util.h"
#include "pin_outs.h"

void sensors_init()
{
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
