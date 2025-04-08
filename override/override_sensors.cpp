#include "override_sensors.h"

#include "liperior_battery.h"
#include "override_state_manager.h"
#include "pin_outs.h"

void sensors_init()
{
  bmp280 = new ReliableBMP280(override_state_manager, spi0, BMP_CS_PIN,
                                                             BMP280::FilterCoefficientSetting::Filter16x,
                                                             BMP280::OssSettingPressure::PressureOss16,
                                                             BMP280::OssSettingTemperature::TemperatureOss16);
  mpu6050 = new ReliableMPU6050(override_state_manager, i2c0, MPU_6050_ADDR, MPU6050::GyroFullScaleRange::Range2000,
                                MPU6050::AccelFullScaleRange::Range16g);

  battery = new OvonicBattery(BAT_VOLTAGE_PIN, 32);
}
