#include "payload_sensors.h"

#include "ovonic_battery.h"
#include "payload_state_manager.h"
#include "pin_outs.h"

void sensors_init()
{
  bmp280 = new ReliableBMP280(payload_state_manager, spi1, BACKUP_BMP_280_CSN_PIN,
                              BMP280::FilterCoefficientSetting::Filter16x,
                              BMP280::OssSettingPressure::PressureOss16,
                              BMP280::OssSettingTemperature::TemperatureOss16);
  mpu6050 = new ReliableMPU6050(payload_state_manager, i2c1, MPU_6050_ADDR,
                                MPU6050::GyroFullScaleRange::Range2000,
                                MPU6050::AccelFullScaleRange::Range16g);

  battery = new OvonicBattery(BAT_VOLTAGE_PIN, 32);
  reliable_clock = new ReliableClock();
}
