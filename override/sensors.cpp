#include "sensors.h"

#include "override_state_manager.h"
#include "pin_outs.h"

void sensors_init()
{
  bmp280 = new OverrideReliableBMP280(override_state_manager);
  mpu6050 = new OverrideReliableMPU6050(override_state_manager);

  battery = new Battery(BAT_VOLTAGE_PIN, 32, 3.125);
}
