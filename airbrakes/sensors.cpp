#include "sensors.h"

#include "liperior_battery.h"
#include "airbrakes_state_manager.h"
#include "pin_outs.h"

void sensors_init()
{
  bmp280 = new AirbrakesReliableBMP280(airbrakes_state_manager);
  mpu6050 = new AirbrakesReliableMPU6050(airbrakes_state_manager);

  battery = new LiperiorBattery(BAT_VOLTAGE_PIN, 32);
}
