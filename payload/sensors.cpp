#include "sensors.h"

#include "ovonic_battery.h"
#include "payload_state_manager.h"


void sensors_init()
{
    bmp280 = new PayloadReliableBMP280(payload_state_manager);
    mpu6050 = new PayloadReliableMPU6050(payload_state_manager);

    battery = new OvonicBattery(BAT_VOLTAGE_PIN, 32);
    reliable_clock = new ReliableClock();
}
