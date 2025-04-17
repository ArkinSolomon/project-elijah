#pragma once

#include <hardware/gpio.h>

#include "battery.h"
#include "elijah_state_framework.h"
#include "override_flight_phase_controller.h"
#include "reliable_bmp_280.h"
#include "reliable_mpu_6050.h"
#include "standard_command_helpers.h"

#define OVERRIDE_STATE_TEMPLATE_TYPES \
OverrideState, \
OverridePersistentKey, \
OverrideFaultKey, \
elijah_state_framework::std_helpers::StandardFlightPhase, \
OverrideFlightPhaseController

struct OverrideState
{
  STANDARD_STATE_COLLECTION_DATA
  double bat_voltage, bat_percent;
};

enum class OverridePersistentKey : uint8_t
{
  STANDARD_PERSISTENT_KEYS
};

enum class OverrideFaultKey : uint8_t
{
  MicroSD = 1,
  BMP280 = 2,
  MPU6050 = 3,
};

extern ReliableBMP280<OVERRIDE_STATE_TEMPLATE_TYPES>* bmp280;
extern ReliableMPU6050<OVERRIDE_STATE_TEMPLATE_TYPES>* mpu6050;
extern Battery* battery;

class OverrideStateManager final : public elijah_state_framework::ElijahStateFramework<OVERRIDE_STATE_TEMPLATE_TYPES>
{
public:
  OverrideStateManager() : ElijahStateFramework("Override", 100)
  {
    REGISTER_STANDARD_KEYS(OverridePersistentKey)
    get_persistent_storage()->finish_registration();

    register_fault(OverrideFaultKey::MicroSD, "MicroSD", CommunicationChannel::SPI_0);
    register_fault(OverrideFaultKey::BMP280, "BMP 280", CommunicationChannel::SPI_0);
    register_fault(OverrideFaultKey::MPU6050, "MPU 6050", CommunicationChannel::I2C_0);

    StdCommandRegistrationHelpers::register_calibration_command(this, &bmp280, &mpu6050, 0, GRAVITY_CONSTANT, 0);
    StdCommandRegistrationHelpers::register_persistent_storage_reset_helper(this, &mpu6050);
    StdCommandRegistrationHelpers::register_test_data_command(this);
    finish_construction();
  }

protected:
  START_STATE_ENCODER(OverrideState)
    ENCODE_STATE(pressure, DataType::Int32, "Pressure", "Pa")
    ENCODE_STATE(temperature, DataType::Double, "Temperature", "degC")
    ENCODE_STATE(altitude, DataType::Double, "Altitude", "m")
    ENCODE_STATE(accel_x, DataType::Double, "Acceleration X", "m/s^2")
    ENCODE_STATE(accel_y, DataType::Double, "Acceleration Y", "m/s^2")
    ENCODE_STATE(accel_z, DataType::Double, "Acceleration Z", "m/s^2")
    ENCODE_STATE(gyro_x, DataType::Double, "Gyro X", "deg/s")
    ENCODE_STATE(gyro_y, DataType::Double, "Gyro Y", "deg/s")
    ENCODE_STATE(gyro_z, DataType::Double, "Gyro Z", "deg/s")
    ENCODE_STATE(bat_voltage, DataType::Double, "Voltage", "V")
    ENCODE_STATE(bat_percent, DataType::Double, "Battery percentage", "%")
  END_STATE_ENCODER()
};

inline OverrideStateManager* override_state_manager = nullptr;
