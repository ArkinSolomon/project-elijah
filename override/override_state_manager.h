#pragma once

#include <hardware/gpio.h>

#include "elijah_state_framework.h"
#include "override_flight_phase_controller.h"
#include "sensors.h"

enum class StandardFlightPhase : uint8_t;

struct OverrideState
{
  int32_t pressure;
  double temperature;
  double altitude;
  double accel_x, accel_y, accel_z;
  double gyro_x, gyro_y, gyro_z;
  double bat_voltage, bat_percent;
};

enum class OverridePersistentStateKey : uint8_t
{
  LaunchKey = 1,
  SeaLevelPressure = 2,
  AccelCalibX = 3,
  AccelCalibY = 4,
  AccelCalibZ = 5,
  GyroCalibX = 6,
  GyroCalibY = 7,
  GyroCalibZ = 8
};

enum class FaultKey : uint8_t
{
  BMP280 = 1,
  MPU6050 = 2,
  MicroSD = 3
};

class OverrideStateManager final : public elijah_state_framework::ElijahStateFramework<OverrideState, OverridePersistentStateKey, FaultKey, StandardFlightPhase, OverrideFlightPhaseController>
{
public:
  OverrideStateManager() : ElijahStateFramework("Override", OverridePersistentStateKey::LaunchKey, 10)
  {
    get_persistent_data_storage()->register_key(OverridePersistentStateKey::SeaLevelPressure, "Barometric pressure",
                                                101325.0);
    get_persistent_data_storage()->register_key(OverridePersistentStateKey::AccelCalibX, "Accelerometer calibration X",
                                                0.0);
    get_persistent_data_storage()->register_key(OverridePersistentStateKey::AccelCalibY, "Accelerometer calibration Y",
                                                0.0);
    get_persistent_data_storage()->register_key(OverridePersistentStateKey::AccelCalibZ, "Accelerometer calibration Z",
                                                0.0);
    get_persistent_data_storage()->register_key(OverridePersistentStateKey::GyroCalibX, "Gyroscope calibration X", 0.0);
    get_persistent_data_storage()->register_key(OverridePersistentStateKey::GyroCalibY, "Gyroscope calibration Y", 0.0);
    get_persistent_data_storage()->register_key(OverridePersistentStateKey::GyroCalibZ, "Gyroscope calibration Z", 0.0);
    get_persistent_data_storage()->finish_registration();

    register_fault(FaultKey::BMP280, "BMP 280", CommunicationChannel::SPI_0);
    register_fault(FaultKey::MPU6050, "MPU 6050", CommunicationChannel::SPI_0);
    register_fault(FaultKey::MicroSD, "MicroSD", CommunicationChannel::SPI_0);

    register_command("Calibrate", [this]
    {
      mpu6050->calibrate(100, 0, -GRAVITY_CONSTANT, 0, 0, 0, 0);
#define MPU_OVERRIDE_PERSISTENT_STATE_SAVE(KEY, PROP) get_persistent_data_storage()->set_double(OverridePersistentStateKey::KEY, mpu6050->get_calibration_data().PROP)
      MPU_OVERRIDE_PERSISTENT_STATE_SAVE(AccelCalibX, diff_xa);
      MPU_OVERRIDE_PERSISTENT_STATE_SAVE(AccelCalibY, diff_ya);
      MPU_OVERRIDE_PERSISTENT_STATE_SAVE(AccelCalibZ, diff_za);
      MPU_OVERRIDE_PERSISTENT_STATE_SAVE(GyroCalibX, diff_yg);
      MPU_OVERRIDE_PERSISTENT_STATE_SAVE(GyroCalibY, diff_yg);
      MPU_OVERRIDE_PERSISTENT_STATE_SAVE(GyroCalibZ, diff_zg);
#undef MPU_OVERRIDE_PERSISTENT_STATE_SAVE
      get_persistent_data_storage()->commit_data();
    });

    finish_construction();
  }

protected:
  START_STATE_ENCODER(OverrideState)
    ENCODE_STATE(pressure, DataType::UInt32, "Pressure", "Pa")
    ENCODE_STATE(temperature, DataType::Double, "Temperature", "°C")
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
