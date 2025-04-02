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
  GyroCalibZ = 8,
  GroundPressure = 9,
  GroundTemperature = 10,
  GroundAltitude = 11,
  IsCalibrated = 12,
};

enum class OverrideFaultKey : uint8_t
{
  MicroSD = 1,
  BMP280 = 2,
  MPU6050 = 3,
};

class OverrideStateManager final : public elijah_state_framework::ElijahStateFramework<
    OverrideState, OverridePersistentStateKey, OverrideFaultKey, StandardFlightPhase, OverrideFlightPhaseController>
{
public:
  OverrideStateManager() : ElijahStateFramework("Override", OverridePersistentStateKey::LaunchKey,
                                                OverrideFaultKey::MicroSD, 100)
  {
    get_persistent_storage()->register_key(OverridePersistentStateKey::SeaLevelPressure, "Barometric pressure",
                                                101325.0);
    get_persistent_storage()->register_key(OverridePersistentStateKey::AccelCalibX, "Accelerometer calibration X",
                                                0.0);
    get_persistent_storage()->register_key(OverridePersistentStateKey::AccelCalibY, "Accelerometer calibration Y",
                                                0.0);
    get_persistent_storage()->register_key(OverridePersistentStateKey::AccelCalibZ, "Accelerometer calibration Z",
                                                0.0);
    get_persistent_storage()->register_key(OverridePersistentStateKey::GyroCalibX, "Gyroscope calibration X", 0.0);
    get_persistent_storage()->register_key(OverridePersistentStateKey::GyroCalibY, "Gyroscope calibration Y", 0.0);
    get_persistent_storage()->register_key(OverridePersistentStateKey::GyroCalibZ, "Gyroscope calibration Z", 0.0);
    get_persistent_storage()->register_key(OverridePersistentStateKey::GroundPressure, "Ground pressure",
                                                static_cast<int32_t>(0));
    get_persistent_storage()->register_key(OverridePersistentStateKey::GroundTemperature, "Ground temperature",
                                                0.0);
    get_persistent_storage()->register_key(OverridePersistentStateKey::GroundAltitude, "Ground altitude", 0.0);
    get_persistent_storage()->register_key(OverridePersistentStateKey::IsCalibrated, "Is calibrated", static_cast<uint8_t>(0));
    get_persistent_storage()->finish_registration();

    register_fault(OverrideFaultKey::MicroSD, "MicroSD", CommunicationChannel::SPI_0);
    register_fault(OverrideFaultKey::BMP280, "BMP 280", CommunicationChannel::SPI_0);
    register_fault(OverrideFaultKey::MPU6050, "MPU 6050", CommunicationChannel::I2C_0);

    register_command("Calibrate", "Sea level pressure (Pa)", [this](double sea_level_pressure)
    {
      mpu6050->calibrate(100, 0, -GRAVITY_CONSTANT, 0, 0, 0, 0);

      int32_t pressure;
      double temperature, altitude;
      bmp280->get_bmp280().read_press_temp_alt(pressure, temperature, altitude, sea_level_pressure);

      get_persistent_storage()->set_double(OverridePersistentStateKey::SeaLevelPressure, sea_level_pressure);
      get_persistent_storage()->set_int32(OverridePersistentStateKey::GroundPressure, pressure);
      get_persistent_storage()->set_double(OverridePersistentStateKey::GroundTemperature, temperature);
      get_persistent_storage()->set_double(OverridePersistentStateKey::GroundAltitude, altitude);
      get_persistent_storage()->set_uint8(OverridePersistentStateKey::IsCalibrated, 0xFF);

      get_persistent_storage()->commit_data();
    });

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
