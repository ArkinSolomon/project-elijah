#pragma once

#include "core1.h"
#include "elijah_state_framework.h"
#include "airbrakes_flight_phase_controller.h"
#include "airbrake_controls.h"
#include "sensors.h"

enum class StandardFlightPhase : uint8_t;

struct AirbrakesState
{
  int32_t pressure;
  double temperature;
  double altitude;
  double accel_x, accel_y, accel_z;
  double gyro_x, gyro_y, gyro_z;
  double bat_voltage, bat_percent;
  int32_t curr_encoder_pos, target_encoder_pos;
  double curr_angle, target_angle, calculated_angle;
};

enum class AirbrakesPersistentStateKey : uint8_t
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
  GroundAltitude = 11
};

enum class AirbrakesFaultKey : uint8_t
{
  MicroSD = 1,
  BMP280 = 2,
  MPU6050 = 3,
};

class AirbrakesStateManager final : public elijah_state_framework::ElijahStateFramework<
    AirbrakesState, AirbrakesPersistentStateKey, AirbrakesFaultKey, StandardFlightPhase, AirbrakesFlightPhaseController>
{
public:
  AirbrakesStateManager() : ElijahStateFramework("Airbrakes", AirbrakesPersistentStateKey::LaunchKey,
                                                 AirbrakesFaultKey::MicroSD, 10)
  {
    get_persistent_data_storage()->register_key(AirbrakesPersistentStateKey::SeaLevelPressure, "Barometric pressure",
                                                101083.7);
    get_persistent_data_storage()->register_key(AirbrakesPersistentStateKey::AccelCalibX, "Accelerometer calibration X",
                                                0.0);
    get_persistent_data_storage()->register_key(AirbrakesPersistentStateKey::AccelCalibY, "Accelerometer calibration Y",
                                                0.0);
    get_persistent_data_storage()->register_key(AirbrakesPersistentStateKey::AccelCalibZ, "Accelerometer calibration Z",
                                                0.0);
    get_persistent_data_storage()->
      register_key(AirbrakesPersistentStateKey::GyroCalibX, "Gyroscope calibration X", 0.0);
    get_persistent_data_storage()->
      register_key(AirbrakesPersistentStateKey::GyroCalibY, "Gyroscope calibration Y", 0.0);
    get_persistent_data_storage()->
      register_key(AirbrakesPersistentStateKey::GyroCalibZ, "Gyroscope calibration Z", 0.0);
    get_persistent_data_storage()->register_key(AirbrakesPersistentStateKey::GroundPressure, "Ground pressure",
                                                static_cast<int32_t>(0));
    get_persistent_data_storage()->register_key(AirbrakesPersistentStateKey::GroundTemperature, "Ground temperature",
                                                0.0);
    get_persistent_data_storage()->register_key(AirbrakesPersistentStateKey::GroundAltitude, "Ground altitude", 0.0);
    get_persistent_data_storage()->finish_registration();

    register_fault(AirbrakesFaultKey::MicroSD, "MicroSD", CommunicationChannel::SPI_0);
    register_fault(AirbrakesFaultKey::BMP280, "BMP 280", CommunicationChannel::SPI_0);
    register_fault(AirbrakesFaultKey::MPU6050, "MPU 6050", CommunicationChannel::SPI_0);

    register_command("Calibrate", [this]
    {
      mpu6050->calibrate(100, 0, 0, -GRAVITY_CONSTANT, 0, 0, 0);

      const double sea_level_pressure = get_persistent_data_storage()->get_double(
        AirbrakesPersistentStateKey::SeaLevelPressure);
      int32_t pressure;
      double temperature, altitude;
      bmp280->get_bmp280().read_press_temp_alt(pressure, temperature, altitude, sea_level_pressure);

      get_persistent_data_storage()->set_int32(AirbrakesPersistentStateKey::GroundPressure, pressure);
      get_persistent_data_storage()->set_double(AirbrakesPersistentStateKey::GroundTemperature, temperature);
      get_persistent_data_storage()->set_double(AirbrakesPersistentStateKey::GroundAltitude, altitude);

      get_persistent_data_storage()->commit_data();
    });

    register_command("Manual target toggle", [this]
    {
      critical_section_enter_blocking(&core1::target_access_cs);
      core1::angle_override_active = !core1::angle_override_active;
      critical_section_exit(&core1::target_access_cs);
    });

    register_command("Set angle", [this](double target_angle)
    {
      critical_section_enter_blocking(&core1::target_access_cs);
      if (!core1::angle_override_active)
      {
        core1::target_encoder_pos = encoder_pos_from_angle(target_angle);
      }
      critical_section_exit(&core1::target_access_cs);
    });

    finish_construction();
  }

protected:
  START_STATE_ENCODER(AirbrakesState)
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
    ENCODE_STATE(target_encoder_pos, DataType::Int32, "Encoder target", "")
    ENCODE_STATE(target_angle, DataType::Double, "Angle", "deg")
    ENCODE_STATE(curr_encoder_pos, DataType::Int32, "Current encoder", "")
    ENCODE_STATE(curr_angle, DataType::Double, "Current angle", "deg")
    ENCODE_STATE(calculated_angle, DataType::Double, "Calculated angle", "deg")
  END_STATE_ENCODER()
};

inline AirbrakesStateManager* airbrakes_state_manager = nullptr;
