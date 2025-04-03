#pragma once

#include "core1.h"
#include "elijah_state_framework.h"
#include "airbrakes_flight_phase_controller.h"
#include "airbrake_controls.h"
#include "sensors.h"

enum class StandardFlightPhase : uint8_t;

struct AirbrakesState
{
  double ms_since_last;
  int32_t pressure;
  double temperature;
  double altitude;
  double accel_x, accel_y, accel_z;
  double gyro_x, gyro_y, gyro_z;
  double bat_voltage, bat_percent;
  int32_t curr_encoder_pos, target_encoder_pos;
  double calculated_angle;
  int32_t calculated_encoder_pos;
};

enum class AirbrakesPersistentStateKey : uint8_t
{
  LaunchKey = 1,
  AccelCalibX = 3,
  AccelCalibY = 4,
  AccelCalibZ = 5,
  GyroCalibX = 6,
  GyroCalibY = 7,
  GyroCalibZ = 8,
  GroundPressure = 9,
  GroundTemperature = 10,
  IsCalibrated = 12
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
                                                 AirbrakesFaultKey::MicroSD, 100)
  {
    get_persistent_storage()->register_key(AirbrakesPersistentStateKey::AccelCalibX, "Accelerometer calibration X",
                                                0.0);
    get_persistent_storage()->register_key(AirbrakesPersistentStateKey::AccelCalibY, "Accelerometer calibration Y",
                                                0.0);
    get_persistent_storage()->register_key(AirbrakesPersistentStateKey::AccelCalibZ, "Accelerometer calibration Z",
                                                0.0);
    get_persistent_storage()->
      register_key(AirbrakesPersistentStateKey::GyroCalibX, "Gyroscope calibration X", 0.0);
    get_persistent_storage()->
      register_key(AirbrakesPersistentStateKey::GyroCalibY, "Gyroscope calibration Y", 0.0);
    get_persistent_storage()->
      register_key(AirbrakesPersistentStateKey::GyroCalibZ, "Gyroscope calibration Z", 0.0);
    get_persistent_storage()->register_key(AirbrakesPersistentStateKey::GroundPressure, "Ground pressure",
                                                static_cast<int32_t>(0));
    get_persistent_storage()->register_key(AirbrakesPersistentStateKey::GroundTemperature, "Ground temperature",
                                                0.0);
    get_persistent_storage()->register_key(AirbrakesPersistentStateKey::IsCalibrated, "Is calibrated", static_cast<uint8_t>(0));
    get_persistent_storage()->finish_registration();

    register_fault(AirbrakesFaultKey::MicroSD, "MicroSD", CommunicationChannel::SPI_0);
    register_fault(AirbrakesFaultKey::BMP280, "BMP 280", CommunicationChannel::SPI_0);
    register_fault(AirbrakesFaultKey::MPU6050, "MPU 6050", CommunicationChannel::I2C_0);

    register_command("Calibrate", [this]
    {
      mpu6050->calibrate(100, 0, 0, -GRAVITY_CONSTANT, 0, 0, 0);

      int32_t pressure;
      double temperature;
      bmp280->get_bmp280().read_press_temp(pressure, temperature);

      get_persistent_storage()->set_int32(AirbrakesPersistentStateKey::GroundPressure, pressure);
      get_persistent_storage()->set_double(AirbrakesPersistentStateKey::GroundTemperature, temperature);
      get_persistent_storage()->set_uint8(AirbrakesPersistentStateKey::IsCalibrated, 0xFF);

      get_persistent_storage()->commit_data();
    });

    register_command("Reset persistent storage", [this]
    {
      get_persistent_storage()->load_default_data();
      mpu6050->load_calibration_data();
    });

    register_command("Manual target toggle", [this]
    {
      critical_section_enter_blocking(&core1::target_access_cs);
      core1::angle_override_active = !core1::angle_override_active;
      if (!core1::angle_override_active)
      {
        core1::target_encoder_pos = 0;
      }
      critical_section_exit(&core1::target_access_cs);
    });

    register_command("Set angle", "Target angle (degrees)", [this](double target_angle)
    {
      critical_section_enter_blocking(&core1::target_access_cs);
      if (core1::angle_override_active)
      {
        core1::target_encoder_pos = encoder_pos_from_angle(target_angle);
      }
      critical_section_exit(&core1::target_access_cs);
    });

    register_command("Next flight phase", [this]
    {
      StandardFlightPhase curr_phase = get_current_flight_phase();
      StandardFlightPhase next_phase;
      if (get_current_flight_phase() == StandardFlightPhase::LANDED)
      {
        next_phase = StandardFlightPhase::PREFLIGHT;
      }
      else
      {
        next_phase = static_cast<StandardFlightPhase>(static_cast<std::underlying_type_t<StandardFlightPhase>>(
          curr_phase) + 1);
      }
      set_flight_phase(next_phase);
    });

    register_command("Reconfigure MPU 6050", [this]
    {
      if (!mpu6050->get_mpu_6050().configure_default())
      {
        set_fault(AirbrakesFaultKey::MPU6050, true, "Failed to reconfigure from command input");
      }
      else
      {
        set_fault(AirbrakesFaultKey::MPU6050, false);
      }
    });

    finish_construction();
  }

protected:
  START_STATE_ENCODER(AirbrakesState)
    ENCODE_STATE(ms_since_last, DataType::Double, "dt", "ms")
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
    ENCODE_STATE(curr_encoder_pos, DataType::Int32, "Current encoder", "")
    ENCODE_STATE(calculated_angle, DataType::Double, "Calculated angle", "deg")
    ENCODE_STATE(calculated_encoder_pos, DataType::Int32, "Calculated encoder", "")
  END_STATE_ENCODER()
};

inline AirbrakesStateManager* airbrakes_state_manager = nullptr;
