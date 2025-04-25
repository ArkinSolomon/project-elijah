#pragma once

#include "core1.h"
#include "battery.h"
#include "elijah_state_framework.h"
#include "airbrakes_flight_phase_controller.h"
#include "airbrake_controls.h"
#include "reliable_bmp_280.h"
#include "reliable_mpu_6050.h"
#include "standard_command_helpers.h"

#define AIRBRAKES_STATE_TEMPLATE_TYPES \
  AirbrakesState, \
  AirbrakesPersistentKey, \
  AirbrakesFaultKey, \
  elijah_state_framework::std_helpers::StandardFlightPhase, \
  AirbrakesFlightPhaseController

struct AirbrakesState
{
  double ms_since_last;
  STANDARD_STATE_COLLECTION_DATA
  double bat_voltage, bat_percent;
  int32_t curr_encoder_pos, target_encoder_pos;
  double calculated_angle;
  int32_t calculated_encoder_pos;
  double modified_accel_x, modified_accel_y, modified_accel_z;
  double velocity;
};

enum class AirbrakesPersistentKey : uint8_t
{
  STANDARD_PERSISTENT_KEYS,
  ChosenTrajectory,
  AdditionalAccelXCalib,
  AdditionalAccelYCalib,
  AdditionalAccelZCalib,
};

enum class AirbrakesFaultKey : uint8_t
{
  MicroSD = 1,
  BMP280 = 2,
  MPU6050 = 3,
  Encoder = 4
};

extern ReliableBMP280<AIRBRAKES_STATE_TEMPLATE_TYPES>* bmp280;
extern ReliableMPU6050<AIRBRAKES_STATE_TEMPLATE_TYPES>* mpu6050;
extern Battery* battery;

class AirbrakesStateManager final : public elijah_state_framework::ElijahStateFramework<AIRBRAKES_STATE_TEMPLATE_TYPES>
{
public:
  AirbrakesStateManager() : ElijahStateFramework("Airbrakes", 100)
  {
    REGISTER_STANDARD_KEYS(AirbrakesPersistentKey)
    get_persistent_storage()->register_key(AirbrakesPersistentKey::ChosenTrajectory, "Chosen trajectory",
                                           static_cast<uint8_t>(0));
    get_persistent_storage()->register_key(AirbrakesPersistentKey::AdditionalAccelXCalib, "Accel calib+ X", 0.0);
    get_persistent_storage()->register_key(AirbrakesPersistentKey::AdditionalAccelYCalib, "Accel calib+ Y", 0.0);
    get_persistent_storage()->register_key(AirbrakesPersistentKey::AdditionalAccelZCalib, "Accel calib+ Z", 0.0);
    get_persistent_storage()->finish_registration();

    register_fault(AirbrakesFaultKey::MicroSD, "MicroSD", CommunicationChannel::SPI_0);
    register_fault(AirbrakesFaultKey::BMP280, "BMP 280", CommunicationChannel::SPI_0);
    register_fault(AirbrakesFaultKey::MPU6050, "MPU 6050", CommunicationChannel::I2C_0);
    register_fault(AirbrakesFaultKey::Encoder, "Encoder", CommunicationChannel::None);

    StdCommandRegistrationHelpers::register_calibration_command(this, &bmp280, &mpu6050, 0, 0, GRAVITY_CONSTANT);
    StdCommandRegistrationHelpers::register_persistent_storage_reset_helper(this, &mpu6050);
    StdCommandRegistrationHelpers::register_test_data_command(this);

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

    register_command("Set encoder pos", "Encoder position", [this](const double encoder_pos)
    {
      critical_section_enter_blocking(&core1::encoder_pos_cs);
      core1::current_encoder_pos = static_cast<int32_t>(std::round(encoder_pos));
      critical_section_exit(&core1::encoder_pos_cs);
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
    ENCODE_STATE(modified_accel_x, DataType::Double, "Modified acceleration X", "m/s^2")
    ENCODE_STATE(modified_accel_y, DataType::Double, "Modified acceleration Y", "m/s^2")
    ENCODE_STATE(modified_accel_z, DataType::Double, "Modified acceleration Z", "m/s^2")
    ENCODE_STATE(velocity, DataType::Double, "Velocity", "m/s")
  END_STATE_ENCODER()
};

inline AirbrakesStateManager* airbrakes_state_manager = nullptr;
