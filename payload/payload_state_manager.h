#pragma once

#include "elijah_state_framework.h"
#include "data_type.h"
#include "mpu_6050.h"
#include "payload_flight_phase_controller.h"
#include "sensors.h"
#include "sensors/ds_1307/ds_1307.h"

struct PayloadState
{
  tm time_inst;
  int32_t pressure;
  double temperature;
  double altitude;
  double accel_x, accel_y, accel_z;
  double gyro_x, gyro_y, gyro_z;
  double bat_voltage, bat_percent;
};

enum class PayloadPersistentDataKey : uint8_t
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

enum class PayloadFaultKey : uint8_t
{
  BMP280 = 1,
  MPU6050 = 2,
  MicroSD = 3,
  OnboardClock = 4,
  DS1307 = 5
};

class PayloadStateManager final : public elijah_state_framework::ElijahStateFramework<
    PayloadState, PayloadPersistentDataKey, PayloadFaultKey, StandardFlightPhase, PayloadFlightPhaseController>
{
public:
  PayloadStateManager(): ElijahStateFramework("Payload", PayloadPersistentDataKey::LaunchKey, PayloadFaultKey::MicroSD, 10)
  {
    get_persistent_data_storage()->register_key(PayloadPersistentDataKey::SeaLevelPressure, "Barometric pressure",
                                                101325.0);
    get_persistent_data_storage()->register_key(PayloadPersistentDataKey::AccelCalibX, "Accelerometer calibration X",
                                                0.0);
    get_persistent_data_storage()->register_key(PayloadPersistentDataKey::AccelCalibY, "Accelerometer calibration Y",
                                                0.0);
    get_persistent_data_storage()->register_key(PayloadPersistentDataKey::AccelCalibZ, "Accelerometer calibration Z",
                                                0.0);
    get_persistent_data_storage()->register_key(PayloadPersistentDataKey::GyroCalibX, "Gyroscope calibration X", 0.0);
    get_persistent_data_storage()->register_key(PayloadPersistentDataKey::GyroCalibY, "Gyroscope calibration Y", 0.0);
    get_persistent_data_storage()->register_key(PayloadPersistentDataKey::GyroCalibZ, "Gyroscope calibration Z", 0.0);
    get_persistent_data_storage()->register_key(PayloadPersistentDataKey::GroundPressure, "Ground altitude",
                                                static_cast<int32_t>(0));
    get_persistent_data_storage()->register_key(PayloadPersistentDataKey::GroundTemperature, "Ground temperature",
                                                0.0);
    get_persistent_data_storage()->register_key(PayloadPersistentDataKey::GroundAltitude, "Ground altitude", 0.0);
    get_persistent_data_storage()->finish_registration();

    register_fault(PayloadFaultKey::BMP280, "BMP 280", CommunicationChannel::I2C_0);
    register_fault(PayloadFaultKey::MPU6050, "MPU 6050", CommunicationChannel::I2C_1);
    register_fault(PayloadFaultKey::MicroSD, "MicroSD", CommunicationChannel::SPI_0);
    register_fault(PayloadFaultKey::OnboardClock, "Onboard Clock", CommunicationChannel::None);
    register_fault(PayloadFaultKey::DS1307, "DS 1307", CommunicationChannel::I2C_0);

    register_command("Calibrate", [this]
    {
      mpu6050->calibrate(100, 0, -GRAVITY_CONSTANT, 0, 0, 0, 0);

      const double sea_level_pressure = get_persistent_data_storage()->get_double(
        PayloadPersistentDataKey::SeaLevelPressure);
      int32_t pressure;
      double temperature, altitude;
      bmp280->get_bmp280().read_press_temp_alt(pressure, temperature, altitude, sea_level_pressure);

      get_persistent_data_storage()->set_int32(PayloadPersistentDataKey::GroundPressure, pressure);
      get_persistent_data_storage()->set_double(PayloadPersistentDataKey::GroundTemperature, temperature);
      get_persistent_data_storage()->set_double(PayloadPersistentDataKey::GroundAltitude, altitude);

      get_persistent_data_storage()->commit_data();
    });

    register_command("Update clock", [this](const tm& time_inst)
    {
      ds_1307::init_clock_with_inst(time_inst);
    });

    finish_construction();
  }

protected:
  START_STATE_ENCODER(PayloadState)
    ENCODE_TIME_STATE(time_inst, "Time")
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

inline PayloadStateManager* payload_state_manager = nullptr;
