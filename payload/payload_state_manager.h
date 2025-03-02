#pragma once

#include "elijah_state_framework.h"
#include "data_type.h"

struct CollectionData
{
  uint64_t sequence;
  tm time_inst;
  int32_t pressure;
  double temperature;
  double altitude;
  double accel_x, accel_y, accel_z;
  double gyro_x, gyro_y, gyro_z;
  double bat_voltage, bat_percent;
};

enum class PayloadPersistentDataKey
{
  SeaLevelPressure,
  AccelCalibX,
  AccelCalibY,
  AccelCalibZ,
  GyroCalibX,
  GyroCalibY,
  GyroCalibZ
};

class PayloadStateManager final : public ElijahStateFramework<CollectionData, PayloadPersistentDataKey>
{
public:
  PayloadStateManager(): ElijahStateFramework("Payload")
  {
    PersistentDataStorage<PayloadPersistentDataKey> persistent_data_storage = get_persistent_data_storage();
    persistent_data_storage.register_key(PayloadPersistentDataKey::SeaLevelPressure, "Barometric pressure",
                                         101325.0);
    persistent_data_storage.register_key(PayloadPersistentDataKey::AccelCalibX, "Accelerometer calibration X", 0.0);
    persistent_data_storage.register_key(PayloadPersistentDataKey::AccelCalibY, "Accelerometer calibration Y", 0.0);
    persistent_data_storage.register_key(PayloadPersistentDataKey::AccelCalibZ, "Accelerometer calibration Z", 0.0);
    persistent_data_storage.register_key(PayloadPersistentDataKey::GyroCalibX, "Gyroscope calibration X", 0.0);
    persistent_data_storage.register_key(PayloadPersistentDataKey::GyroCalibY, "Gyroscope calibration Y", 0.0);
    persistent_data_storage.register_key(PayloadPersistentDataKey::GyroCalibZ, "Gyroscope calibration Z", 0.0);
    persistent_data_storage.finish_registration();

    finish_registration();
  }

protected:
  START_STATE_ENCODER(CollectionData)
     ENCODE_STATE(sequence, DataType::UInt64, "Sequence", "")
     ENCODE_STATE(time_inst, DataType::Time, "Time", "")
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

inline auto payload_state_manager = PayloadStateManager();
