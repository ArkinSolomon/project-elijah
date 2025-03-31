#include "payload_reliable_mpu_6050.h"

#include "payload_state_manager.h"

PayloadReliableMPU6050::PayloadReliableMPU6050(PayloadStateManager* payload_state_manager) : ReliableMPU6050(
  payload_state_manager, PayloadFaultKey::MPU6050, i2c1, MPU_6050_ADDR, MPU6050::GyroFullScaleRange::Range2000,
  MPU6050::AccelFullScaleRange::Range16g, PayloadPersistentDataKey::AccelCalibX,
  PayloadPersistentDataKey::AccelCalibY, PayloadPersistentDataKey::AccelCalibZ,
  PayloadPersistentDataKey::GyroCalibX, PayloadPersistentDataKey::GyroCalibY,
  PayloadPersistentDataKey::GyroCalibZ)
{
}

void PayloadReliableMPU6050::update_state(PayloadState& state, const double xa, const double ya, const double za,
                                          const double xg, const double yg,
                                          const double zg) const
{
  state.accel_x = xa;
  state.accel_y = ya;
  state.accel_z = za;

  state.gyro_x = xg;
  state.gyro_y = yg;
  state.gyro_z = zg;
}
