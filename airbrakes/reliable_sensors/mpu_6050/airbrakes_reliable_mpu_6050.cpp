#include "airbrakes_reliable_mpu_6050.h"

#include "airbrakes_state_manager.h"

AirbrakesReliableMPU6050::AirbrakesReliableMPU6050(AirbrakesStateManager* override_state_manager) : ReliableMPU6050(
  override_state_manager, AirbrakesFaultKey::MPU6050,i2c0, MPU_6050_ADDR, MPU6050::GyroFullScaleRange::Range500,
  MPU6050::AccelFullScaleRange::Range4g, AirbrakesPersistentStateKey::AccelCalibX,
  AirbrakesPersistentStateKey::AccelCalibY, AirbrakesPersistentStateKey::AccelCalibZ,
  AirbrakesPersistentStateKey::GyroCalibX, AirbrakesPersistentStateKey::GyroCalibY,
  AirbrakesPersistentStateKey::GyroCalibZ)
{
}

void AirbrakesReliableMPU6050::update_state(AirbrakesState& state, const double xa, const double ya, const double za,
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
