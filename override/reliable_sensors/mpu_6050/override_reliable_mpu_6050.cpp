#include "override_reliable_mpu_6050.h"

#include "override_state_manager.h"

OverrideReliableMPU6050::OverrideReliableMPU6050(OverrideStateManager* override_state_manager) : ReliableMPU6050(
    override_state_manager, OverrideFaultKey::MPU6050, i2c0, MPU_6050_ADDR, MPU6050::GyroFullScaleRange::Range500,
    MPU6050::AccelFullScaleRange::Range4g, OverridePersistentStateKey::AccelCalibX,
    OverridePersistentStateKey::AccelCalibY, OverridePersistentStateKey::AccelCalibZ,
    OverridePersistentStateKey::GyroCalibX, OverridePersistentStateKey::GyroCalibY,
    OverridePersistentStateKey::GyroCalibZ)
{
}

void OverrideReliableMPU6050::update_state(OverrideState& state, const double xa, const double ya, const double za,
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
