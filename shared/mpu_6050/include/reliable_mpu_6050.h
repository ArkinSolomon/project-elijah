#pragma once

#include <hardware/gpio.h>

#include "mpu_6050.h"
#include "pin_outs.h"
#include "reliable_component_helper.h"

FRAMEWORK_TEMPLATE_DECL
class ReliableMPU6050 : public elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>
{
public:
  ReliableMPU6050(elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework,
                  i2c_inst_t* i2c_inst, uint8_t i2c_addr,
                  MPU6050::GyroFullScaleRange default_gyro_range,
                  MPU6050::AccelFullScaleRange default_accel_range);

  [[nodiscard]] MPU6050& get_mpu_6050();

  void calibrate(unsigned int cycles, double xa, double ya, double za, double xg, double yg, double zg);
  void load_calibration_data();

protected:
  std::string on_init(TStateData& state) override;
  std::string on_update(TStateData& state) override;

private:
  static constexpr EPersistentStorageKey calib_xa_key = EPersistentStorageKey::AccelCalibX;
  static constexpr EPersistentStorageKey calib_ya_key = EPersistentStorageKey::AccelCalibY;
  static constexpr EPersistentStorageKey calib_za_key = EPersistentStorageKey::AccelCalibZ;
  static constexpr EPersistentStorageKey calib_xg_key = EPersistentStorageKey::GyroCalibX;
  static constexpr EPersistentStorageKey calib_yg_key = EPersistentStorageKey::GyroCalibY;
  static constexpr EPersistentStorageKey calib_zg_key = EPersistentStorageKey::GyroCalibZ;

  MPU6050 mpu;
};

FRAMEWORK_TEMPLATE_DECL
ReliableMPU6050<FRAMEWORK_TEMPLATE_TYPES>::ReliableMPU6050(
  elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework,
  i2c_inst_t* i2c_inst, const uint8_t i2c_addr, const MPU6050::GyroFullScaleRange default_gyro_range,
  const MPU6050::AccelFullScaleRange default_accel_range) :
  elijah_state_framework::ReliableComponentHelper<FRAMEWORK_TEMPLATE_TYPES>(framework, EFaultKey::MPU6050),
  mpu(i2c_inst, i2c_addr, default_gyro_range, default_accel_range)
{
}

FRAMEWORK_TEMPLATE_DECL
MPU6050& ReliableMPU6050<FRAMEWORK_TEMPLATE_TYPES>::get_mpu_6050()
{
  return mpu;
}

FRAMEWORK_TEMPLATE_DECL
void ReliableMPU6050<FRAMEWORK_TEMPLATE_TYPES>::calibrate(const unsigned int cycles, const double xa, const double ya,
                                                          const double za, const double xg, const double yg,
                                                          const double zg)
{
  mpu.calibrate(cycles, xa, ya, za, xg, yg, zg);
#define MPU_OVERRIDE_PERSISTENT_STATE_SAVE(K, PROP) this->get_framework()->get_persistent_storage()->set_double(K, mpu.get_calibration_data().PROP)
  MPU_OVERRIDE_PERSISTENT_STATE_SAVE(calib_xa_key, diff_xa);
  MPU_OVERRIDE_PERSISTENT_STATE_SAVE(calib_ya_key, diff_ya);
  MPU_OVERRIDE_PERSISTENT_STATE_SAVE(calib_za_key, diff_za);
  MPU_OVERRIDE_PERSISTENT_STATE_SAVE(calib_xg_key, diff_yg);
  MPU_OVERRIDE_PERSISTENT_STATE_SAVE(calib_yg_key, diff_yg);
  MPU_OVERRIDE_PERSISTENT_STATE_SAVE(calib_zg_key, diff_zg);
#undef MPU_OVERRIDE_PERSISTENT_STATE_SAVE
}

FRAMEWORK_TEMPLATE_DECL
void ReliableMPU6050<FRAMEWORK_TEMPLATE_TYPES>::load_calibration_data()
{
#define PERSISTENT_CALIB_GET(K) this->get_framework()->get_persistent_storage()->get_double(K)
  mpu.load_calibration_data(
    PERSISTENT_CALIB_GET(calib_xa_key),
    PERSISTENT_CALIB_GET(calib_ya_key),
    PERSISTENT_CALIB_GET(calib_za_key),
    PERSISTENT_CALIB_GET(calib_xg_key),
    PERSISTENT_CALIB_GET(calib_yg_key),
    PERSISTENT_CALIB_GET(calib_zg_key)
  );
#undef PERSISTENT_CALIB_GET
}

FRAMEWORK_TEMPLATE_DECL
std::string ReliableMPU6050<FRAMEWORK_TEMPLATE_TYPES>::on_init(TStateData& state)
{
  if (!mpu.check_chip_id())
  {
    return "Failed to read chip id";
  }

  if (!mpu.configure_default())
  {
    return "Failed to configure";
  }

  load_calibration_data();
  return "";
}

FRAMEWORK_TEMPLATE_DECL
std::string ReliableMPU6050<FRAMEWORK_TEMPLATE_TYPES>::on_update(TStateData& state)
{
  double xa, ya, za;
  double xg, yg, zg;

  if (!mpu.get_data(xa, ya, za, xg, yg, zg))
  {
    return "Failed to get data";
  }

  state.accel_x = xa;
  state.accel_y = ya;
  state.accel_z = za;
  state.gyro_x = xg;
  state.gyro_y = yg;
  state.gyro_z = zg;

  return "";
}
