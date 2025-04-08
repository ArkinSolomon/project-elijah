#pragma once
#include "elijah_state_framework.h"
#include "reliable_bmp_280.h"
#include "reliable_mpu_6050.h"

namespace elijah_state_framework::std_helpers
{
  class StdCommandRegistrationHelpers final
  {
  public:
    FRAMEWORK_TEMPLATE_DECL
    static void register_calibration_command(ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework,
                                             ReliableBMP280<FRAMEWORK_TEMPLATE_TYPES>** bmp280,
                                             ReliableMPU6050<FRAMEWORK_TEMPLATE_TYPES>** mpu6050);

    FRAMEWORK_TEMPLATE_DECL
    static void register_persistent_storage_reset_helper(ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework,
                                                         ReliableMPU6050<FRAMEWORK_TEMPLATE_TYPES>** mpu6050);
  };
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::std_helpers::StdCommandRegistrationHelpers::register_calibration_command(
  elijah_state_framework::ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework,
  ReliableBMP280<FRAMEWORK_TEMPLATE_TYPES>** bmp280,
  ReliableMPU6050<FRAMEWORK_TEMPLATE_TYPES>** mpu6050)
{
  // __breakpoint();
  framework->register_command("Calibrate", [framework, bmp280, mpu6050]
  {
    (*mpu6050)->calibrate(100, 0, -GRAVITY_CONSTANT, 0, 0, 0, 0);

    int32_t pressure;
    double temperature;
    (*bmp280)->get_bmp280().read_press_temp(pressure, temperature);

    framework->get_persistent_storage()->set_int32(EPersistentStorageKey::GroundPressure, pressure);
    framework->get_persistent_storage()->set_double(EPersistentStorageKey::GroundTemperature, temperature);
    framework->get_persistent_storage()->set_uint8(EPersistentStorageKey::IsCalibrated, 0xFF);

    framework->get_persistent_storage()->commit_data();
  });
}

FRAMEWORK_TEMPLATE_DECL
void elijah_state_framework::std_helpers::StdCommandRegistrationHelpers::register_persistent_storage_reset_helper(
  ElijahStateFramework<FRAMEWORK_TEMPLATE_TYPES>* framework, ReliableMPU6050<FRAMEWORK_TEMPLATE_TYPES>** mpu6050)
{
  framework->register_command("Reset persistent storage", [framework, mpu6050]
  {
    framework->get_persistent_storage()->load_default_data();
    (*mpu6050)->load_calibration_data();
  });
}
