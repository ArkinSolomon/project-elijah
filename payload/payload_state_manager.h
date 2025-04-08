#pragma once

#include "battery.h"
#include "elijah_state_framework.h"
#include "data_type.h"
#include "mpu_6050.h"
#include "payload_flight_phase_controller.h"
#include "standard_command_helpers.h"
#include "reliable_clock/reliable_clock.h"
#include "sensors/ds_1307/ds_1307.h"

#define PAYLOAD_STATE_TEMPLATE_TYPES \
PayloadState, \
PayloadPersistentKey, \
PayloadFaultKey, \
elijah_state_framework::std_helpers::StandardFlightPhase, \
PayloadFlightPhaseController

#define DS_1307_TEST_REG 0x37

struct PayloadState
{
  tm time_inst;
  STANDARD_STATE_COLLECTION_DATA
  double bat_voltage, bat_percent;
};

enum class PayloadPersistentKey : uint8_t
{
  STANDARD_PERSISTENT_KEYS
};

enum class PayloadFaultKey : uint8_t
{
  BMP280 = 1,
  MPU6050 = 2,
  MicroSD = 3,
  OnboardClock = 4,
  DS1307 = 5
};

extern ReliableBMP280<PAYLOAD_STATE_TEMPLATE_TYPES>* bmp280;
extern ReliableMPU6050<PAYLOAD_STATE_TEMPLATE_TYPES>* mpu6050;
extern Battery* battery;
extern ReliableClock* reliable_clock;

class PayloadStateManager final : public elijah_state_framework::ElijahStateFramework<PAYLOAD_STATE_TEMPLATE_TYPES>
{
public:
  PayloadStateManager(): ElijahStateFramework("Payload", 100)
  {
    REGISTER_STANDARD_KEYS(PayloadPersistentKey)
    get_persistent_storage()->finish_registration();

    register_fault(PayloadFaultKey::BMP280, "BMP 280", CommunicationChannel::SPI_1);
    register_fault(PayloadFaultKey::MPU6050, "MPU 6050", CommunicationChannel::I2C_1);
    register_fault(PayloadFaultKey::MicroSD, "MicroSD", CommunicationChannel::SPI_0);
    register_fault(PayloadFaultKey::OnboardClock, "Onboard Clock", CommunicationChannel::None);
    register_fault(PayloadFaultKey::DS1307, "DS 1307", CommunicationChannel::I2C_0);

    StdCommandRegistrationHelpers::register_calibration_command(this, &bmp280, &mpu6050, 0, GRAVITY_CONSTANT, 0);
    StdCommandRegistrationHelpers::register_persistent_storage_reset_helper(this, &mpu6050);

    register_command("Update clock", "Time", [this](tm time_inst)
    {
      // Because the FAT lib wants this idk man
      time_inst.tm_year -= 80;
      reliable_clock->set_clock(time_inst);
    });

    register_command("DS 1307 dump", [this]
    {
      reliable_clock->get_ds_1307().reg_dump();
    });

    register_command("DS 1307 erase", [this]
    {
      reliable_clock->get_ds_1307().erase_data();
    });

    register_command(std::format("DS 1307 0xAA to 0x{:02X}", DS_1307_TEST_REG), [this]
    {
      constexpr uint8_t test_data = 0xAA;
      reliable_clock->get_ds_1307().write_custom_register(DS_1307_TEST_REG, &test_data, 1);
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
