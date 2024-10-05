#include "mpu_6050.h"

#include <format>

#include "src/pin_outs.h"
#include "src/status_manager.h"
#include "src/usb_communication.h"
#include "src/sensors/i2c/i2c_util.h"

/**
 * Check if the chip is detected.
 *
 * @return True if the device is detected and the device id reads 0x68.
 */
bool mpu_6050::check_chip_id()
{
  uint8_t read_id;
  const bool success = i2c_util::read_ubyte(I2C_BUS1, MPU_6050_ADDR, _reg_defs::REG_WHO_AM_I, read_id);
  usb_communication::send_string(std::format("mpu_6050::check_chip_id: {:02x}", read_id));
  return success && read_id == MPU_6050_DEVICE_ID;
}

void mpu_6050::accel_loop(CollectionData& collection_data)
{
  static bool device_detected = false;

  if (!device_detected)
  {
    device_detected = check_chip_id();
    if (!device_detected)
    {
      set_fault(status_manager::DEVICE_MPU_6050, true);
      usb_communication::send_string("Fault: MPU 6050, device not detected");
      return;
    }
  }

  usb_communication::send_string("MPU 6050 detected");
  set_fault(status_manager::DEVICE_MPU_6050, false);
}
