#include "hmc_5883l.h"

#include <format>

#include "src/pin_outs.h"
#include "src/status_manager.h"
#include "src/usb_communication.h"
#include "src/sensors/i2c/i2c_util.h"

bool hmc_5883l::check_device_id()
{
  uint8_t read_data[3];
  const bool success = i2c_util::read_bytes(I2C_BUS0, 0x3D, _reg_defs::REG_IDENT_A,
                                            read_data, 3);
  if (!success)
  {
    return false;
  }

  uint32_t read_id = read_data[0] << 16 | read_data[1] << 8 | read_data[2];
  usb_communication::send_string(std::format("HMC5883L Device ID: 0x{:06x}", read_id));
  return read_id == HMC_5883L_DEVICE_ID;
}

bool hmc_5883l::configure(samples_averaged samples_averaged, data_output_rate data_output_rate,
                          measurement_mode measurement_mode, gain_setting gain_configuration, const bool isHighSpeedI2c,
                          operating_mode operating_mode)
{
  const uint8_t config_a = static_cast<uint8_t>(samples_averaged) << 5 | static_cast<uint8_t>(data_output_rate) << 3 |
    static_cast<uint8_t>(measurement_mode);
  const uint8_t config_b = static_cast<uint8_t>(gain_configuration) << 5;
  const uint8_t mode_data = (isHighSpeedI2c ? 0x80 : 0x00) | static_cast<uint8_t>(operating_mode);

  const uint8_t write_data[4] = {_reg_defs::REG_CONFIG_A, config_a, config_b, mode_data};
  const int success = i2c_write_blocking_until(I2C_BUS0, HMC_5883L_ADDR, write_data, 4, false,
                                               delayed_by_ms(get_absolute_time(), 100));

  return success == 4;
}

void hmc_5883l::accel_loop(CollectionData& collection_data)
{
  static bool is_configured = false;
  if (!check_device_id())
  {
    set_fault(status_manager::DEVICE_HMC_5883L, true);
    usb_communication::send_string("Fault: HMC 5883L, device not detected");
    return;
  }

  if (!is_configured)
  {
    is_configured = configure(samples_averaged::FOUR_SAMPLES, data_output_rate::OUTPUT_RATE_75Hz,
                              measurement_mode::NORMAL, gain_setting::GAIN_1090_GAUSS, false,
                              operating_mode::SINGLE_MEASUREMENT);

    if (!is_configured)
    {
      set_fault(status_manager::DEVICE_HMC_5883L, true);
      usb_communication::send_string("Fault: HMC 5883L, unable to configure");
      return;
    }
  }

  // if (!gpio_get(HMC_5883L_RDY_PIN))
  // {
  //   set_fault(status_manager::DEVICE_HMC_5883L, false);
  //   return;
  // }

  set_fault(status_manager::DEVICE_HMC_5883L, false);
}
