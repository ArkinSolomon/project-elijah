#include "main.h"

#include <cstdio>
#include <cstring>
#include <hardware/clocks.h>

#include "core_1.h"
#include "pin_outs.h"
#include "status_manager.h"
#include "usb_communication.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "sensors/bmp_180/bmp_180.h"
#include "sensors/ds_1307/ds_1307.h"
#include "sensors/i2c/i2c_util.h"

int main()
{
  set_sys_clock_khz(133000, true);
  pin_init();
  usb_communication::init_usb_com();
  status_manager::status_manager_pio_init();
  launch_core_1();

  CollectionData collection_data{{}};
  while (true)
  {
    clock_loop(collection_data);
    pressure_loop(collection_data);

    if (stdio_usb_connected())
    {
      if (status_manager::get_current_status() != status_manager::USB)
      {
        usb_communication::say_hello();
      }

      set_status(status_manager::USB);
      usb_communication::scan_for_packets();
      usb_communication::send_collection_data(collection_data);
    }
    else if (status_manager::get_current_status() == status_manager::BOOTING || (status_manager::get_current_status() ==
      status_manager::USB && !stdio_usb_connected()))
    {
      set_status(status_manager::NORMAL);
    }

    const double alt_ft = collection_data.altitude * 3.2808;
    const double temp_f = collection_data.temperature * (static_cast<double>(9) / 5) + 32;

    // const float conversion_factor = 3.3f / 0x1000;
    // uint16_t adc_raw = adc_read();
    // float voltage = adc_raw * conversion_factor;
    // float t = 27 - (voltage - 0.706) / 0.001721;

    // usb_communication::send_string(
    //   "[%02d/%02d/%04d %02d:%02d:%02d] temp (C): %.1f temp: (F): %.2f press (Pa): %d, altitude (m): %.3f altitude (ft): %.3f\n",
    //   collection_data.time_inst.month, collection_data.time_inst.date, collection_data.time_inst.year,
    //   collection_data.time_inst.hours, collection_data.time_inst.minutes, collection_data.time_inst.seconds,
    //   collection_data.temperature, temp_f, collection_data.pressure, collection_data.altitude, alt_ft);
  }
}

void pin_init()
{
  i2c_util::i2c_init(I2C_BUS, I2C_SDA_PIN, I2C_SCL_PIN);

  gpio_init(TEMP_LED_PIN);
  gpio_set_dir(TEMP_LED_PIN, true);

  adc_init();
  adc_select_input(ONBOARD_TEMP_PIN);
}

void clock_loop(CollectionData& collection_data)
{
  static bool clock_detected = false, clock_set = false;

  if (!clock_detected || !clock_set)
  {
    clock_detected = ds_1307::check_clock(clock_set);
    if (!clock_detected)
    {
      usb_communication::send_string("Fault: DS 1307 [NO_DETECT]");
      set_fault(status_manager::fault_id::DEVICE_DS_1307, true);
      return;
    }

    if (!clock_set)
    {
      usb_communication::send_string("Fault: DS 1307 [NOT_SET]");
      set_fault(status_manager::fault_id::DEVICE_DS_1307, true);

      if (!clock_detected)
      {
        clock_set = false;
        usb_communication::send_string("Fault: DS 1307 [NO_DETECT_AFTER_SET]");
        set_fault(status_manager::fault_id::DEVICE_DS_1307, true);
      }
      return;
    }
  }

  clock_detected = get_time_instance(collection_data.time_inst);
  if (!clock_detected)
  {
    load_blank_inst(collection_data.time_inst);
    return;
  }

  set_fault(status_manager::fault_id::DEVICE_DS_1307, false);
}

void pressure_loop(CollectionData& collection_data)
{
  static bool bmp_180_calib_received = false;

  if (!bmp_180_calib_received)
  {
    const bool device_detected = bmp_180::check_device_id();
    if (!device_detected || !bmp_180::read_calibration_data())
    {
      usb_communication::send_string("Fault: BMP 180 [ND_OR_RCD_FAIL]");
      set_fault(status_manager::fault_id::DEVICE_BMP_180, true);
      collection_data.pressure = -1;
      collection_data.temperature = collection_data.altitude = -1;
      bmp_180_calib_received = false;
      return;
    }

    // bmp_180_calib_data.AC1 = 408;
    // bmp_180_calib_data.AC2 = -72;
    // bmp_180_calib_data.AC3 = -14383;
    // bmp_180_calib_data.AC4 = 32741;
    // bmp_180_calib_data.AC5 = 32757;
    // bmp_180_calib_data.AC6 = 23153;
    // bmp_180_calib_data.B1 = 6190;
    // bmp_180_calib_data.B2 = 4;
    // bmp_180_calib_data.MB = -32768;
    // bmp_180_calib_data.MC = -8711;
    // bmp_180_calib_data.MD = 2868;

    bmp_180_calib_received = true;
  }

  const bool success = read_press_temp_alt(bmp_180::oss_setting::ULTRA_HIGH, collection_data.temperature,
                                           collection_data.pressure, collection_data.altitude);
  collection_data.temperature = 10.5;
  if (!success)
  {
    usb_communication::send_string("Fault: BMP 180 [READ_FAIL]");
    set_fault(status_manager::fault_id::DEVICE_BMP_180, true);
    bmp_180_calib_received = false;
    collection_data.pressure = -1;
    collection_data.temperature = collection_data.altitude = -1;
    return;
  }

  set_fault(status_manager::fault_id::DEVICE_BMP_180, false);
}
