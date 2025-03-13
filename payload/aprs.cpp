#include "aprs.h"

#include <cmath>
#include <cstring>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/structs/io_bank0.h>
#include <pico/time.h>

#include "pin_outs.h"

std::unique_ptr<uint8_t> aprs::encode_aprs_string_message(const std::string& callsign, const std::string& message,
                                                          size_t& message_len)
{
  message_len = 7 /* Destination */ + callsign.length() + message.length();
  std::unique_ptr<uint8_t> data(new uint8_t[message_len]);

  const std::string destination = "TLMXXX0";
  memcpy(data.get(), destination.c_str(), 7);

  strcpy(reinterpret_cast<char*>(data.get() + 7), callsign.c_str());
  *reinterpret_cast<char*>(data.get() + 7 + callsign.length()) = '0';

  return data;
}

void aprs::init_aprs_system(const uint32_t clock_speed_khz, const uint8_t pwm_gpio, const pwm_chan channel)
{
  gpio_set_function(pwm_gpio, GPIO_FUNC_PWM);

  pwm_slice_num = pwm_gpio_to_slice_num(pwm_gpio);
  pwm_channel = channel;

  pwm_set_clkdiv(pwm_slice_num, 255);

  pwm_freq = clock_speed_khz * 1000.0 / 255.0;
  pwm_set_wrap(pwm_slice_num, 2);
  pwm_set_chan_level(pwm_slice_num, channel, 1);
  pwm_set_enabled(pwm_slice_num, true);
}

void aprs::transmit_bytes(const uint8_t* data, const size_t len, const size_t flag_count)
{
  bool run_once = false;

  size_t ones_count = 0;

  bool last_bit = true;
  uint64_t pwm_period = 0;

  for (size_t i = 0; i < len + flag_count * 2; i++)
  {
    uint8_t curr_byte = data[i];
    bool should_bit_stuff = true;
    if (i < flag_count || i > flag_count + len)
    {
      should_bit_stuff = false;
      curr_byte = APRS_FRAME_FLAG;
    }

    for (size_t j = 0; j < 8; j++)
    {
      bool curr_bit = (curr_byte & 1 << 7 - j) > 0;
      if (should_bit_stuff && curr_bit)
      {
        ones_count++;
        if (ones_count == 6)
        {
          curr_bit = false;
          j--;
          ones_count = 0;
        }
      }
      else
      {
        ones_count = 0;
      }

      if (!run_once || last_bit != curr_bit)
      {
        run_once = true;
        const uint target_hz = curr_bit ? 1200 : 2200;
        pwm_period = static_cast<uint64_t>(round(1.0 / target_hz * 1000.0 * 1000.0));

        const auto wrap_num = static_cast<uint16_t>(pwm_freq / target_hz);

        pwm_set_wrap(pwm_slice_num, wrap_num);
        pwm_set_chan_level(pwm_slice_num, pwm_channel, wrap_num / 2);

        last_bit = curr_bit;
      }

      sleep_us(pwm_period);
    }
  }
}
