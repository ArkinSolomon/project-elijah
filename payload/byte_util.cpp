#include "byte_util.h"
#include <cmath>
#include <cstring>
#include <pico/mem_ops.h>

void byte_util::encode_int16(const int16_t data, uint8_t* output, uint8_t& sign, const uint8_t sign_bit)
{
  encode_sign(data < 0, sign, sign_bit);

  const uint16_t absolute_value = abs(data);
  encode_uint16(absolute_value, output);
}

void byte_util::encode_uint16(const uint16_t data, uint8_t* output)
{
  output[0] = data >> 8;
  output[1] = data & 0xFF;
}

void byte_util::encode_int32(const int32_t data, uint8_t* output, uint8_t& sign, const uint8_t sign_bit)
{
  encode_sign(data < 0, sign, sign_bit);

  const uint32_t absolute_value = abs(data);
  encode_uint32(absolute_value, output);
}

void byte_util::encode_uint32(const uint32_t data, uint8_t* output)
{
  output[0] = data >> 0x18;
  output[1] = data >> 0x10;
  output[2] = data >> 0x08;
  output[3] = data & 0xFF;
}

void byte_util::encode_int64(const int64_t data, uint8_t* output, uint8_t& sign, uint8_t sign_bit)
{
  encode_sign(data < 0, sign, sign_bit);

  const uint64_t absolute_value = llabs(data);
  encode_uint64(absolute_value, output);
}

void byte_util::encode_uint64(const uint64_t data, uint8_t* output)
{
  output[0] = data >> 0x38;
  output[1] = data >> 0x30;
  output[2] = data >> 0x28;
  output[3] = data >> 0x20;
  output[4] = data >> 0x18;
  output[5] = data >> 0x10;
  output[6] = data >> 0x08;
  output[7] = data & 0xFF;
}

void byte_util::encode_double(const double data, uint8_t* output)
{
  memcpy(output, &data, 8);
}

void byte_util::encode_sign(const bool sign_set, uint8_t& sign, const uint8_t sign_bit)
{
  const uint8_t sign_mask = ~(1 << sign_bit);
  sign &= sign_mask;
  if (sign_set)
  {
    sign |= 1 << sign_bit;
  }
}

uint32_t byte_util::decode_uint32(const uint8_t* input)
{
  const uint32_t result = static_cast<uint32_t>(input[0]) << 0x18
    | static_cast<uint32_t>(input[1]) << 0x10
    | static_cast<uint32_t>(input[2]) << 0x08
    | static_cast<uint32_t>(input[3]);
  return result;
}

uint64_t byte_util::decode_uint64(const uint8_t* output)
{
  const uint64_t result = static_cast<uint64_t>(output[0]) << 0x38
  | static_cast<uint64_t>(output[1]) << 0x30
  | static_cast<uint64_t>(output[2]) << 0x28
  | static_cast<uint64_t>(output[3]) << 0x20
  | static_cast<uint64_t>(output[4]) << 0x18
  | static_cast<uint64_t>(output[5]) << 0x10
  | static_cast<uint64_t>(output[6]) << 0x08
  | static_cast<uint64_t>(output[7]);
  return result;
}

double byte_util::decode_double(const uint8_t* data)
{
  double output;
  memcpy(&output, data, 8);
  return output;
}
