#pragma once
#include <cstdint>

namespace byte_util
{
  void encode_int16(int16_t data, uint8_t* output, uint8_t& sign, uint8_t sign_bit);
  void encode_uint16(uint16_t data, uint8_t* output);
  void encode_int32(int32_t data, uint8_t* output, uint8_t& sign, uint8_t sign_bit);
  void encode_uint32(uint32_t data, uint8_t* output);
  void encode_int64(int64_t data, uint8_t* output, uint8_t& sign, uint8_t sign_bit);
  void encode_uint64(uint64_t data, uint8_t* output);
  void encode_double(double data, uint8_t* output);
  void encode_sign(bool sign_set, uint8_t& sign, uint8_t sign_bit);

  uint32_t decode_uint32(const uint8_t* input);
  double decode_double(const uint8_t* data);
}
