#pragma once
#include <cstdint>
#include <ctime>

enum class DataType : uint8_t
{
  String = 1,
  Int8 = 2,
  Uint8 = 3,
  Int16 = 4,
  UInt16 = 5,
  Int32 = 6,
  UInt32 = 7,
  Int64 = 8,
  UInt64 = 9,
  Float = 10,
  Double = 11,
  Time = 12
};

namespace data_type_helpers
{
  inline size_t get_size_for_data_type(const DataType type)
  {
    switch (type)
    {
    case DataType::Int8:
      return sizeof(int8_t);
    case DataType::Uint8:
      return sizeof(uint8_t);
    case DataType::Int16:
      return sizeof(int16_t);
    case DataType::UInt16:
      return sizeof(uint16_t);
    case DataType::Int32:
      return sizeof(int32_t);
    case DataType::UInt32:
      return sizeof(uint32_t);
    case DataType::Int64:
      return sizeof(int64_t);
    case DataType::UInt64:
      return sizeof(uint64_t);
    case DataType::Float:
      return sizeof(float);
    case DataType::Double:
      return sizeof(double);
    case DataType::Time:
      return 8;
    case DataType::String:
    default:
      return 0;
    }
  }
}
