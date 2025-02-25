#pragma once
#include <cstdint>
#include <ctime>

enum class DataType : uint8_t
{
  STRING = 1,
  INT8 = 2,
  UINT8 = 3,
  INT16 = 4,
  UINT16 = 5,
  INT32 = 6,
  UINT32 = 7,
  INT64 = 8,
  UINT64 = 9,
  FLOAT = 10,
  DOUBLE = 11,
  TIME = 12
};

namespace data_type_helpers
{
  inline size_t get_size_for_data_type(const DataType type)
  {
    switch (type)
    {
    case DataType::INT8:
      return sizeof(int8_t);
    case DataType::UINT8:
      return sizeof(uint8_t);
    case DataType::INT16:
      return sizeof(int16_t);
    case DataType::UINT16:
      return sizeof(uint16_t);
    case DataType::INT32:
      return sizeof(int32_t);
    case DataType::UINT32:
      return sizeof(uint32_t);
    case DataType::INT64:
      return sizeof(int64_t);
    case DataType::UINT64:
      return sizeof(uint64_t);
    case DataType::FLOAT:
      return sizeof(float);
    case DataType::DOUBLE:
      return sizeof(double);
    case DataType::TIME:
      return sizeof(tm);
    case DataType::STRING:
    default:
      return 0;
    }
  }
}
