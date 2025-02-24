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
#define DATA_TYPE_SIZE_CASE(TYPE, CPP_TYPE) case DataType::TYPE: return sizeof(CPP_TYPE)
  inline size_t get_size_for_data_type(const DataType type)
  {
    switch (type)
    {
      DATA_TYPE_SIZE_CASE(INT8, int8_t);
      DATA_TYPE_SIZE_CASE(UINT8, uint8_t);
      DATA_TYPE_SIZE_CASE(INT16, int16_t);
      DATA_TYPE_SIZE_CASE(UINT16, uint16_t);
      DATA_TYPE_SIZE_CASE(INT32, int32_t);
      DATA_TYPE_SIZE_CASE(UINT32, uint32_t);
      DATA_TYPE_SIZE_CASE(INT64, int64_t);
      DATA_TYPE_SIZE_CASE(UINT64, uint64_t);
      DATA_TYPE_SIZE_CASE(FLOAT, float);
      DATA_TYPE_SIZE_CASE(DOUBLE, double);
      DATA_TYPE_SIZE_CASE(TIME, tm);
    case DataType::STRING:
    default:
      return 0;
    }
  }
}