
#ifndef __CPU_MEM_UNIT_DEFINES__
#define __CPU_MEM_UNIT_DEFINES__

enum class MopType {
  unit_stride = 0,
  indexed_unordered = 1,
  strided = 2,
  indexed_ordered = 3
};

enum class LumopType {
  unit_stride_load = 0,
  whole_reg = 0x8,
  mask = 0xb,
  fault_only_first = 0x10
};

enum class SumopType {
  unit_stride_load = 0,
  whole_reg = 0x8,
  mask = 0xb
};

enum class Location {
  mem = 0,
  vector_rf = 1
};

#endif