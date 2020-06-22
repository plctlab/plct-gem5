
from m5.params import *
from TickedObject import TickedObject

class MemUnitWriteTiming(TickedObject):
    type = 'MemUnitWriteTiming'
    cxx_header = "cpu/vector_engine/vmu/write_timing_unit.hh"

    channel = Param.Unsigned("Channel")
    cacheLineSize = Param.Unsigned("Cache Line Size")
    VRF_LineSize = Param.Unsigned("Reg Line Size")
