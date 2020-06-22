
from m5.params import *

from m5.SimObject import SimObject

class VectorMemUnit(SimObject):
    type = 'VectorMemUnit'
    cxx_header = "cpu/vector_engine/vmu/vector_mem_unit.hh"

    memReader = Param.MemUnitReadTiming("read streaming submodule")
    memReader_addr = Param.MemUnitReadTiming("read streaming submodule")
    memWriter = Param.MemUnitWriteTiming("write streaming submodule")