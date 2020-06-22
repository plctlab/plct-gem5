
from m5.params import *

from ClockedObject import *

class VectorRegister(ClockedObject):
    type = 'VectorRegister'
    cxx_header = "cpu/vector_engine/vpu/register_file/vector_reg.hh"

    lanes_per_access = Param.Unsigned("how many lanes are requesting a r/w")
    size = Param.Unsigned("Size of vector_reg in Bytes")
    lineSize = Param.Unsigned("how many bytes accessed per cycle")
    numPorts = Param.Unsigned("Independent Ports per vector_reg")
    accessLatency = Param.Cycles("Read/Write Latency to vector_reg")

    port = VectorSlavePort("Ports for other units to access the vector_reg")