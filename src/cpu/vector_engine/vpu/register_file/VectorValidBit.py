
from m5.params import *

from TickedObject import TickedObject

class VectorValidBit(TickedObject):
    type = 'VectorValidBit'
    cxx_header = "cpu/vector_engine/vpu/register_file/vector_reg_valid_bit.hh"

    PhysicalRegs = Param.Unsigned("Number of Vector Physical Registers")