
from m5.params import *

from m5.SimObject import SimObject

class VectorRename(SimObject):
    type = 'VectorRename'
    cxx_header = "cpu/vector_engine/vpu/rename/vector_rename.hh"

    PhysicalRegs = Param.Unsigned("Number of Vector Physical Registers")