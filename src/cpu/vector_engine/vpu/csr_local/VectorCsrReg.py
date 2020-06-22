
from m5.params import *

from m5.SimObject import SimObject

class VectorCsrReg(SimObject):
    type = 'VectorCsrReg'
    cxx_header = "cpu/vector_engine/vpu/csr_local/vector_csr.hh"

    max_vl 	= Param.Unsigned("Maximun VL")