
from m5.params import *

from TickedObject import TickedObject

class ReorderBuffer(TickedObject):
    type = 'ReorderBuffer'
    cxx_header = "cpu/vector_engine/vpu/rob/reorder_buffer.hh"

    ROB_Size = Param.Unsigned("Number of ROB entries")