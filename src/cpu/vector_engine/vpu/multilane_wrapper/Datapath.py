
from m5.params import *

from TickedObject import TickedObject

class Datapath(TickedObject):
    type = 'Datapath'
    cxx_header = "cpu/vector_engine/vpu/multilane_wrapper/datapath.hh"

    VectorLanes = Param.Unsigned("Number of Vector Lanes")


