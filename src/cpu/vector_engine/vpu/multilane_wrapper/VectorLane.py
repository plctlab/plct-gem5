
from m5.params import *

from m5.SimObject import SimObject

class VectorLane(SimObject):
    type = 'VectorLane'
    cxx_header = "cpu/vector_engine/vpu/multilane_wrapper/vector_lane.hh"

    dataPath = Param.Datapath("This unit's datapath")

    lane_id     = Param.Unsigned("Cluster ID")
    srcAReader  = Param.MemUnitReadTiming("srcA read streaming submodule")
    srcBReader  = Param.MemUnitReadTiming("srcB read streaming submodule")
    srcMReader  = Param.MemUnitReadTiming("srcM read streaming submodule")
    dstReader   = Param.MemUnitReadTiming("srcC read streaming submodule")
    dstWriter   = Param.MemUnitWriteTiming("dst write streaming submodule")