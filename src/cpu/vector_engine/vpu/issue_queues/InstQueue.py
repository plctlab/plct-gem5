
from m5.params import *

from TickedObject import TickedObject

class InstQueue(TickedObject):
    type = 'InstQueue'
    cxx_header = "cpu/vector_engine/vpu/issue_queues/inst_queue.hh"

    OoO_queues = Param.Bool("Out-of-Order/In-Order Queues")
    vector_mem_queue_size = Param.Unsigned("memory queue entries")
    vector_arith_queue_size = Param.Unsigned("arithmetic queue entries")