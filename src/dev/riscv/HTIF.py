from m5.objects.Device import BasicPioDevice
from m5.params import *
from m5.proxy import *

class HTIF(BasicPioDevice):
    type = 'HTIF'
    cxx_header = 'dev/riscv/htif.hh'
    cxx_class = 'gem5::HTIF'
    pio_size = Param.Addr(0x1000, "PIO Size")

    def generateDeviceTree(self, state):
        node = self.generateBasicPioDeviceNode(state, "HTIF", self.pio_addr,
                                               self.pio_size)
        node.appendCompatible(["ucb,htif0"])
        yield node
