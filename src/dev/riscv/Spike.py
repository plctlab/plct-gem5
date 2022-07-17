from m5.objects.Platform import Platform
from m5.objects.Clint import Clint
from m5.objects.HTIF import HTIF
from m5.objects.Terminal import Terminal
from m5.params import *
from m5.proxy import *
from m5.util.fdthelper import *

class Spike(Platform):
    type = 'Spike'
    cxx_header = "dev/riscv/spike.hh"
    cxx_class = 'gem5::Spike'

    clint = Param.Clint(Clint(pio_addr=0x2000000), "CLINT")
    htif = Param.HTIF(HTIF(pio_addr=0x1000000), "HTIF")

    terminal = Terminal()

    def _on_chip_devices(self):
        """Returns a list of on-chip peripherals
        """
        return [self.clint]

    def _off_chip_devices(self):
        """Returns a list of off-chip peripherals
        """
        return [self.htif]

    def _on_chip_ranges(self):
        """Returns a list of on-chip peripherals
            address range
        """
        return [
            AddrRange(dev.pio_addr, size=dev.pio_size)
            for dev in self._on_chip_devices()
        ]

    def _off_chip_ranges(self):
        """Returns a list of off-chip peripherals
            address range
        """
        return [
            AddrRange(dev.pio_addr, size=dev.pio_size)
            for dev in self._off_chip_devices()
        ]

    def attachPlic(self):
        """Count number of PLIC interrupt sources
        """

    def attachOnChipIO(self, bus):
        """Attach on-chip IO devices, needs modification
            to support DMA
        """
        for device in self._on_chip_devices():
            device.pio = bus.mem_side_ports

    def attachOffChipIO(self, bus):
        """Attach off-chip IO devices, needs modification
            to support DMA
        """
        for device in self._off_chip_devices():
            device.pio = bus.mem_side_ports

    def setNumCores(self, num_cpu):
        """ Sets the CLINT to have the right number of threads.
            Assumes that the cores have a single hardware thread.
        """
        self.clint.num_threads = num_cpu

    def generateDeviceTree(self, state):
        cpus_node = FdtNode("cpus")
        cpus_node.append(FdtPropertyWords("timebase-frequency", [10000000]))
        yield cpus_node

        node = FdtNode("soc")
        local_state = FdtState(addr_cells=2, size_cells=2)
        node.append(local_state.addrCellsProperty())
        node.append(local_state.sizeCellsProperty())
        node.append(FdtProperty("ranges"))
        node.appendCompatible(["simple-bus"])

        for subnode in self.recurseDeviceTree(local_state):
            node.append(subnode)

        yield node

    # For generating devicetree
    _cpu_count = 0
    def annotateCpuDeviceNode(self, cpu, state):
        cpu.append(FdtPropertyStrings('mmu-type', 'riscv,sv48'))
        cpu.append(FdtPropertyStrings('status', 'okay'))
        cpu.append(FdtPropertyStrings('riscv,isa', 'rv64imafdcsu'))
        cpu.appendCompatible(["riscv"])

        int_node = FdtNode("interrupt-controller")
        int_state = FdtState(interrupt_cells=1)
        int_node.append(int_state.interruptCellsProperty())
        int_node.append(FdtProperty("interrupt-controller"))
        int_node.appendCompatible("riscv,cpu-intc")

        cpus = self.system.unproxy(self).cpu
        phandle = int_state.phandle(cpus[self._cpu_count])
        self._cpu_count += 1
        int_node.append(FdtPropertyWords("phandle", [phandle]))

        cpu.append(int_node)
