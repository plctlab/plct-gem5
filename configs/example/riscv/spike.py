import argparse

from m5.objects import *
from m5.util import addToPath
from m5.util.fdthelper import *

addToPath("../../")

from common.FSConfig import *
from common.SysPaths import *
from common.Benchmarks import *
from common import Simulation
from common import CacheConfig
from common import MemConfig
from common import ObjectList
from common.Caches import *
from common import Options

# ----------------------------- Add Options ---------------------------- #
parser = argparse.ArgumentParser()
Options.addCommonOptions(parser)
Options.addFSOptions(parser)
args = parser.parse_args()

# CPU and Memory
(CPUClass, mem_mode, FutureClass) = Simulation.setCPUClass(args)
MemClass = Simulation.setMemClass(args)

np = args.num_cpus

# ---------------------------- Setup System ---------------------------- #
# Default Setup
system = System()
system.mem_mode = mem_mode

# TODO: 80001000 -> 80002000 is the HTIF mem region, it's not fixed and
# should be calculated using ELF symbols.
size_remain = int(Addr('128MB')) - 0x2000
system.mem_ranges = [
    AddrRange(start=0x80000000, end=0x80001000),
    AddrRange(start=0x80002000, size=size_remain),
]

system.workload = RiscvBareMetal()
system.workload.bootloader = args.kernel

system.iobus = IOXBar()
system.membus = MemBus()

system.system_port = system.membus.cpu_side_ports

system.platform = Spike()

# TODO: Should be calculated using ELF symbols.
system.platform.htif.pio_addr = 0x80001000

system.platform.rtc = RiscvRTC(frequency=Frequency("100MHz"))
system.platform.clint.int_pin = system.platform.rtc.int_pin

system.bridge = Bridge(delay="50ns")
system.bridge.mem_side_port = system.iobus.cpu_side_ports
system.bridge.cpu_side_port = system.membus.mem_side_ports
system.bridge.ranges = system.platform._off_chip_ranges()

system.platform.attachOnChipIO(system.membus)
system.platform.attachOffChipIO(system.iobus)
system.platform.attachPlic()
system.platform.setNumCores(np)

system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)
system.clk_domain = SrcClockDomain(
    clock=args.sys_clock,
    voltage_domain=system.voltage_domain,
)
system.cpu_voltage_domain = VoltageDomain()
system.cpu_clk_domain = SrcClockDomain(
    clock=args.cpu_clock,
    voltage_domain=system.cpu_voltage_domain,
)

system.cpu = [
    CPUClass(clk_domain=system.cpu_clk_domain, cpu_id=i) for i in range(np)
]

uncacheable_range = [
    *system.platform._on_chip_ranges(),
    *system.platform._off_chip_ranges(),
]

for i in range(np):
    system.cpu[i].createThreads()
    system.cpu[i].mmu.pma_checker = PMAChecker(uncacheable=uncacheable_range)

CacheConfig.config_cache(args, system)
MemConfig.config_mem(args, system)

root = Root(full_system=True, system=system)

Simulation.setWorkCountOptions(system, args)
Simulation.run(args, root, system, FutureClass)
