#include "dev/riscv/spike.hh"

#include "dev/riscv/clint.hh"
#include "dev/riscv/htif.hh"
#include "dev/riscv/plic.hh"
#include "params/Spike.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace RiscvISA;

Spike::Spike(const Params &params) :
    Platform(params),
    clint(params.clint), htif(params.htif)
{
}

void
Spike::postConsoleInt()
{
}

void
Spike::clearConsoleInt()
{
}

void
Spike::postPciInt(int line)
{
}

void
Spike::clearPciInt(int line)
{
}

void
Spike::serialize(CheckpointOut &cp) const
{
}

void
Spike::unserialize(CheckpointIn &cp)
{
}

} // namespace gem5
