#ifndef __DEV_RISCV_SPIKE_HH__
#define __DEV_RISCV_SPIKE_HH__

#include "dev/platform.hh"
#include "dev/riscv/clint.hh"
#include "dev/riscv/htif.hh"
#include "params/Spike.hh"

namespace gem5
{

using namespace RiscvISA;

class Spike : public Platform
{
  public:
    Clint *clint;
    HTIF *htif;

  public:
    typedef SpikeParams Params;
    Spike(const Params &params);

    void postConsoleInt() override;

    void clearConsoleInt() override;

    void postPciInt(int line) override;

    void clearPciInt(int line) override;

    void serialize(CheckpointOut &cp) const override;

    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif  // __DEV_RISCV_HIFIVE_HH__
