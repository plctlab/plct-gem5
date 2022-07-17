#ifndef __DEV_RISCV_HTIF_HH__
#define __DEV_RISCV_HTIF_HH__

#include "arch/riscv/interrupts.hh"
#include "dev/io_device.hh"
#include "params/HTIF.hh"

namespace gem5
{

using namespace RiscvISA;

class HTIF : public BasicPioDevice
{
protected:
    uint64_t tohost;
public:
    typedef HTIFParams Params;
    HTIF(const Params &params);

public:
    /**
     * PioDevice interface functions
    */
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
};

} // namespace gem5

#endif // __DEV_RISCV_HTIF_HH__