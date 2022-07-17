#include "dev/riscv/htif.hh"

#include "debug/HTIF.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace RiscvISA;

HTIF::HTIF(const Params &params) :
    BasicPioDevice(params, params.pio_size),
    tohost(0)
{
}

Tick
HTIF::read(PacketPtr pkt)
{
    pkt->setLE<uint32_t>(0);
    pkt->makeResponse();
    return pioDelay;
}

Tick
HTIF::write(PacketPtr pkt)
{
    if (pkt->getSize() != 4) {
        pkt->makeResponse();
        return pioDelay;
    }

    Addr offset = pkt->getAddr() - pioAddr;
    uint32_t value = pkt->getLE<uint32_t>();
    if (offset == 0x0) {
        tohost = value;

        uint8_t device = tohost >> 56;
        uint8_t cmd = tohost >> 48;
        uint64_t payload = tohost & 0xFFFFFFFFFFFFULL;
        if (device == 0x0) {
            if (cmd == 0x0) {
                if (payload & 0x1) {
                    int exit_code = payload >> 1;
                    DPRINTF(HTIF, "HTIF exit (exit code = %d)\n", exit_code);
                    exit(exit_code);
                }
            }
        }

        tohost = 0;
    }
    pkt->makeResponse();
    return pioDelay;
}

} // namespace gem5