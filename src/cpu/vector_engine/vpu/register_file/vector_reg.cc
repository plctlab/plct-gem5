/*
 * Copyright (c) 2020 Barcelona Supercomputing Center
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Cristóbal Ramírez
 */

#include "cpu/vector_engine/vpu/register_file/vector_reg.hh"

#include <cassert>
#include <cstdint>
#include <iomanip>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "debug/VectorRegister.hh"

// vector_reg::VectorRegisterPort
VectorRegister::VectorRegisterPort::VectorRegisterPort(
    const std::string &name, VectorRegister& vector_reg) :
    QueuedSlavePort(name, &vector_reg, queue), queue(vector_reg, *this),
    vector_reg(vector_reg)
{
}

Tick
VectorRegister::VectorRegisterPort::recvAtomic(PacketPtr pkt)
{
    panic("VectorRegisterPort::recvAtomic called\n");
}

void
VectorRegister::VectorRegisterPort::recvFunctional(PacketPtr pkt)
{
    panic("VectorRegisterPort::recvFunctional called\n");
}

bool
VectorRegister::VectorRegisterPort::recvTimingReq(PacketPtr pkt)
{
    assert(pkt->isRead() || pkt->isWrite());
    assert(pkt->isRequest());
    assert(pkt->needsResponse());

    return vector_reg.handleTimingReq(pkt, this);
}

AddrRangeList
VectorRegister::VectorRegisterPort::getAddrRanges() const
{
    panic("VectorRegisterPort::getAddrRanges called\n");
}

void
VectorRegister::regStats()
{
    ClockedObject::regStats();

    numReads_64bit_elements
        .name(name() + ".numReads_64bit_elements")
        .desc("Number of reads to the vrf, 64-bit elements");
    numWritess_64bit_elements
        .name(name() + ".numWritess_64bit_elements")
        .desc("Number of writes to the vrf, 64-bit elements");
    numReads_perLane_64bit_elements
        .name(name() + ".numReads_perLane_64bit_elements")
        .desc("Number of reads to the vrf per lane, 64-bit elements");
    numWritess_perLane_64bit_elements
        .name(name() + ".numWritess_perLane_64bit_elements")
        .desc("Number of writes to the vrf per lane, 64-bit elements");
}

uint64_t
VectorRegister::get_size()
{
    return size;
}

VectorRegister::VectorRegister(const VectorRegisterParams* p) :
    ClockedObject(p),num_lanes(p->num_lanes),
    num_regs(p->num_regs),mvl(p->mvl),
    size(p->size), lineSize(p->lineSize),
    numPorts(p->numPorts), accessLatency(p->accessLatency)
{
    assert(size % lineSize == 0);
    assert(lineSize % sizeof(float) == 0);

    //in this moment 4-byte word is smallest addressable unit
    bytesPerBankAccess = sizeof(uint8_t);
    data = new uint8_t[size];

    for (uint8_t i=0; i<numPorts; ++i) {
        ports.push_back(new VectorRegisterPort(name() + ".port", *this));
    }

}

VectorRegister::~VectorRegister()
{
    delete data;
}


// vector_reg memory/port functions
bool
VectorRegister::handleTimingReq(PacketPtr pkt, VectorRegisterPort *port)
{
    //need to make sure all accesses happen within a single line
    uint64_t start_addr = pkt->getAddr();
    uint64_t end_addr = pkt->getAddr() + pkt->getSize() -1;
    uint64_t start_line_addr = start_addr - (start_addr % lineSize);
    uint64_t end_line_addr = end_addr - (end_addr % lineSize);

    assert(start_line_addr == end_line_addr);

    //need to make sure we are accessing full data from accessed banks
    assert(start_addr % bytesPerBankAccess == 0);
    assert((end_addr+1) % bytesPerBankAccess == 0);

    // The memories for the VRF can be seen as num_entries*WORD_WIDTH.
    //The read/write accesses are counted for each 64-bit read/write operation.
    uint64_t WORD_WIDTH = 8;

    // Physical register size in bytes
    uint64_t phys_reg_size = mvl/8 ;
    // Corresponding physical register
    uint64_t phys_reg = pkt->getAddr() / phys_reg_size;

    if (pkt->isRead())
    {
        numReads_64bit_elements = numReads_64bit_elements.value() + (pkt->getSize()/WORD_WIDTH); // 64-bit elements
        numReads_perLane_64bit_elements = numReads_perLane_64bit_elements.value() + ((pkt->getSize()/WORD_WIDTH) / num_lanes); // 64-bit elements
        memcpy(pkt->getPtr<uint8_t>(), data+start_addr, pkt->getSize());
        DPRINTF(VectorRegister,"Have been read %u bytes from addr 0x%lx (Physical Reg %d)\n"
            ,pkt->getSize(), pkt->getAddr(),phys_reg);
    } else {
        numWritess_64bit_elements = numWritess_64bit_elements.value() + (pkt->getSize()/WORD_WIDTH); // 64-bit elements
        numWritess_perLane_64bit_elements = numWritess_perLane_64bit_elements.value() + ((pkt->getSize()/WORD_WIDTH) / num_lanes); // 64-bit elements
        memcpy(data+start_addr, pkt->getPtr<uint8_t>(), pkt->getSize());
        DPRINTF(VectorRegister,"Have been written %u bytes to addr 0x%lx (Physical Reg %d)\n"
            ,pkt->getSize(), pkt->getAddr(),phys_reg);
    }

    pkt->makeTimingResponse();
    pkt->headerDelay = 0;
    pkt->payloadDelay = 0;
    port->schedTimingResp(pkt,curTick()
        + cyclesToTicks(Cycles(accessLatency))/*, true*/);

    return true;
}

Port &
VectorRegister::getPort(const std::string &if_name, PortID idx)
{
    //if (if_name != "port") {
    //    return AbstractMemory::getPort(if_name, idx);
    //} else {
        return *ports[idx];
    //}
}

VectorRegister *
VectorRegisterParams::create()
{
    return new VectorRegister(this);
}
