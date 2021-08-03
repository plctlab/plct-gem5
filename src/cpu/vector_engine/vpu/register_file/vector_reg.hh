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

#ifndef __CPU_VECTOR_REG_HH__
#define __CPU_VECTOR_REG_HH__

#include <cstdint>
#include <string>
#include <vector>
#include <functional>

#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/abstract_mem.hh"
#include "mem/packet.hh"
#include "mem/qport.hh"
#include "params/VectorRegister.hh"
#include "debug/VectorRegister.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"

class VectorRegister : public ClockedObject
{
public:
    class VectorRegisterPort : public QueuedSlavePort
    {
      public:
        VectorRegisterPort(const std::string& name,
            VectorRegister& vector_reg);

      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        AddrRangeList getAddrRanges() const override;

      private:
        RespPacketQueue queue;
        VectorRegister& vector_reg;
    };

public:
    VectorRegister(const VectorRegisterParams *p);
    ~VectorRegister();

    Port& getPort(const std::string& if_name,
                                  PortID idx = InvalidPortID) override;

    // called by the VectorRegisterPort on a received packet
    bool handleTimingReq(PacketPtr pkt, VectorRegisterPort *port);
    void regStats() override;
    // Return the size of the register file in bytes
    uint64_t get_size();
private:

    //python configuration
    const uint64_t num_lanes;
    const uint64_t num_regs;
    const uint64_t mvl;
    const uint64_t size;
    const uint64_t lineSize;
    const uint64_t numPorts;
    const uint64_t accessLatency;

    //how many bytes per line are distributed to each bank
    uint64_t numBanks;
    uint64_t bytesPerBankAccess;

    // the slave port to access the vector_reg
    std::vector<VectorRegisterPort *> ports;

    //the physical vector_reg storage
    uint8_t * data;
public:
    Stats::Scalar numReads_64bit_elements;
    Stats::Scalar numWritess_64bit_elements;
    Stats::Scalar numReads_perLane_64bit_elements;
    Stats::Scalar numWritess_perLane_64bit_elements;
};

#endif //__CPU_VECTOR_REG_HH__
