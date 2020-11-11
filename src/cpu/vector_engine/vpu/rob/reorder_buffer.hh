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

#ifndef __CPU_VECTOR_ROB_H__
#define __CPU_VECTOR_ROB_H__


#include <bitset>
#include <cstdint>
#include <functional>
#include <vector>

#include "cpu/vector_engine/vector_engine.hh"
#include "params/ReorderBuffer.hh"
#include "sim/ticked_object.hh"

class VectorEngine;

class ReorderBuffer : public TickedObject
{
public:
    class rob_entry {
        public:
        rob_entry(uint16_t old_dst, bool valid_old_dst):
        old_dst(old_dst),valid_old_dst(valid_old_dst),executed(0) {}
        ~rob_entry() {}
        //private:
        uint16_t old_dst;
        bool valid_old_dst;
        bool executed;
    };

    ReorderBuffer(ReorderBufferParams *p);
    ~ReorderBuffer();

    void startTicking(VectorEngine& vector_wrapper);
    void stopTicking();
    bool isOccupied();
    void regStats() override;
    void evaluate() override;

    bool rob_full();
    bool rob_empty();
    uint32_t set_rob_entry(uint32_t old_dst, bool valid_old_dst);
    void set_rob_entry_executed(uint32_t idx);
protected:
    bool occupied;
public:
    const uint64_t ROB_Size;
private:
    std::vector<rob_entry *> rob;
    uint32_t tail;
    uint32_t head;
    int valid_elements;
    VectorEngine* vectorwrapper;
public:
    Stats::Scalar VectorROBentriesUsed;
};



#endif // __CPU_VECTOR_ROB_H__


