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

#ifndef __CPU_INST_QUEUE_HH__
#define __CPU_INST_QUEUE_HH__

#include <cmath>
#include <cstdint>
#include <functional>

#include "arch/riscv/insts/vector_static_inst.hh"
#include "base/statistics.hh"
#include "cpu/minor/exec_context.hh"
#include "cpu/vector_engine/vector_engine.hh"
#include "cpu/vector_engine/vpu/rob/reorder_buffer.hh"
#include "params/InstQueue.hh"
#include "sim/ticked_object.hh"

class VectorEngine;
//class ExecContextPtr;
//class ExecContext;

class InstQueue : public TickedObject
{

public:
    class QueueEntry {
        public:
        QueueEntry(RiscvISA::VectorStaticInst& insn, VectorDynInst *dyn_insn,
            ExecContextPtr& _xc, std::function<void()> dependencie_callback,
            uint64_t src1,uint64_t src2,uint64_t rename_vtype,uint64_t rename_vl):
            dependencie_callback(dependencie_callback),
            insn(insn),
            dyn_insn(dyn_insn)/*,xc(_xc)*/,src1(src1),src2(src2),
            rename_vtype(rename_vtype),rename_vl(rename_vl),issued(0)
            {
                xc=_xc;
            }
        ~QueueEntry() {}

        std::function<void()> dependencie_callback;
        RiscvISA::VectorStaticInst& insn;
        VectorDynInst     *dyn_insn;
        ExecContextPtr xc;
        uint64_t src1;
        uint64_t src2;
        uint64_t rename_vtype;
        uint64_t rename_vl;
        bool issued;
    };

    InstQueue(InstQueueParams *p);
    ~InstQueue();

    void startTicking(VectorEngine& vector_wrapper/*,
        std::function<void()> dependencie_callback*/);
    void stopTicking();
    bool isOccupied();

    bool arith_queue_full();
    bool mem_queue_full();
    //overrides
    void regStats() override;
    void evaluate() override;

protected:
    bool occupied;

public:
    std::deque<QueueEntry *> Instruction_Queue;
    std::deque<QueueEntry *> Memory_Queue;
    bool OoO_queues;
    uint64_t vector_mem_queue_size;
    uint64_t vector_arith_queue_size;
private:
    VectorEngine* vectorwrapper;
    bool dst_dp;
    //std::function<void(uint8_t*,uint8_t)> dataCallback;
    //std::function<void()> dependencie_callback;
public:
    Stats::Scalar idle_count_by_dependency;
    Stats::Scalar VectorMemQueueSlotsUsed;
    Stats::Scalar VectorArithQueueSlotsUsed;
};

#endif //__CPU_INST_QUEUE_HH__
