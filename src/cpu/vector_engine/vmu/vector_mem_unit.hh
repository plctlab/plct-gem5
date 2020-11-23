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

#ifndef __CPU_MEM_UNIT_HH__
#define __CPU_MEM_UNIT_HH__

#include <functional>

#include "arch/riscv/insts/vector_static_inst.hh"
#include "cpu/minor/exec_context.hh"
#include "cpu/vector_engine/vector_dyn_inst.hh"
#include "cpu/vector_engine/vector_engine.hh"
#include "cpu/vector_engine/vmu/read_timing_unit.hh"
#include "cpu/vector_engine/vmu/write_timing_unit.hh"
#include "params/VectorMemUnit.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"

class VectorEngine;

class VectorMemUnit : public SimObject
{
  public:
    VectorMemUnit(const VectorMemUnitParams *p);
    ~VectorMemUnit();

    bool isOccupied();
    void issue(VectorEngine& vector_wrapper,RiscvISA::VectorStaticInst& insn,
      VectorDynInst *dyn_insn, ExecContextPtr& xc, uint64_t src1, uint64_t src2,
      uint64_t vtype,uint64_t vl,
      std::function<void(Fault fault)> done_callback);

  private:
    bool occupied;
    MemUnitReadTiming * memReader;
    MemUnitReadTiming * memReader_addr;
    MemUnitWriteTiming * memWriter;

    VectorEngine* vectorwrapper;

    uint64_t vt(uint64_t val, int lo, int len) const {
      return (val >> lo) & ((uint64_t(1) << len)-1);
    }
};

#endif // __CPU_MEM_UNIT_HH__
