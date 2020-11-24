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

#ifndef __CPU_VECTOR_EXE_UNIT_HH__
#define __CPU_VECTOR_EXE_UNIT_HH__

#include <cstdint>
#include <deque>
#include <functional>

#include "arch/riscv/insts/vector_static_inst.hh"
#include "cpu/vector_engine/vector_dyn_inst.hh"
#include "cpu/vector_engine/vector_engine.hh"
#include "cpu/vector_engine/vmu/read_timing_unit.hh"
#include "cpu/vector_engine/vmu/write_timing_unit.hh"
#include "cpu/vector_engine/vpu/multilane_wrapper/datapath.hh"
#include "params/VectorLane.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"

class VectorEngine;

class VectorLane : public SimObject
{
  public:
    VectorLane(const VectorLaneParams *p);
    ~VectorLane();

    bool isOccupied();
    void issue(VectorEngine& vector_wrapper,RiscvISA::VectorStaticInst& insn,
        VectorDynInst *dyn_insn, ExecContextPtr& xc, uint64_t src1,
        uint64_t vtype,uint64_t vl,
        std::function<void(Fault fault)> done_callback);
    //internal state for current instruction
    //these need to be public so Datapath can access them
    std::deque<uint8_t*> AdataQ;
    std::deque<uint8_t*> BdataQ;
    std::deque<uint8_t*> MdataQ;
    std::deque<uint8_t*> DstdataQ;
    uint64_t data_acc_dp;
    uint32_t data_acc_sp;
  protected:
    uint64_t lane_id;
    bool occupied;
    MemUnitReadTiming * srcAReader;
    MemUnitReadTiming * srcBReader;
    MemUnitReadTiming * srcMReader;
    MemUnitReadTiming * dstReader;
    MemUnitWriteTiming * dstWriter;

  private:
    //configuration from python object
    Datapath * dataPath;
    VectorEngine* vectorwrapper;

    //internal state for current instruction
    uint64_t Aread;
    uint64_t Bread;
    uint64_t Mread;
    uint64_t Dread;
    uint64_t Dstread;

    bool arith1Src;
    bool arith2Srcs;
    bool arith3Srcs;
    bool vector_to_scalar;

    uint8_t scalar_reg;
    uint64_t scalar_data;

    bool masked_op;
    bool vx_op;
    bool vf_op;
    bool vi_op;
};

#endif // __CPU_VECTOR_EXE_UNIT_HH__
