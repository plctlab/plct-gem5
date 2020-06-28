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

#ifndef __CPU_VECTOR_ENGINE_INTERFACE_HH__
#define __CPU_VECTOR_ENGINE_INTERFACE_HH__

#include <algorithm>
#include <functional>
#include <numeric>
#include <string>
#include <vector>

#include "arch/riscv/insts/vector_static_inst.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/minor/exec_context.hh"
#include "cpu/vector_engine/vector_engine.hh"
#include "params/VectorEngineInterface.hh"
#include "sim/sim_object.hh"

/**
 * The VectorEngineInterface is a custom interface for the RISC-V Vector Engine
 */
class VectorEngine;

class VectorEngineInterface : public SimObject
{
public:
    VectorEngineInterface(VectorEngineInterfaceParams *p);
    ~VectorEngineInterface();

    /**
    * requestGrant function is used by the scalar to ask for permision to send
    * a new vector instruction to the vector engine.
    */
    bool requestGrant(RiscvISA::VectorStaticInst* insn);

    /**
    * sendCommand function receives the command from the scalar core
    * and sent it to the vector engine. Previous to use this function,
    * a granted signal  from the vector engine must be received, otherwise,
    * the command must not be send.
    */
    void sendCommand(RiscvISA::VectorStaticInst *vinst ,ExecContextPtr& xc ,
        uint64_t src1, uint64_t src2, std::function<void()> done_callback);

    /**
    * reqAppVectorLength function is used by the vector configuration
    * instructions. This instructions ask to the vector engine for some
    * vector length, and the vector engine answer with the available one.
    */
    uint64_t reqAppVectorLength(uint64_t rvl, uint64_t vtype, bool r_mvl);

    /**
    * bussy function is used by the scalar core to know the state of the vector
    * engine. "1" means that the vector engine has vector instructions either
    * in the instructions queues, memory unit o vector lanes. It is useful for
    * synchronization.
    */
    bool bussy();

private:
    /**
    The vector Engine
    */
    VectorEngine *vector_engine;
};
#endif // __CPU_VECTOR_ENGINE_INTERFACE_HH__