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

#include "cpu/vector_engine/vector_engine_interface.hh"

#include <cassert>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "debug/VectorEngineInterface.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"

VectorEngineInterface::VectorEngineInterface(VectorEngineInterfaceParams *p) :
SimObject(p),vector_engine(p->vector_engine)
{
}

VectorEngineInterface::~VectorEngineInterface()
{
}

bool
VectorEngineInterface::requestGrant(RiscvISA::VectorStaticInst* vinst)
{
    bool grant = vector_engine->requestGrant(vinst);
    DPRINTF(VectorEngineInterface,"Resquesting a grant with answer : %d\n",grant);
    return grant;
}

void
VectorEngineInterface::sendCommand(RiscvISA::VectorStaticInst* vinst ,ExecContextPtr& xc ,
        uint64_t src1, uint64_t src2,
        std::function<void()> done_callback)
{
    DPRINTF(VectorEngineInterface,"Sending a new command to the vector engine\n");
    vector_engine->dispatch(*vinst,xc,src1,src2,done_callback);
}

uint64_t
VectorEngineInterface::reqAppVectorLength(uint64_t rvl, uint64_t vtype, bool r_mvl)
{
    DPRINTF(VectorEngineInterface,"Resquesting a vector length\n");
     uint64_t gvl = vector_engine->vector_config->
        reqAppVectorLength(rvl,vtype,r_mvl);
    return gvl;
}

bool
VectorEngineInterface::bussy()
{

    bool bussy = vector_engine->isOccupied();
    return bussy;
}

VectorEngineInterface *
VectorEngineInterfaceParams::create()
{
    return new VectorEngineInterface(this);
}