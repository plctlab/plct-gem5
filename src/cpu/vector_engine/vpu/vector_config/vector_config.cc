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

#include "cpu/vector_engine/vpu/vector_config/vector_config.hh"

#include <bitset>
#include <cstdint>
#include <deque>
#include <functional>

#include "debug/VectorConfig.hh"
#include "params/VectorConfig.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"

/**
 * VPU local Configuration
 */

VectorConfig::VectorConfig(VectorConfigParams *p) :
SimObject(p) ,  max_vector_length(p->max_vl)
{
}

VectorConfig::~VectorConfig()
{
}

uint64_t
VectorConfig::reqAppVectorLength(uint64_t rvl, uint64_t vtype, bool r_mvl){
uint32_t gvl=0;
uint32_t vsew = vt(vtype,2,3);

switch (vsew)
{
case 0:
    gvl=max_vector_length/8; break;
case 1:
    gvl=max_vector_length/16; break;
case 2:
    gvl=max_vector_length/32; break;
case 3:
    gvl=max_vector_length/64; break;
default:
    panic("vsew not implemented\n"); gvl=0;
}

return (r_mvl) ? gvl:(rvl>gvl) ? gvl:rvl;
}

uint64_t
VectorConfig::vector_length_in_bits(uint64_t vl, uint64_t vtype){
uint64_t vl_bits=0;
uint32_t vsew = vt(vtype,2,3);

switch (vsew)
{
case 0:
    vl_bits=vl*1*8; break;
case 1:
    vl_bits=vl*2*8; break;
case 2:
    vl_bits=vl*4*8; break;
case 3:
    vl_bits=vl*8*8; break;
default:
    panic("vsew not implemented\n"); vl_bits=0;
}

return  vl_bits;
}

uint64_t
VectorConfig::get_max_vector_length_elem(uint64_t vsew){
uint32_t mvl=0;
switch (vsew)
{
case 0:
    mvl=max_vector_length/8; break;
case 1:
    mvl=max_vector_length/16; break;
case 2:
    mvl=max_vector_length/32; break;
case 3:
    mvl=max_vector_length/64; break;
default:
    panic("Vtype not implemented\n"); mvl=0;
}
return mvl;
}

uint64_t
VectorConfig::get_max_vector_length_bits(){
    return max_vector_length;
}

VectorConfig *
VectorConfigParams::create()
{
    return new VectorConfig(this);
}
