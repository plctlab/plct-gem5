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
VectorConfig::reqAppVectorLength(uint64_t rvl, uint64_t vtype, bool r_mvl) {
    uint32_t gvl  = 0;
    uint32_t sew  =  get_vtype_sew(vtype);
    uint32_t lmul = get_vtype_lmul(vtype);
    gvl= lmul * max_vector_length/sew;
    return (r_mvl) ? gvl:(rvl>gvl) ? gvl:rvl;
}

uint64_t
VectorConfig::vector_length_in_bits(uint64_t vl, uint64_t vtype) {
    uint64_t vl_bits=0;
    uint32_t sew = get_vtype_sew(vtype);
    vl_bits = vl*sew;
    return  vl_bits;
}

uint64_t
VectorConfig::get_max_vector_length_elem(uint64_t vtype) {
    uint32_t mvl_elem=0;
    uint32_t sew  =  get_vtype_sew(vtype);
    uint32_t lmul = get_vtype_lmul(vtype);
    mvl_elem = lmul*max_vector_length/sew;
    return mvl_elem;
}

uint64_t
VectorConfig::get_max_vector_length_bits(uint64_t vtype) {
    uint32_t mvl_bits=0;
    //uint32_t sew  =  get_vtype_sew(vtype);
    uint32_t lmul = get_vtype_lmul(vtype);
    mvl_bits = lmul*max_vector_length;
    return mvl_bits;
}

uint64_t
VectorConfig::get_mvl_lmul1_bits() {
    uint32_t mvl_bits=0;
    mvl_bits = max_vector_length;
    return mvl_bits;
}

uint64_t
VectorConfig::get_vtype_lmul(uint64_t vtype) {
    uint8_t vlmul = vt(vtype,0,2);
    uint64_t LMUL;

    switch (vlmul){
        case 0:
            LMUL = 1; break;
        case 1: 
            LMUL = 2; break;
        case 2:
            LMUL = 4; break;
        case 3:
            LMUL = 8; break;
        default:  panic("LMUL not implemented\n"); LMUL = 0;
    }
    return LMUL;
}

uint64_t
VectorConfig::get_vtype_sew(uint64_t vtype) {
    uint8_t vsew = vt(vtype,2,3);
    uint64_t SEW;
    switch (vsew){
        case 0:
            SEW = 8; break;
        case 1: 
            SEW = 16; break;
        case 2:
            SEW = 32; break;
        case 3:
            SEW = 64; break;
        default:  panic("SEW not supported\n"); SEW = 0;
    }
    return SEW;
}

uint64_t
VectorConfig::get_vtype_ediv(uint64_t vtype) {
    uint8_t vediv = vt(vtype,5,2);
    uint64_t EDIV;

    switch (vediv){
        case 0:
            EDIV = 1; break;
        case 1: 
            EDIV = 2; break;
        case 2:
            EDIV = 4; break;
        case 3:
            EDIV = 8; break;
        default:  panic("EDIV not implemented\n"); EDIV = 0;
    }
    return EDIV;
}

VectorConfig *
VectorConfigParams::create()
{
    return new VectorConfig(this);
}

