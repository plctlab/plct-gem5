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

#ifndef __CPU_VECTOR_CSR_H__
#define __CPU_VECTOR_CSR_H__

#include <bitset>
#include <cstdint>
#include <deque>
#include <functional>

#include "debug/VectorConfig.hh"
#include "params/VectorConfig.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"

/**
 * The MVL is given in bits.
 * A vector register is viewed as being divided
 * into VLEN/SEW standard-width elements
 * Vector standard element width - VSEW.
 * The VL is given in elements.
 */

class VectorConfig : public SimObject
{

public:
    VectorConfig(VectorConfigParams *p);
    ~VectorConfig();

    uint64_t reqAppVectorLength(uint64_t rvl, uint64_t vtype, bool r_mvl);
    uint64_t vector_length_in_bits(uint64_t vl, uint64_t vtype);
    uint64_t get_max_vector_length_elem(uint64_t vtype);
    uint64_t get_max_vector_length_bits(uint64_t vtype);
    uint64_t get_mvl_lmul1_bits();

    uint64_t get_vtype_lmul(uint64_t vtype);
    uint64_t get_vtype_sew(uint64_t vtype);
    uint64_t get_vtype_ediv(uint64_t vtype);

private:
    /* The maximum vector length in bits of one vector register (LMUL=1)*/
    uint64_t max_vector_length;

    uint64_t vt(uint64_t val, int lo, int len) const {
      return (val >> lo) & ((uint64_t(1) << len)-1);
    }

};


#endif // __CPU_VECTOR_CSR_H__


