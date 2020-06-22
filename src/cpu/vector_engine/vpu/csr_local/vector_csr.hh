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

#include "debug/VectorCsrReg.hh"
#include "params/VectorCsrReg.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"

/**
 * The MVL is given in bits.
 * A vector register is viewed as being divided
 * into VLEN/SEW standard-width elements
 * Vector standard element width - VSEW.
 * The VL is given in elements.
 */

class VectorCsrReg : public SimObject
{

public:
    VectorCsrReg(VectorCsrRegParams *p);
    ~VectorCsrReg();

    uint64_t req_new_vector_length(uint64_t rvl, uint64_t vtype, bool r_mvl);
    uint64_t get_max_vector_length_elem(uint64_t vsew);
    uint64_t get_max_vector_length_bits();

    uint64_t get_vector_length(){ return vector_length; }
    void set_vector_length(uint64_t val){
        vector_length  = val;
        DPRINTF(VectorCsrReg,"Setting vl : %d\n", vector_length);
    }

    uint64_t get_vtype_vlmul() const { return vtype.vlmul; }
    uint64_t get_vtype_vsew() const { return vtype.vsew; }
    uint64_t get_vtype_vediv() const { return vtype.vediv; }

    void set_vtype(uint64_t val)
    {
    vtype.vlmul = vt(val,0,2);
    vtype.vsew = vt(val,2,3);
    vtype.vediv = vt(val,5,2);
    DPRINTF(VectorCsrReg,"Setting vtype : %d , vlmul %d , vsew %d, vediv %d\n"
        ,val, vtype.vlmul, vtype.vsew , vtype.vediv);
    }

private:
    uint64_t max_vector_length;
    uint64_t vector_length;

    struct vtype{ uint64_t vlmul; uint64_t vsew; uint64_t vediv; } vtype;

    uint64_t vt(uint64_t val, int lo, int len) const {
      return (val >> lo) & ((uint64_t(1) << len)-1);
    }

};


#endif // __CPU_VECTOR_CSR_H__


