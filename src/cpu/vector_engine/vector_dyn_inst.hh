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

#ifndef __ARCH_RISCV_VECTOR_DYN_INSTS_HH__
#define __ARCH_RISCV_VECTOR_DYN_INSTS_HH__

#include <string>

#include "arch/riscv/insts/vector_static_inst.hh"

class VectorStaticInst;

class VectorDynInst
{
public:
VectorDynInst() : vinst(NULL),
  renamed_src1(1024),renamed_src2(1024),renamed_src3(1024),
  renamed_dst(1024),renamed_old_dst(1024),renamed_mask(1024),
  rob_entry(1024){
  }
~VectorDynInst() {}

/* Renamed registers */
  uint16_t get_renamed_src1()  { return renamed_src1; }
  void set_renamed_src1(uint16_t val)  { renamed_src1 = val; }

  uint16_t get_renamed_src2() { return renamed_src2; }
  void set_renamed_src2(uint16_t val) { renamed_src2  = val; }

  uint16_t get_renamed_src3() { return renamed_src3; }
  void set_renamed_src3(uint16_t val) { renamed_src3  = val; }

  uint16_t get_renamed_dst() { return renamed_dst; }
  void set_renamed_dst(uint16_t val) { renamed_dst  = val; }

  uint16_t get_renamed_old_dst() { return renamed_old_dst; }
  void set_renamed_old_dst(uint16_t val) { renamed_old_dst  = val; }

  uint16_t get_renamed_mask() { return renamed_mask; }
  void set_renamed_mask(uint16_t val) { renamed_mask  = val; }

/* rob_entry */
  uint16_t get_rob_entry() { return rob_entry; }
  void set_rob_entry(uint16_t val) { rob_entry  = val; }

  RiscvISA::VectorStaticInst*
  get_VectorStaticInst() {
    return vinst;
  }

  void
  set_VectorStaticInst(RiscvISA::VectorStaticInst* instruction){
    vinst  = instruction;
    }

private:
  RiscvISA::VectorStaticInst *vinst;
  uint16_t  renamed_src1;
  uint16_t  renamed_src2;
  uint16_t  renamed_src3;
  uint16_t  renamed_dst;
  uint16_t  renamed_old_dst;
  uint16_t  renamed_mask;
  uint16_t  rob_entry;
};

#endif // __ARCH_RISCV_VECTOR_DYN_INSTS_HH__
