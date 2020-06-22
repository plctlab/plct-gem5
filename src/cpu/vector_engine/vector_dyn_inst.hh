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
VectorDynInst() :inst(NULL),PSrc1(1024),PSrc2(1024),PSrc3(1024),
  PDst(1024),POldDst(1024),PMask(1024),rob_entry(1024){}
~VectorDynInst() {}

  uint16_t get_PSrc1()  { return PSrc1; }
  void set_PSrc1(uint16_t val)  { PSrc1 = val; }

  uint16_t get_PSrc2() { return PSrc2; }
  void set_PSrc2(uint16_t val) { PSrc2  = val; }

  uint16_t get_PSrc3() { return PSrc3; }
  void set_PSrc3(uint16_t val) { PSrc3  = val; }

  uint16_t get_PDst() { return PDst; }
  void set_PDst(uint16_t val) { PDst  = val; }

  uint16_t get_POldDst() { return POldDst; }
  void set_POldDst(uint16_t val) { POldDst  = val; }

  uint16_t get_PMask() { return PMask; }
  void set_PMask(uint16_t val) { PMask  = val; }

  uint16_t get_rob_entry() { return rob_entry; }
  void set_rob_entry(uint16_t val) { rob_entry  = val; }

  RiscvISA::VectorStaticInst*
  get_VectorStaticInst() {
    return inst;
  }

  void
  set_VectorStaticInst(RiscvISA::VectorStaticInst* instruction){
    inst  = instruction;
    }

private:
  RiscvISA::VectorStaticInst *inst;
  uint16_t  PSrc1;
  uint16_t  PSrc2;
  uint16_t  PSrc3;
  uint16_t  PDst;
  uint16_t  POldDst;
  uint16_t  PMask;
  uint16_t  rob_entry;
};
#endif // __ARCH_RISCV_VECTOR_DYN_INSTS_HH__
