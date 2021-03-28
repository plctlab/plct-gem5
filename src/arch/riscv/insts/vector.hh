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


#ifndef __ARCH_RISCV_VECTOR_INSTS_HH__
#define __ARCH_RISCV_VECTOR_INSTS_HH__

#include <string>

#include "arch/registers.hh"
#include "arch/riscv/insts/vector_static_inst.hh"
#include "cpu/static_inst.hh"

namespace RiscvISA
{
/*
 * Vector Arithmetic Instructions
 */
class RiscvVectorDataOp : public RiscvVectorInsn
    {
      public:
        RiscvVectorDataOp(const char *mnem, ExtMachInst _machInst,
            OpClass __opClass) :
            RiscvVectorInsn(mnem, _machInst, __opClass)
        {
          /*
           * Here is defined the scalar source registers and
           * destination registers for those vector
           * instructions that make us of it.
           */
            if ((func3()==4) || (func3()==6)) {
                _numSrcRegs = 1;
                _numDestRegs = 0;
                _srcRegIdx[0] = RegId(IntRegClass, vs1());
            } else if ((func3()==5)) {
                _numSrcRegs = 1;
                _numDestRegs = 0;
                _srcRegIdx[0] = RegId(FloatRegClass, vs1());
            } else {
                _numSrcRegs = 0;
                _numDestRegs = 0;
            }
        }

        std::string generateDisassembly(Addr pc,
            const Loader::SymbolTable *symtab) const;
    };

/*
 * Vector Configuration Instructions
 */
class RiscvVectorCfgOp : public RiscvVectorInsn
    {
      public:
        RiscvVectorCfgOp(const char *mnem, ExtMachInst _machInst,
            OpClass __opClass) :
            RiscvVectorInsn(mnem, _machInst, __opClass)
        {
          /*
           * Here is defined the scalar source registers and
           * destination registers for those vector
           * instructions that make us of it.
           */
            if (getName() == "vsetvli") {
             _numSrcRegs =  1;
             _numDestRegs = 1;
             _srcRegIdx[0] = RegId(IntRegClass, vs1());
             _destRegIdx[0] = RegId(IntRegClass, vd());
             } else {
             _numSrcRegs =  2;
             _numDestRegs = 1;
             _srcRegIdx[0] = RegId(IntRegClass, vs1());
             _srcRegIdx[1] = RegId(IntRegClass, vs2());
             _destRegIdx[0] = RegId(IntRegClass, vd());
             }
        }

        std::string generateDisassembly(Addr pc,
            const Loader::SymbolTable *symtab) const;
    };
/*
 * Vector Memory Instructions
 */
class RiscvVectorMemOp : public RiscvVectorInsn
    {
      public:
        RiscvVectorMemOp(const char *mnem, ExtMachInst _machInst,
            OpClass __opClass) :
            RiscvVectorInsn(mnem, _machInst, __opClass)
        {
          /*
           * Here is defined the scalar source registers and
           * destination registers for those vector
           * instructions that make us of it.
           * TODO: is still pending strided which uses rs2 as
           * second operand.
           */
          if(mop()==2) { // Strided memory access
            _numSrcRegs =  2;
            _numDestRegs = 0;
            _srcRegIdx[0] = RegId(IntRegClass, vs1());
            _srcRegIdx[1] = RegId(IntRegClass, vs2());
          } else {
            _numSrcRegs =  1;
            _numDestRegs = 0;
            _srcRegIdx[0] = RegId(IntRegClass, vs1());
          }
        }

        std::string generateDisassembly(Addr pc,
            const Loader::SymbolTable *symtab) const;
    };
/*
 * Move data to scalar core instructions
 */
class RiscvVectorToScalarOp : public RiscvVectorInsn
    {
      public:
        RiscvVectorToScalarOp(const char *mnem, ExtMachInst _machInst,
            OpClass __opClass) :
            RiscvVectorInsn(mnem, _machInst, __opClass)
        {
          /*
           * Here is defined the destination registers for those vector
           * instructions that make us of it. These instructions writes
           * in the integer or floating point register.
           */
            if ((func3()==1)) {
                _numSrcRegs = 0;
                _numDestRegs = 1;
                _destRegIdx[0] = RegId(FloatRegClass, vd());
            } else if ((func3()==2)) {
                _numSrcRegs = 0;
                if ((func6()==12)) {
                  _numSrcRegs = 1;
                  _srcRegIdx[0] = RegId(IntRegClass, vs1());
                }
                _numDestRegs = 1;
                _destRegIdx[0] = RegId(IntRegClass, vd());
            }
        }

        std::string generateDisassembly(Addr pc,
            const Loader::SymbolTable *symtab) const;
    };
}
#endif // __ARCH_RISCV_VECTOR_INSTS_HH__