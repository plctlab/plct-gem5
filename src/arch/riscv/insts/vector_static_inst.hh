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

#ifndef __ARCH_RISCV_VECTOR_STATIC_INSTS_HH__
#define __ARCH_RISCV_VECTOR_STATIC_INSTS_HH__

#include <string>

#include "arch/registers.hh"
#include "arch/riscv/types.hh"
#include "cpu/static_inst.hh"

namespace RiscvISA
{
/*
struct VecStaticInstFlags {
  enum VecFlags {
      IsVecArithmOp = 0,
      IsVecMemOp = 1,
      IsVecConfigOp = 2,
      IsVecFP = 3,
      IsVecInt = 4,
      IsVecOneSrc = 5,
      IsVecTwoSrc = 6,
      IsVecThreeSrc = 7,
      NumVecFlags = 8
  };
  static const char *FlagsStrings[NumVecFlags];
};
*/
/* VectorStaticInst holds the info of all vector instructions */
class VectorStaticInst : public StaticInst//, public VecStaticInstFlags
{
protected:
    using StaticInst::StaticInst;
    //std::bitset<NumVecFlags> vecflags;

public:
      void advancePC(PCState &pc) const override { pc.advance(); }
      /* vector instruction name*/
      virtual std::string getName() const = 0;


      /* general riscv vector instruction */
      virtual bool isVectorInst() const = 0;
      /* riscv vector configuration instruction */
      virtual bool isVecConfig() const = 0;
      /* general riscv vector memory instruction */
      virtual bool isVectorInstMem() const = 0;
      /* vector load */
      virtual bool isLoad() const = 0;
      /* vector store */
      virtual bool isStore() const = 0;
      /* general riscv vector arithmetic instruction */
      virtual bool isVectorInstArith() const = 0;
      /* Vector reduction instruction */
      virtual bool is_reduction() const = 0;

      /* Single-Width Floating-Point/Integer Type-Convert Instructions */
      virtual bool isConvertIntToFP() const = 0;
      virtual bool isConvertFPToInt() const = 0;
      /* Widening Floating-Point/Integer Type-Convert Instructions */
      virtual bool isWidening() const = 0;
      virtual bool isWConvertFPToInt() const = 0;
      virtual bool isWConvertIntToFP() const = 0;
      virtual bool isWConvertFPToFP() const = 0;
      /* Narrowing Floating-Point/Integer Type-Convert Instructions */
      virtual bool isNarrowing() const = 0;
      virtual bool isNConvertFPToInt() const = 0;
      virtual bool isNConvertIntToFP() const = 0;
      virtual bool isNConvertFPToFP() const = 0;

      /* Vector Floating-Point Compare Instruction */
      virtual bool isFPCompare() const = 0;
      /* Vector Integer Comparison Instructions */
      virtual bool isIntCompare() const = 0;
      /* The instruction create a Mask */
      virtual bool isMaskDst() const = 0;
      /* Vector instructions that writes back the result to the scalar rf */
      virtual bool VectorMaskLogical() const = 0;
      /* Vector instructions that writes back the result to the scalar rf */
      virtual bool VectorToScalar() const = 0;
      /* Vector instructions  that have src2 as vector source*/
      virtual bool arith1Src() const = 0;
      /* Vector instructions  that have src1 and src2 as vector sources*/
      virtual bool arith2Srcs() const = 0;
      /* Vector instructions  with 3 vector sources (e.g. vmadd)*/
      virtual bool arith3Srcs() const = 0;

      virtual bool isFP()  const = 0;
      virtual bool isInt()  const = 0;
      virtual bool isConvert()  const = 0;

      /* vector slides */
      virtual bool is_slideup() const = 0;
      virtual bool is_slidedown() const = 0;
      virtual bool is_slide() const = 0;

      /* vtype field*/
      virtual uint32_t vtype() const = 0;
      /* func3 field*/
      virtual uint32_t func3() const = 0;
      /* func5 field*/
      virtual uint32_t func5() const = 0;
      /* func6 field*/
      virtual uint32_t func6() const = 0;
      /* vm field - indicates if the operations is masked or not*/
      virtual bool vm() const = 0;
      /* mop field - indicates the memory addressing mode*/
      virtual uint8_t mop() const = 0;
      /* width field - specifies size of memory elements,
      and distinguishes from FP scalar*/
      virtual uint32_t width() const = 0;
      /* vs1, vs2, vs3 and vd fields*/
      virtual RegIndex vs1() const = 0;
      virtual RegIndex vs2() const = 0;
      virtual RegIndex vs3() const = 0;
      virtual RegIndex vd() const = 0;
      /* the PC of the instruction*/
      uint64_t getPC() { return pc; }
      void setPC(uint64_t program_counter) { pc = program_counter; }

private:
  /* the PC of the instruction*/
  uint64_t pc;
};


class RiscvVectorInsn : public VectorStaticInst
{
  protected:
  RiscvVectorInsn(const char *mnem, MachInst _machInst, OpClass __opClass):
      VectorStaticInst(mnem, _machInst, __opClass),
      b(_machInst),mnemo(mnem){}
  ~RiscvVectorInsn() {}

  std::string getName() const  override { return mnemo; }

  std::string regName(RegIndex reg) const;

  virtual std::string generateDisassembly(Addr pc,
          const Loader::SymbolTable *symtab) const = 0;

  uint32_t bits() { return b; }

  uint32_t opcode()          const { return x(0, 7); }

  uint32_t vtype()           const  override   { return x(20, 11); }

  uint32_t func3()           const override { return x(12, 3); }
  uint32_t func5()           const override { return x(27, 5); }
  uint32_t func6()           const override { return x(26, 6); }

  bool vm()                  const  override   { return x(25, 1); }

  uint8_t mop()              const override { return x(26, 3); }

  RegIndex vs1()             const override { return (RegIndex)x(15, 5); }
  RegIndex vs2()             const override { return (RegIndex)x(20, 5); }
  RegIndex vs3()             const override { return (RegIndex)x(7, 5); }
  RegIndex vd()              const override { return (RegIndex)x(7, 5); }

  bool isFP()                const  override   { return ((func3()==1) || (func3()==5)) && !isConvert(); }
  bool isInt()               const  override   { return ((func3()==0) || (func3()==2) || (func3()==3) || (func3()==4) || (func3()==6)) && !is_slide(); }
  bool isConvert()           const override { return isConvertIntToFP() || isConvertFPToInt(); }
  bool is_slide()            const override { return is_slideup() || is_slidedown() ; }

  bool VectorToScalar()      const override { return opClass() == VectorToScalarOp; }

  bool VectorMaskLogical()   const override { return opClass() == VectorMaskLogicalOp; }

  bool arith1Src()           const override { return (opClass() == VectorArith1SrcOp) 
                                                    || VectorToScalar() 
                                                    || isConvertFPToInt() || isConvertIntToFP()
                                                    || isWConvertFPToInt() || isWConvertIntToFP() || isWConvertFPToFP()
                                                    ; }

  bool arith2Srcs()          const override { return (opClass() == VectorArith2SrcOp) || is_slide() || VectorMaskLogical() || is_reduction() || isFPCompare() || isIntCompare(); }

  bool arith3Srcs()          const override { return opClass() == VectorArith3SrcOp; }

  bool isLoad()              const override { return opClass() == VectorMemoryLoadOp; }

  bool isStore()             const override { return opClass() == VectorMemoryStoreOp; }

  bool isConvertIntToFP()    const override { return (opClass() == VectorConvertIntToFPOp) || (opClass() == VectorWConvertIntToFPOp) || (opClass() == VectorNConvertIntToFPOp); }
  bool isConvertFPToInt()    const override { return opClass() == VectorConvertFPToIntOp; }

  bool isWConvertFPToInt()    const override { return opClass() == VectorWConvertFPToIntOp; }
  bool isWConvertIntToFP()    const override { return opClass() == VectorWConvertIntToFPOp; }
  bool isWConvertFPToFP()     const override { return opClass() == VectorWConvertFPToFPOp; }
  bool isNConvertFPToInt()    const override { return opClass() == VectorNConvertFPToIntOp; }
  bool isNConvertIntToFP()    const override { return opClass() == VectorNConvertIntToFPOp; }
  bool isNConvertFPToFP()     const override { return opClass() == VectorNConvertFPToFPOp; }

  bool isWidening()           const override { return isWConvertFPToInt() || isWConvertIntToFP() || isWConvertFPToFP(); }
  bool isNarrowing()          const override { return isNConvertFPToInt() || isNConvertIntToFP() || isNConvertFPToFP(); }

  bool is_reduction()        const override { return opClass() == VectorReductionOp; }

  bool isFPCompare()         const override { return opClass() == VectorFPCompareOp; }
  bool isIntCompare()        const override { return opClass() == VectorIntCompareOp; }

  bool isMaskDst()           const override { return isFPCompare() || isIntCompare(); }

  bool is_slideup()          const override { return opClass() == VectorSlideUpOp; }

  bool is_slidedown()        const override { return opClass() == VectorSlideDownOp; }

  bool isVectorInstArith()   const override { return arith1Src() || arith2Srcs() || arith3Srcs(); }

  bool isVectorInstMem()     const override { return isLoad() || isStore(); }

  //bool isVecConfigOp()          const { return vecflags[IsVecConfigOp]; }
  bool isVecConfig()         const override { return opClass() == VectorConfigOp; }

  bool isVectorInst()        const { return isVectorInstArith() || isVectorInstMem() || isVecConfig(); }

  uint32_t width()           const override { return x(12, 3); }

private:
  const uint32_t b;
  uint32_t x(int lo, int len) const {
    return (b >> lo) & ((uint32_t(1) << len)-1);
  }
  const char *mnemo;
};

}

#endif // __ARCH_RISCV_VECTOR_STATIC_INSTS_HH__
