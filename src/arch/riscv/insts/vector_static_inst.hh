/*
 * Copyright (c) 2015 RISC-V Foundation
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
 */

#ifndef __ARCH_RISCV_VECTOR_STATIC_INSTS_HH__
#define __ARCH_RISCV_VECTOR_STATIC_INSTS_HH__

#include <string>

#include "arch/registers.hh"
#include "arch/riscv/types.hh"
#include "cpu/static_inst.hh"

namespace RiscvISA
{
/* VectorStaticInst holds the info of all vector instructions */
class VectorStaticInst : public StaticInst
{
protected:
    using StaticInst::StaticInst;

public:
      void advancePC(PCState &pc) const override { pc.advance(); }
      /* vector instruction name*/
      virtual std::string getName() const = 0;

      /* general riscv vector instruction */
      virtual bool isVectorInst() const = 0;
      /* riscv vector configuration instruction */
      virtual bool isSetVL() const = 0;
      virtual bool isSetVLi() const = 0;
      /* general riscv vector memory instruction */
      virtual bool isVectorInstMem() const = 0;
      /* vector load */
      virtual bool isLoad() const = 0;
      /* vector store */
      virtual bool isStore() const = 0;
      /* general riscv vector arithmetic instruction */
      virtual bool isVectorInstArith() const = 0;

      /* Vector instructions that writes back the result to the scalar rf */
      virtual bool write_to_scalar_reg() const = 0;
      /* Vector instructions without vector source registers (e.g. vmerge.vx)*/
      virtual bool arith_no_src() const = 0;
      /* Vector instructions  that uses src1 as vector source*/
      virtual bool arith_src1() const = 0;
      /* Vector instructions  that have src2 as vector source*/
      virtual bool arith_src2() const = 0;
      /* Vector instructions  that have src1 and src2 as vector sources*/
      virtual bool arith_src1_src2() const = 0;
      /* Vector instructions  with 3 vector sources (e.g. vmadd)*/
      virtual bool arith_src1_src2_src3() const = 0;

      /* vector slides */
      virtual bool is_slideup() const = 0;
      virtual bool is_slidedown() const = 0;
      virtual bool is_slide() const = 0;

      /* vector operations with masks*/
      virtual bool is_mask_m() const = 0;

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

uint32_t opcode() const { return x(0, 7); }

uint32_t vtype() const  override   { return x(20, 11); }


uint32_t func3() const override { return x(12, 3); }
uint32_t func5() const override { return x(27, 5); }
uint32_t func6() const override { return x(26, 6); }

bool vm() const  override   { return x(25, 1); }

uint8_t mop() const override { return x(26, 3); } // memory addressing mode

RegIndex vs1() const override { return (RegIndex)x(15, 5); }
RegIndex vs2() const override { return (RegIndex)x(20, 5); }
RegIndex vs3() const override { return (RegIndex)x(7, 5); }
RegIndex vd() const override { return (RegIndex)x(7, 5); }

bool isSetVL() const override {
  return (getName() == "vsetvli") | (getName() == "vsetvl") ; }

bool isSetVLi() const override {
  return (getName() == "vsetvli"); }

bool write_to_scalar_reg() const override {
  return ((getName() == "vfmv_fs") | (getName() == "vmpopc_m")
    | (getName() == "vmfirst_m"));  }

bool arith_no_src() const override {
  return ((getName() == "vfmerge_vf") | (getName() == "vmerge_vx")
    | (getName() == "vmerge_vi")); }

bool arith_src1() const override { return 0; }

bool arith_src2() const override {
  return ((getName() == "vadd_vi")      | (getName() == "vsub_vi")
    | (getName() == "vfsqrt_v")
    | (getName() == "vfcvt_x_f_v")  | (getName() == "vfcvt_f_x_v")
    | (getName() == "vfmv_fs")
    | (getName() == "vslideup_vi") | (getName() == "vslidedown_vi")
    | (getName() == "vslideup_vx") | (getName() == "vslidedown_vx")
    | (getName() == "vslide1up_vx") | (getName() == "vslide1down_vx")
    | (getName() == "vmpopc_m") | (getName() == "vmfirst_m")
    );  }

bool arith_src1_src2() const override {
  return ((getName() == "vfadd_vv") | (getName() == "vfsub_vv")
    | (getName() == "vfmul_vv")     | (getName() == "vfdiv_vv")
    | (getName() == "vfsgnj_vv")
    | (getName() == "vflt_vv")      | (getName() == "vfle_vv")
    | (getName() == "vmerge_vv")    | (getName() == "vfmin_vv")
    | (getName() == "vfmax_vv")
    | (getName() == "vand_vv")      | (getName() == "vor_vv")
    | (getName() == "vxor_vv")
    | (getName() == "vadd_vv")      | (getName() == "vsub_vv")
    | (getName() == "vmul_vv")
    | (getName() == "vdiv_vv")      | (getName() == "vrem_vv")
    | (getName() == "vsll_vv")      | (getName() == "vsrl_vv")
    | (getName() == "vmin_vv")
    | (getName() == "vmseq_vv")     | (getName() == "vmslt_vv")
    | (getName() == "vfsgnj_vv")    | (getName() == "vfsgnjn_vv")
    | (getName() == "vfsgnjx_vv")
    | (getName() == "vfredsum_vs")
    ); }

bool arith_src1_src2_src3() const override {
  return ((getName() == "vfmacc_vv") | (getName() == "vfmadd_vv")
    | (getName() == "vfmacc_vf")
    );  }

bool isLoad() const override {
  return ((getName() == "vlb_v") | (getName() == "vlh_v")
    | (getName() == "vlw_v")    | (getName() == "vle_v")
    | (getName() == "vlxe_v")   | (getName() == "vlxw_v")
    ); }

bool isStore() const override {
  return ((getName() == "vsb_v") | (getName() == "vsh_v")
    | (getName() == "vsw_v")    | (getName() == "vse_v")
    ); }

bool is_slideup() const override {
  return ((getName() == "vslideup_vi") | (getName() == "vslideup_vx")
    | (getName() == "vslide1up_vx")
    ); }

bool is_slidedown() const override {
  return ((getName() == "vslidedown_vi") | (getName() == "vslidedown_vx")
    | (getName() == "vslide1down_vx")
    ); }

bool is_slide() const override {
  return ( is_slideup() || is_slidedown()
    ); }

bool is_mask_m()  const override {
  return ((getName() == "vmpopc_m")  | (getName() == "vmfirst_m")
  ); }

bool isVectorInstArith() const override {
  return ((opcode() == 0x57) && (func3() != 0x7)
  ); }
bool isVectorInstMem() const override {
  return isLoad() || isStore(); }

bool isVectorInst() const {
  return ( isVectorInstArith() || isVectorInstMem() || isSetVL()
  ); }

uint32_t width() const override { return x(12, 3); }

private:
const uint32_t b;
uint32_t x(int lo, int len) const {
  return (b >> lo) & ((uint32_t(1) << len)-1);
}
const char *mnemo;
};

}

#endif // __ARCH_RISCV_VECTOR_STATIC_INSTS_HH__
