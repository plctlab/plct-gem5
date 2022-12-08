/*
 * Copyright (c) 2022 PLCT Lab
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

#include "arch/riscv/insts/vector.hh"

#include <sstream>
#include <string>

#include "arch/riscv/insts/static_inst.hh"
#include "arch/riscv/utility.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

namespace RiscvISA
{

float
getVflmul(uint32_t vlmul_encoding) {
  int vlmul = int8_t(vlmul_encoding << 5) >> 5;
  float vflmul = vlmul >= 0 ? 1 << vlmul : 1.0 / (1 << -vlmul);
  return vflmul;
}

uint32_t
getVlmax(VTYPE vtype, uint32_t vlen) {
  uint32_t sew = getSew(vtype.vsew);
  uint32_t vlmax = (vlen/sew) * getVflmul(vtype.vlmul);
  return vlmax;
}

std::string
VConfOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", ";
    if (bit31 && bit30 == 0) {
        ss << registerName(srcRegIdx(0)) << ", " << registerName(srcRegIdx(1));
    } else if (bit31 && bit30) {
        ss << uimm << ", " << generateZimmDisassembly();
    } else {
        ss << registerName(srcRegIdx(0)) << ", " << generateZimmDisassembly();
    }
    return ss.str();
}

std::string
VConfOp::generateZimmDisassembly() const
{
    std::stringstream s;

    // VSETIVLI uses ZIMM10 and VSETVLI uses ZIMM11
    uint64_t zimm = (bit31 && bit30) ? zimm10 : zimm11;

    bool frac_lmul = bits(zimm, 2);
    int sew = 1 << (bits(zimm, 5, 3) + 3);
    int lmul = bits(zimm, 1, 0);
    auto vta = bits(zimm, 6) == 1 ? "ta" : "tu";
    auto vma = bits(zimm, 7) == 1 ? "ma" : "mu";
    s << "e" << sew;
    if (frac_lmul) {
        std::string lmul_str = "";
        switch(lmul){
        case 3:
            lmul_str = "f2";
            break;
        case 2:
            lmul_str = "f4";
            break;
        case 1:
            lmul_str = "f8";
            break;
        default:
            panic("Unsupport fractional LMUL");
        }
        s << ", m" << lmul_str;
    } else {
        s << ", m" << (1 << lmul);
    }
    s << ", " << vta << ", " << vma;
    return s.str();
}

std::string
VectorNonSplitInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", "
        << registerName(srcRegIdx(0));
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VectorArithMicroInst::generateDisassembly(Addr pc,
        const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", ";
    if (machInst.funct3 == 0x3) {
        // OPIVI
      ss  << registerName(srcRegIdx(0)) << ", " << machInst.vecimm;
    } else {
      ss  << registerName(srcRegIdx(1)) << ", " << registerName(srcRegIdx(0));
    }
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VectorArithMacroInst::generateDisassembly(Addr pc,
        const Loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", ";
    if (machInst.funct3 == 0x3) {
        // OPIVI
      ss  << registerName(srcRegIdx(0)) << ", " << machInst.vecimm;
    } else {
      ss  << registerName(srcRegIdx(1)) << ", " << registerName(srcRegIdx(0));
    }
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VectorVMUNARY0MicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0));
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VectorVMUNARY0MacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0));
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VectorSlideMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) <<  ", ";
    if (machInst.funct3 == 0x3) {
      ss  << registerName(srcRegIdx(0)) << ", " << machInst.vecimm;
    } else {
      ss  << registerName(srcRegIdx(1)) << ", " << registerName(srcRegIdx(0));
    }
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VectorSlideMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", ";
    if (machInst.funct3 == 0x3) {
      ss  << registerName(srcRegIdx(0)) << ", " << machInst.vecimm;
    } else {
      ss  << registerName(srcRegIdx(1)) << ", " << registerName(srcRegIdx(0));
    }
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VleMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", "
       << VLENB * microIdx << '(' << registerName(srcRegIdx(0)) << ')' << ", "
       << registerName(srcRegIdx(1));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VlWholeMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", "
       << VLENB * microIdx << '(' << registerName(srcRegIdx(0)) << ')';
    return ss.str();
}

std::string VseMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(1)) << ", "
       << VLENB * microIdx  << '(' << registerName(srcRegIdx(0)) << ')';
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VsWholeMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(1)) << ", "
       << VLENB * microIdx << '(' << registerName(srcRegIdx(0)) << ')';
    return ss.str();
}

std::string VleMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')';
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VlWholeMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')';
    return ss.str();
}

std::string VseMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(1)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')';
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VsWholeMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(1)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')';
    return ss.str();
}

std::string VlStrideMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')' <<
        ", " << registerName(srcRegIdx(1));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VlStrideMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')' <<
        ", "<< registerName(srcRegIdx(1));
    if (microIdx != 0 || machInst.vtype8.vma == 0 || machInst.vtype8.vta == 0)
        ss << ", " << registerName(srcRegIdx(2));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VsStrideMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(2)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')' <<
        ", " << registerName(srcRegIdx(1));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VsStrideMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(2)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')' <<
        ", "<< registerName(srcRegIdx(1));
    if (microIdx != 0 || machInst.vtype8.vma == 0 || machInst.vtype8.vta == 0)
        ss << ", " << registerName(srcRegIdx(2));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VlIndexMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", "
        << '(' << registerName(srcRegIdx(0)) << "),"
        << registerName(srcRegIdx(1));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VlIndexMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(destRegIdx(0)) << "[" << uint16_t(vdElemIdx) << "], "
        << '(' << registerName(srcRegIdx(0)) << "), "
        << registerName(srcRegIdx(1)) << "[" << uint16_t(vs2ElemIdx) << "]";
    if (microIdx != 0 || machInst.vtype8.vma == 0 || machInst.vtype8.vta == 0)
        ss << ", " << registerName(srcRegIdx(2));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VsIndexMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(2)) << ", "
        << '(' << registerName(srcRegIdx(0)) << "),"
        << registerName(srcRegIdx(1));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VsIndexMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(srcRegIdx(2)) << "[" << uint16_t(vs3ElemIdx) << "], "
        << '(' << registerName(srcRegIdx(0)) << "), "
        << registerName(srcRegIdx(1)) << "[" << uint16_t(vs2ElemIdx) << "]";
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string
VMvWholeMacroInst::generateDisassembly(Addr pc,
    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        registerName(srcRegIdx(1));
    return ss.str();
}

std::string
VMvWholeMicroInst::generateDisassembly(Addr pc,
    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        registerName(srcRegIdx(1));
    return ss.str();
}

} // namespace RiscvISA
} // namespace gem5
