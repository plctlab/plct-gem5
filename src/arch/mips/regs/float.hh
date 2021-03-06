/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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

#ifndef __ARCH_MIPS_REGS_FLOAT_HH__
#define __ARCH_MIPS_REGS_FLOAT_HH__

#include <cstdint>

namespace gem5
{
namespace MipsISA
{
namespace float_reg
{

enum FPControlRegNums
{
    _F0Idx,
    _F1Idx,
    _F2Idx,
    _F3Idx,
    _F4Idx,
    _F5Idx,
    _F6Idx,
    _F7Idx,
    _F8Idx,
    _F9Idx,
    _F10Idx,
    _F11Idx,
    _F12Idx,
    _F13Idx,
    _F14Idx,
    _F15Idx,
    _F16Idx,
    _F17Idx,
    _F18Idx,
    _F19Idx,
    _F20Idx,
    _F21Idx,
    _F22Idx,
    _F23Idx,
    _F24Idx,
    _F25Idx,
    _F26Idx,
    _F27Idx,
    _F28Idx,
    _F29Idx,
    _F30Idx,
    _F31Idx,
    NumArchRegs,

    _FirIdx = NumArchRegs,
    _FccrIdx,
    _FexrIdx,
    _FenrIdx,
    _FcsrIdx,

    NumRegs,
};

inline constexpr RegId
    F0(FloatRegClass, _F0Idx),
    F1(FloatRegClass, _F1Idx),
    F2(FloatRegClass, _F2Idx),
    F3(FloatRegClass, _F3Idx),
    F4(FloatRegClass, _F4Idx),
    F5(FloatRegClass, _F5Idx),
    F6(FloatRegClass, _F6Idx),
    F7(FloatRegClass, _F7Idx),
    F8(FloatRegClass, _F8Idx),
    F9(FloatRegClass, _F9Idx),
    F10(FloatRegClass, _F10Idx),
    F11(FloatRegClass, _F11Idx),
    F12(FloatRegClass, _F12Idx),
    F13(FloatRegClass, _F13Idx),
    F14(FloatRegClass, _F14Idx),
    F15(FloatRegClass, _F15Idx),
    F16(FloatRegClass, _F16Idx),
    F17(FloatRegClass, _F17Idx),
    F18(FloatRegClass, _F18Idx),
    F19(FloatRegClass, _F19Idx),
    F20(FloatRegClass, _F20Idx),
    F21(FloatRegClass, _F21Idx),
    F22(FloatRegClass, _F22Idx),
    F23(FloatRegClass, _F23Idx),
    F24(FloatRegClass, _F24Idx),
    F25(FloatRegClass, _F25Idx),
    F26(FloatRegClass, _F26Idx),
    F27(FloatRegClass, _F27Idx),
    F28(FloatRegClass, _F28Idx),
    F29(FloatRegClass, _F29Idx),
    F30(FloatRegClass, _F30Idx),
    F31(FloatRegClass, _F31Idx),

    Fir(FloatRegClass, _FirIdx),
    Fccr(FloatRegClass, _FccrIdx),
    Fexr(FloatRegClass, _FexrIdx),
    Fenr(FloatRegClass, _FenrIdx),
    Fcsr(FloatRegClass, _FcsrIdx);

} // namespace float_reg

enum FCSRBits
{
    Inexact = 1,
    Underflow,
    Overflow,
    DivideByZero,
    Invalid,
    Unimplemented
};

enum FCSRFields
{
    Flag_Field = 1,
    Enable_Field = 6,
    Cause_Field = 11
};

const uint32_t MIPS32_QNAN = 0x7fbfffff;
const uint64_t MIPS64_QNAN = 0x7ff7ffffffffffffULL;

} // namespace MipsISA
} // namespace gem5

#endif
