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

#include "debug/Datapath.hh"

float
Datapath::compute_float_fp_op(float Aitem, float Bitem, uint8_t Mitem,
    float Dstitem,  RiscvISA::VectorStaticInst* insn)
{
    float Ditem=0;
    std::string operation = insn->getName();
    numFP32_operations = numFP32_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vfadd_vv") | (operation == "vfadd_vf")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem + Bitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %f + %f  = %f  \n",Aitem,
            Bitem, Ditem);
    }

    if ((operation == "vfsub_vv") | (operation == "vfsub_vf")){
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem - Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %f - %f  = %f  \n",Bitem,
            Aitem, Ditem);
    }

    if ((operation == "vfmul_vv") | (operation == "vfmul_vf")){
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem * Bitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %f * %f  = %f  \n",Aitem,
            Bitem, Ditem);
    }

    if ((operation == "vfdiv_vv") || (operation == "vfdiv_vf")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem / Aitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %f / %f  = %f\n" ,Bitem,
            Aitem, Ditem);
    }

    if ((operation == "vfsqrt_v")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? sqrt(Bitem) : Dstitem;
        DPRINTF(Datapath,"WB Instruction = sqrt (%f)   = %f  \n",
            Bitem,Ditem);
    }

    /**************************************************************************
     * Vector Floating-Point MIN/MAX Instructions
     *************************************************************************/

    if ((operation == "vfmin_vv") || (operation == "vfmin_vf")) {
        Ditem = (Aitem < Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %f , %f  = min :%f\n",
            Aitem,Bitem, Ditem);
    }
    if ((operation == "vfmax_vv") || (operation == "vfmax_vf")) {
        Ditem = (Aitem > Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %f , %f  = max :%f\n",
            Aitem,Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Floating-Point Sign-Injection Instructions
     *************************************************************************/

    if ((operation == "vfsgnj_vv") || (operation == "vfsgnj_vf")) {
        Ditem = (Bitem>=0.0) ? fabs(Aitem):-Aitem;
        DPRINTF(Datapath,"WB Instruction = %f , %f  = %f\n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vfsgnjn_vv") || (operation == "vfsgnjn_vf")) {
        Ditem = (Bitem>=0.0) ?-Aitem: fabs(Aitem);
        DPRINTF(Datapath,"WB Instruction = %f , %f  = %f\n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vfsgnjx_vv") || (operation == "vfsgnjx_vf")) {
        Ditem = ((Bitem>=0.0) & (Aitem >= 0.0)) ? Aitem:
            ((Bitem<0.0) & (Aitem < 0.0)) ?  fabs(Aitem) :
            ((Bitem>=0.0) & (Aitem < 0.0)) ? Aitem : -Aitem;
        DPRINTF(Datapath,"WB Instruction = %f , %f  = %f\n",
            Aitem,Bitem, Ditem);
    }

    /**************************************************************************
     * 
     *************************************************************************/

    if ((operation == "vfmerge_vf")) {
        Ditem = (vm==0) ? ((Mitem==1) ? Aitem:Bitem) : Aitem;
        if(vm==0) {
            DPRINTF(Datapath,"WB Instruction = %f : %f  = %f\n",
            Aitem,Bitem,Ditem);
        } else {
            DPRINTF(Datapath,"WB Instruction = %f = %f\n",
            Aitem,Ditem);
        }
        
    }

    if ((operation == "vfmacc_vv") || (operation == "vfmacc_vf")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
            (Aitem * Bitem) + Dstitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %f * %f + %f  = %f\n",
            Aitem,Bitem,Dstitem, Ditem);
    }

    if ((operation == "vfmadd_vv") || (operation == "vfmadd_vf")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
            (Aitem * Dstitem) + Bitem  : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %f * %f + %f  = %f\n",
            Aitem,Dstitem,Bitem, Ditem);
    }

    if (vm==0){
        DPRINTF(Datapath,"WB Instruction is masked vm(%d),"
            " old(%f)  \n",Mitem,Dstitem);
    }

    return Ditem;
}

double
Datapath::compute_double_fp_op(double Aitem, double Bitem,
     uint8_t Mitem, double Dstitem,  RiscvISA::VectorStaticInst* insn)
{
    float Ditem=0;
    std::string operation = insn->getName();
    numFP64_operations = numFP64_operations.value() + 1; // number of 64-bit FP operations

    if ((operation == "vfadd_vv") | (operation == "vfadd_vf")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem + Bitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lf + %lf  = %lf  \n",Aitem,
            Bitem, Ditem);
    }

    if ((operation == "vfsub_vv") | (operation == "vfsub_vf")){
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem - Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lf - %lf  = %lf  \n",Bitem,
            Aitem, Ditem);
    }

    if ((operation == "vfmul_vv") | (operation == "vfmul_vf")){
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem * Bitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lf * %lf  = %lf  \n",Aitem,
            Bitem, Ditem);
    }

    if ((operation == "vfdiv_vv") || (operation == "vfdiv_vf")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem / Aitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lf / %lf  = %lf\n" ,Bitem,
            Aitem, Ditem);
    }

    if ((operation == "vfsqrt_v")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? sqrt(Bitem) : Dstitem;
        DPRINTF(Datapath,"WB Instruction = sqrt (%lf)   = %lf  \n",
            Bitem,Ditem);
    }

    /**************************************************************************
     * Vector Floating-Point MIN/MAX Instructions
     *************************************************************************/

    if ((operation == "vfmin_vv") || (operation == "vfmin_vf")) {
        Ditem = (Aitem < Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %lf , %lf  = min :%lf\n",
            Aitem,Bitem, Ditem);
    }
    if ((operation == "vfmax_vv") || (operation == "vfmax_vf")) {
        Ditem = (Aitem > Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %lf , %lf  = max :%lf\n",
            Aitem,Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Floating-Point Sign-Injection Instructions
     *************************************************************************/

    if ((operation == "vfsgnj_vv") || (operation == "vfsgnj_vf")) {
        Ditem = (Bitem>=0.0) ? fabs(Aitem):-Aitem;
        DPRINTF(Datapath,"WB Instruction = %lf , %lf  = %lf\n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vfsgnjn_vv") || (operation == "vfsgnjn_vf")) {
        Ditem = (Bitem>=0.0) ?-Aitem: fabs(Aitem);
        DPRINTF(Datapath,"WB Instruction = %lf , %lf  = %lf\n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vfsgnjx_vv") || (operation == "vfsgnjx_vf")) {
        Ditem = ((Bitem>=0.0) & (Aitem >= 0.0)) ? Aitem:
            ((Bitem<0.0) & (Aitem < 0.0)) ?  fabs(Aitem) :
            ((Bitem>=0.0) & (Aitem < 0.0)) ? Aitem : -Aitem;
        DPRINTF(Datapath,"WB Instruction = %lf , %lf  = %lf\n",
            Aitem,Bitem, Ditem);
    }

    /**************************************************************************
     * 
     *************************************************************************/

    if ((operation == "vfmerge_vf")) {
        Ditem = (vm==0) ? ((Mitem==1) ? Aitem:Bitem) : Aitem;
        if(vm==0) {
            DPRINTF(Datapath,"WB Instruction = %lf : %lf  = %f\n",
            Aitem,Bitem,Ditem);
        } else {
            DPRINTF(Datapath,"WB Instruction = %lf = %lf\n",
            Aitem,Ditem);
        }
    }

    if ((operation == "vfmacc_vv") || (operation == "vfmacc_vf")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
            (Aitem * Bitem) + Dstitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lf * %lf + %lf  = %lf\n",
            Aitem,Bitem,Dstitem, Ditem);
    }

    if ((operation == "vfmadd_vv") || (operation == "vfmadd_vf")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
            (Aitem * Dstitem) + Bitem  : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lf * %lf + %lf  = %lf\n",
            Aitem,Dstitem,Bitem, Ditem);
    }

    if (vm==0){
        DPRINTF(Datapath,"WB Instruction is masked vm(%d),"
            " old(%f)  \n",Mitem,Dstitem);
    }

    return Ditem;
}

double 
Datapath::computeDoubleFPReduction(double accumDp,double Bitem,uint8_t Mitem)
{
    double reduction;
    std::string operation = insn->getName();
    numFP64_operations = numFP64_operations.value() + 1; // number of 64-bit FP operations

    if ((operation == "vfredsum_vs") || (operation == "vfredosum_vs")) {
         reduction = (vm==1) ? accumDp + Bitem : (Mitem) ? accumDp + Bitem : accumDp;
         DPRINTF(Datapath," Reduction: Source %lf  Acc= %lf\n" ,Bitem, reduction);
    }

    if (operation == "vfredmax_vs") {
         reduction = (vm==1) ? ((accumDp > Bitem) ? accumDp:Bitem) :
                     (Mitem) ? ((accumDp > Bitem) ? accumDp:Bitem) : accumDp;
         DPRINTF(Datapath," Reduction: Source %lf  Max= %lf\n" ,Bitem, reduction);
    }

    if (operation == "vfredmin_vs") {
         reduction = (vm==1) ? ((accumDp < Bitem) ? accumDp:Bitem) :
                     (Mitem) ? ((accumDp < Bitem) ? accumDp:Bitem) : accumDp;
         DPRINTF(Datapath," Reduction: Source %lf  Min= %lf\n" ,Bitem, reduction);
    }

    return reduction;
}

float
Datapath::computeSingleFPReduction(float accumSp,float Bitem,uint8_t Mitem)
{
    float reduction;
    std::string operation = insn->getName();
    numFP32_operations = numFP32_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vfredsum_vs") || (operation == "vfredosum_vs")) {
         reduction = (vm==1) ? accumSp + Bitem : (Mitem) ? accumSp + Bitem : accumSp;
         DPRINTF(Datapath," Reduction: Source %f  Acc= %f\n" ,Bitem, reduction);
    }

    if (operation == "vfredmax_vs") {
         reduction = (vm==1) ? ((accumSp > Bitem) ? accumSp:Bitem) :
                     (Mitem) ? ((accumSp > Bitem) ? accumSp:Bitem) : accumSp;
         DPRINTF(Datapath," Reduction: Source %f  Max= %f\n" ,Bitem, reduction);
    }

    if (operation == "vfredmin_vs") {
         reduction = (vm==1) ? ((accumSp < Bitem) ? accumSp:Bitem) :
                     (Mitem) ? ((accumSp < Bitem) ? accumSp:Bitem) : accumSp;
         DPRINTF(Datapath," Reduction: Source %f  Min= %f\n" ,Bitem, reduction);
    }

    return reduction;
}


int
Datapath::compute_float_fp_comp_op(float Aitem, float Bitem,
    RiscvISA::VectorStaticInst* insn)
{
    int Ditem=0;
    std::string operation = insn->getName();
    numFP32_operations = numFP32_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vmfeq_vv") || (operation == "vmfeq_vf")) {
        Ditem = (Bitem == Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %f == %f  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmfne_vv") || (operation == "vmfne_vf")) {
        Ditem = (Bitem != Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %f != %f  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmflt_vv") || (operation == "vmflt_vf")) {
        Ditem = (Bitem < Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %f < %f  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmfle_vv") || (operation == "vmfle_vf")) {
        Ditem = (Bitem <= Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %f <= %f  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if (operation == "vmfgt_vf") {
        Ditem = (Bitem > Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %f > %f  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if (operation == "vmfge_vf") {
        Ditem = (Bitem >= Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %f >= %f  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    return Ditem;
}

long int
Datapath::compute_double_fp_comp_op(double Aitem, double Bitem,
    RiscvISA::VectorStaticInst* insn)
{
    long int Ditem=0;
    std::string operation = insn->getName();
    numFP64_operations = numFP64_operations.value() + 1; // number of 64-bit FP operations

    if ((operation == "vmfeq_vv") || (operation == "vmfeq_vf")) {
        Ditem = (Bitem == Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %lf == %lf  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmfne_vv") || (operation == "vmfne_vf")) {
        Ditem = (Bitem != Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %lf != %lf  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmflt_vv") || (operation == "vmflt_vf")) {
        Ditem = (Bitem < Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %lf < %lf  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmfle_vv") || (operation == "vmfle_vf")) {
        Ditem = (Bitem <= Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %lf <= %lf  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if (operation == "vmfgt_vf") {
        Ditem = (Bitem > Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %lf > %lf  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if (operation == "vmfge_vf") {
        Ditem = (Bitem >= Aitem) ? 1 : 0;
        DPRINTF(Datapath,"WB Instruction = %lf >= %lf  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    return Ditem;
}

long int
Datapath::compute_long_int_op(long int Aitem, long int Bitem,
    uint8_t Mitem, long int Dstitem,  RiscvISA::VectorStaticInst* insn)
{
    long int Ditem=0;
    std::string operation = insn->getName();
    numALU64_operations = numALU64_operations.value() + 1; // number of 64-bit ALU operations

    if ((operation == "vadd_vv") || (operation == "vadd_vx") || (operation == "vadd_vi")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem + Bitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d + %d  = %d  \n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vsub_vv") || (operation == "vsub_vx") || (operation == "vsub_vi")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem - Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d - %d  = %d  \n"
            ,Bitem,Aitem, Ditem);
    }

    if ((operation == "vmul_vv") || (operation == "vmul_vx")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem * Bitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d * %d  = %d  \n"
            ,Aitem,Bitem, Ditem);
        numMUL64_operations = numMUL64_operations.value() + 1; // number of 64-bit MUL operations
        numALU64_operations = numALU64_operations.value() - 1; // number of 64-bit ALU operations
    }

    if ((operation == "vdiv_vv") || (operation == "vdiv_vx")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem / Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d / %d  = %d  \n"
            ,Bitem,Aitem, Ditem);
    }
    if ((operation == "vrem_vv") || (operation == "vrem_vx")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem % Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d mod %d  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vsll_vv") || (operation == "vsll_vx") || (operation == "vsll_vi")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem << Aitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %x << %x  = %x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vsrl_vv") || (operation == "vsrl_vx") || (operation == "vsrl_vi")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem >> Aitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d >> %d  = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vand_vv") || (operation == "vand_vx") || (operation == "vand_vi")) {
        Ditem = (Bitem & Aitem);
        DPRINTF(Datapath,"WB Instruction = 0x%x & 0x%x  = 0x%x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vor_vv") || (operation == "vor_vx") || (operation == "vor_vi")) {
        Ditem = (Bitem | Aitem);
        DPRINTF(Datapath,"WB Instruction = 0x%x | 0x%x  = 0x%x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vxor_vv") || (operation == "vxor_vx") || (operation == "vxor_vi")) {
        Ditem = (Bitem ^ Aitem);
        DPRINTF(Datapath,"WB Instruction = 0x%x ^ 0x%x  = 0x%x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmerge_vv")  | (operation == "vmerge_vx")  | (operation == "vmerge_vi")) {
        Ditem = (vm==0) ? ((Mitem==1) ? Aitem:Bitem) : Aitem;
        if(vm==0) {
            DPRINTF(Datapath,"WB Instruction = %d : %d  = %d\n",
            Aitem,Bitem,Ditem);
        } else {
            DPRINTF(Datapath,"WB Instruction = %d = %d\n",
            Aitem,Ditem);
        }
    }

    /**************************************************************************
     * Vector Integer Min/Max Instructions
     *************************************************************************/

    if ((operation == "vmin_vv") || (operation == "vmin_vx")) {
        Ditem = (Aitem < Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %d , %d  = min :%d  \n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vminu_vv") || (operation == "vminu_vx")) {
        Ditem = ((uint64_t)Aitem < (uint64_t)Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %d , %d  = min :%d  \n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vmax_vv") || (operation == "vmax_vx")) {
        Ditem = (Aitem > Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %d , %d  = max :%d  \n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vmaxu_vv") || (operation == "vmaxu_vx")) {
        Ditem = ((uint64_t)Aitem < (uint64_t)Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %d , %d  = max :%d  \n",
            Aitem,Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Integer Comparison Instructions
     *************************************************************************/
    if ((operation == "vmseq_vv") || (operation == "vmseq_vx") || (operation == "vmseq_vi")) {
        Ditem = (Bitem == Aitem);
        DPRINTF(Datapath,"WB Instruction = %d == %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmsne_vv") || (operation == "vmsne_vx") || (operation == "vmsne_vi")) {
        Ditem = (Bitem != Aitem);
        DPRINTF(Datapath,"WB Instruction = %d != %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmsltu_vv") || (operation == "vmsltu_vx")) {
        Ditem = (uint64_t)((uint64_t)Bitem < (uint64_t)Aitem);
        DPRINTF(Datapath,"WB Instruction = %d < %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmslt_vv") || (operation == "vmslt_vx")) {
        Ditem = (Bitem < Aitem);
        DPRINTF(Datapath,"WB Instruction = %d < %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmsleu_vv") || (operation == "vmsleu_vx") || (operation == "vmsleu_vi")) {
        Ditem = (uint64_t)((uint64_t)Bitem <= (uint64_t)Aitem);
        DPRINTF(Datapath,"WB Instruction = %d <= %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmsle_vv") || (operation == "vmsle_vx") || (operation == "vmsle_vi")) {
        Ditem = (Bitem <= Aitem);
        DPRINTF(Datapath,"WB Instruction = %d <= %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmsgtu_vx") || (operation == "vmsgtu_vi")) {
        Ditem = (uint64_t)((uint64_t)Bitem > (uint64_t)Aitem);
        DPRINTF(Datapath,"WB Instruction = %d > %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmsgt_vx") || (operation == "vmsgt_vi")) {
        Ditem = (Bitem > Aitem);
        DPRINTF(Datapath,"WB Instruction = %d > %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if (vm==0) {
        DPRINTF(Datapath,"WB Instruction is masked vm(%d), old(%d)"
            "\n",Mitem,Dstitem);
    }

    return Ditem;
}

int
Datapath::compute_int_op(int Aitem, int Bitem, uint8_t Mitem,
        int Dstitem,  RiscvISA::VectorStaticInst* insn)
{
    int Ditem=0;
    std::string operation = insn->getName();
    numALU32_operations = numALU32_operations.value() + 1; // number of 32-bit ALU operations

    if ((operation == "vadd_vv") || (operation == "vadd_vx") || (operation == "vadd_vi")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem + Bitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d + %d  = %d\n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vsub_vv") || (operation == "vsub_vx") || (operation == "vsub_vi")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem - Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d - %d  = %d\n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmul_vv") || (operation == "vmul_vx")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Aitem * Bitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d * %d  = %d\n",
            Aitem,Bitem, Ditem);
        numMUL32_operations = numMUL32_operations.value() + 1; // number of 32-bit MUL operations
        numALU32_operations = numALU32_operations.value() - 1; // number of 32-bit ALU operations
    }

    if ((operation == "vdiv_vv") || (operation == "vdiv_vx")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem / Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d / %d  = %d\n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vrem_vv") || (operation == "vrem_vx")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem % Aitem : Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d mod %d  = %d\n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vsll_vv") || (operation == "vsll_vx") || (operation == "vsll_vi")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem << Aitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %lx << %lx  = %lx\n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vsrl_vv") || (operation == "vsrl_vx") || (operation == "vsrl_vi")) {
        Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ? Bitem >> Aitem :Dstitem;
        DPRINTF(Datapath,"WB Instruction = %d >> %d  = %d\n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vand_vv") || (operation == "vand_vx") || (operation == "vand_vi")) {
        Ditem = (Bitem & Aitem);
        DPRINTF(Datapath,"WB Instruction = 0x%x & 0x%x  = 0x%x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vor_vv") || (operation == "vor_vx") || (operation == "vor_vi")) {
        Ditem = (Bitem | Aitem);
        DPRINTF(Datapath,"WB Instruction = 0x%x | 0x%x  = 0x%x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vxor_vv") || (operation == "vxor_vx") || (operation == "vxor_vi")) {
        Ditem = (Bitem ^ Aitem);
        DPRINTF(Datapath,"WB Instruction = 0x%x ^ 0x%x  = 0x%x  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmerge_vv")  | (operation == "vmerge_vx")  | (operation == "vmerge_vi")) {
        Ditem = (vm==0) ? ((Mitem==1) ? Aitem:Bitem) : Aitem;
        if(vm==0) {
            DPRINTF(Datapath,"WB Instruction = %d : %d  = %d\n",
            Aitem,Bitem,Ditem);
        } else {
            DPRINTF(Datapath,"WB Instruction = %d = %d\n",
            Aitem,Ditem);
        }
    }

    /**************************************************************************
     * Vector Integer Min/Max Instructions
     *************************************************************************/

    if ((operation == "vmin_vv") || (operation == "vmin_vx")) {
        Ditem = (Aitem < Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %d , %d  = min :%d  \n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vminu_vv") || (operation == "vminu_vx")) {
        Ditem = ((uint32_t)Aitem < (uint32_t)Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %d , %d  = min :%d  \n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vmax_vv") || (operation == "vmax_vx")) {
        Ditem = (Aitem > Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %d , %d  = max :%d  \n",
            Aitem,Bitem, Ditem);
    }

    if ((operation == "vmaxu_vv") || (operation == "vmaxu_vx")) {
        Ditem = ((uint32_t)Aitem < (uint32_t)Bitem) ? Aitem:Bitem;
        DPRINTF(Datapath,"WB Instruction = %d , %d  = max :%d  \n",
            Aitem,Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Integer Comparison Instructions
     *************************************************************************/
    if ((operation == "vmseq_vv") || (operation == "vmseq_vx") || (operation == "vmseq_vi")) {
        Ditem = (Bitem == Aitem);
        DPRINTF(Datapath,"WB Instruction = %d == %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmsne_vv") || (operation == "vmsne_vx") || (operation == "vmsne_vi")) {
        Ditem = (Bitem != Aitem);
        DPRINTF(Datapath,"WB Instruction = %d != %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmsltu_vv") || (operation == "vmsltu_vx")) {
        Ditem = (uint32_t)((uint32_t)Bitem < (uint32_t)Aitem);
        DPRINTF(Datapath,"WB Instruction = %d < %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmslt_vv") || (operation == "vmslt_vx")) {
        Ditem = (Bitem < Aitem);
        DPRINTF(Datapath,"WB Instruction = %d < %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmsleu_vv") || (operation == "vmsleu_vx") || (operation == "vmsleu_vi")) {
        Ditem = (uint32_t)((uint32_t)Bitem <= (uint32_t)Aitem);
        DPRINTF(Datapath,"WB Instruction = %d <= %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmsle_vv") || (operation == "vmsle_vx") || (operation == "vmsle_vi")) {
        Ditem = (Bitem <= Aitem);
        DPRINTF(Datapath,"WB Instruction = %d <= %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmsgtu_vx") || (operation == "vmsgtu_vi")) {
        Ditem = (uint32_t)((uint32_t)Bitem > (uint32_t)Aitem);
        DPRINTF(Datapath,"WB Instruction = %d > %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if ((operation == "vmsgt_vx") || (operation == "vmsgt_vi")) {
        Ditem = (Bitem > Aitem);
        DPRINTF(Datapath,"WB Instruction = %d > %d ? = %d  \n",
            Bitem,Aitem, Ditem);
    }

    if (vm==0) {
        DPRINTF(Datapath,"WB Instruction is masked vm(%d), old(%d)"
            "\n",Mitem,Dstitem);
    }
    return Ditem;
}

int16_t
Datapath::compute_int16_op(int16_t Aitem, int16_t Bitem, uint8_t Mitem,
    int16_t Dstitem, RiscvISA::VectorStaticInst* insn)
{
    int16_t Ditem = 0;
    std::string operation = insn->getName();
    numALU16_operations = numALU16_operations.value() + 1; // number of 16-bit ALU operations

    if ((operation == "vadd_vv") || (operation == "vadd_vx") || (operation == "vadd_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem + Bitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d\n",
            Aitem, Bitem, Ditem);
    }

    if ((operation == "vsub_vv") || (operation == "vsub_vx") || (operation == "vsub_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem - Aitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d\n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmul_vv") || (operation == "vmul_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem * Bitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d\n",
            Aitem, Bitem, Ditem);
        numMUL16_operations = numMUL16_operations.value() + 1; // number of 16-bit MUL operations
        numALU16_operations = numALU16_operations.value() - 1; // number of 16-bit ALU operations
    }

    if ((operation == "vdiv_vv") || (operation == "vdiv_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem / Aitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d / %d  = %d\n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vrem_vv") || (operation == "vrem_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem % Aitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d mod %d  = %d\n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vsll_vv") || (operation == "vsll_vx") || (operation == "vsll_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem << Aitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %lx << %lx  = %lx\n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vsrl_vv") || (operation == "vsrl_vx") || (operation == "vsrl_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem >> Aitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d\n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vand_vv") || (operation == "vand_vx") || (operation == "vand_vi")) {
        Ditem = (Bitem & Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x & 0x%x  = 0x%x  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vor_vv") || (operation == "vor_vx") || (operation == "vor_vi")) {
        Ditem = (Bitem | Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x | 0x%x  = 0x%x  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vxor_vv") || (operation == "vxor_vx") || (operation == "vxor_vi")) {
        Ditem = (Bitem ^ Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x ^ 0x%x  = 0x%x  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmerge_vv") | (operation == "vmerge_vx") | (operation == "vmerge_vi")) {
        Ditem = (vm == 0) ? ((Mitem == 1) ? Aitem : Bitem) : Aitem;
        if (vm == 0) {
            DPRINTF(Datapath, "WB Instruction = %d : %d  = %d\n",
                Aitem, Bitem, Ditem);
        }
        else {
            DPRINTF(Datapath, "WB Instruction = %d = %d\n",
                Aitem, Ditem);
        }
    }

    /**************************************************************************
     * Vector Integer Min/Max Instructions
     *************************************************************************/

    if ((operation == "vmin_vv") || (operation == "vmin_vx")) {
        Ditem = (Aitem < Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = min :%d  \n",
            Aitem, Bitem, Ditem);
    }

    if ((operation == "vminu_vv") || (operation == "vminu_vx")) {
        Ditem = ((uint32_t)Aitem < (uint32_t)Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = min :%d  \n",
            Aitem, Bitem, Ditem);
    }

    if ((operation == "vmax_vv") || (operation == "vmax_vx")) {
        Ditem = (Aitem > Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = max :%d  \n",
            Aitem, Bitem, Ditem);
    }

    if ((operation == "vmaxu_vv") || (operation == "vmaxu_vx")) {
        Ditem = ((uint32_t)Aitem < (uint32_t)Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = max :%d  \n",
            Aitem, Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Integer Comparison Instructions
     *************************************************************************/
    if ((operation == "vmseq_vv") || (operation == "vmseq_vx") || (operation == "vmseq_vi")) {
        Ditem = (Bitem == Aitem);
        DPRINTF(Datapath, "WB Instruction = %d == %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsne_vv") || (operation == "vmsne_vx") || (operation == "vmsne_vi")) {
        Ditem = (Bitem != Aitem);
        DPRINTF(Datapath, "WB Instruction = %d != %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsltu_vv") || (operation == "vmsltu_vx")) {
        Ditem = (uint32_t)((uint32_t)Bitem < (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d < %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmslt_vv") || (operation == "vmslt_vx")) {
        Ditem = (Bitem < Aitem);
        DPRINTF(Datapath, "WB Instruction = %d < %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsleu_vv") || (operation == "vmsleu_vx") || (operation == "vmsleu_vi")) {
        Ditem = (uint32_t)((uint32_t)Bitem <= (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d <= %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsle_vv") || (operation == "vmsle_vx") || (operation == "vmsle_vi")) {
        Ditem = (Bitem <= Aitem);
        DPRINTF(Datapath, "WB Instruction = %d <= %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsgtu_vx") || (operation == "vmsgtu_vi")) {
        Ditem = (uint32_t)((uint32_t)Bitem > (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d > %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsgt_vx") || (operation == "vmsgt_vi")) {
        Ditem = (Bitem > Aitem);
        DPRINTF(Datapath, "WB Instruction = %d > %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if (vm == 0) {
        DPRINTF(Datapath, "WB Instruction is masked vm(%d), old(%d)"
            "\n", Mitem, Dstitem);
    }
    return Ditem;
}

int8_t
Datapath::compute_int8_op(int8_t Aitem, int8_t Bitem, uint8_t Mitem,
    int8_t Dstitem, RiscvISA::VectorStaticInst* insn)
{
    int8_t Ditem = 0;
    std::string operation = insn->getName();
    numALU8_operations = numALU8_operations.value() + 1; // number of 8-bit ALU operations

    if ((operation == "vadd_vv") || (operation == "vadd_vx") || (operation == "vadd_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem + Bitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d\n",
            Aitem, Bitem, Ditem);
    }

    if ((operation == "vsub_vv") || (operation == "vsub_vx") || (operation == "vsub_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem - Aitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d\n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmul_vv") || (operation == "vmul_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem * Bitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d\n",
            Aitem, Bitem, Ditem);
        numMUL8_operations = numMUL8_operations.value() + 1; // number of 8-bit MUL operations
        numALU8_operations = numALU8_operations.value() - 1; // number of 8-bit ALU operations
    }

    if ((operation == "vdiv_vv") || (operation == "vdiv_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem / Aitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d / %d  = %d\n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vrem_vv") || (operation == "vrem_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem % Aitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d mod %d  = %d\n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vsll_vv") || (operation == "vsll_vx") || (operation == "vsll_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem << Aitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %lx << %lx  = %lx\n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vsrl_vv") || (operation == "vsrl_vx") || (operation == "vsrl_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem >> Aitem : Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d\n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vand_vv") || (operation == "vand_vx") || (operation == "vand_vi")) {
        Ditem = (Bitem & Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x & 0x%x  = 0x%x  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vor_vv") || (operation == "vor_vx") || (operation == "vor_vi")) {
        Ditem = (Bitem | Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x | 0x%x  = 0x%x  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vxor_vv") || (operation == "vxor_vx") || (operation == "vxor_vi")) {
        Ditem = (Bitem ^ Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x ^ 0x%x  = 0x%x  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmerge_vv") | (operation == "vmerge_vx") | (operation == "vmerge_vi")) {
        Ditem = (vm == 0) ? ((Mitem == 1) ? Aitem : Bitem) : Aitem;
        if (vm == 0) {
            DPRINTF(Datapath, "WB Instruction = %d : %d  = %d\n",
                Aitem, Bitem, Ditem);
        }
        else {
            DPRINTF(Datapath, "WB Instruction = %d = %d\n",
                Aitem, Ditem);
        }
    }

    /**************************************************************************
     * Vector Integer Min/Max Instructions
     *************************************************************************/

    if ((operation == "vmin_vv") || (operation == "vmin_vx")) {
        Ditem = (Aitem < Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = min :%d  \n",
            Aitem, Bitem, Ditem);
    }

    if ((operation == "vminu_vv") || (operation == "vminu_vx")) {
        Ditem = ((uint32_t)Aitem < (uint32_t)Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = min :%d  \n",
            Aitem, Bitem, Ditem);
    }

    if ((operation == "vmax_vv") || (operation == "vmax_vx")) {
        Ditem = (Aitem > Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = max :%d  \n",
            Aitem, Bitem, Ditem);
    }

    if ((operation == "vmaxu_vv") || (operation == "vmaxu_vx")) {
        Ditem = ((uint32_t)Aitem < (uint32_t)Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = max :%d  \n",
            Aitem, Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Integer Comparison Instructions
     *************************************************************************/
    if ((operation == "vmseq_vv") || (operation == "vmseq_vx") || (operation == "vmseq_vi")) {
        Ditem = (Bitem == Aitem);
        DPRINTF(Datapath, "WB Instruction = %d == %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsne_vv") || (operation == "vmsne_vx") || (operation == "vmsne_vi")) {
        Ditem = (Bitem != Aitem);
        DPRINTF(Datapath, "WB Instruction = %d != %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsltu_vv") || (operation == "vmsltu_vx")) {
        Ditem = (uint32_t)((uint32_t)Bitem < (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d < %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmslt_vv") || (operation == "vmslt_vx")) {
        Ditem = (Bitem < Aitem);
        DPRINTF(Datapath, "WB Instruction = %d < %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsleu_vv") || (operation == "vmsleu_vx") || (operation == "vmsleu_vi")) {
        Ditem = (uint32_t)((uint32_t)Bitem <= (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d <= %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsle_vv") || (operation == "vmsle_vx") || (operation == "vmsle_vi")) {
        Ditem = (Bitem <= Aitem);
        DPRINTF(Datapath, "WB Instruction = %d <= %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsgtu_vx") || (operation == "vmsgtu_vi")) {
        Ditem = (uint32_t)((uint32_t)Bitem > (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d > %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsgt_vx") || (operation == "vmsgt_vi")) {
        Ditem = (Bitem > Aitem);
        DPRINTF(Datapath, "WB Instruction = %d > %d ? = %d  \n",
            Bitem, Aitem, Ditem);
    }

    if (vm == 0) {
        DPRINTF(Datapath, "WB Instruction is masked vm(%d), old(%d)"
            "\n", Mitem, Dstitem);
    }
    return Ditem;
}

/*
 * Integer Reductions :Missing cases
 */
int
Datapath::computeIntReduction(int accumInt,int Bitem,uint8_t Mitem)
{
    int reduction;
    std::string operation = insn->getName();
    numALU32_operations = numALU32_operations.value() + 1; // number of 32-bit ALU operations

    if (operation == "vredsum_vs") {
         reduction = (vm==1) ? accumInt + Bitem : (Mitem) ? accumInt + Bitem : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Acc= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredmax_vs") {
         reduction = (vm==1) ? ((accumInt > Bitem) ? accumInt:Bitem) :
                     (Mitem) ? ((accumInt > Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Max= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredmaxu_vs") {
         reduction = (vm==1) ? (((uint32_t)accumInt > (uint32_t)Bitem) ? accumInt:Bitem) :
                     (Mitem) ? (((uint32_t)accumInt > (uint32_t)Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Maxu= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredmin_vs") {
         reduction = (vm==1) ? ((accumInt < Bitem) ? accumInt:Bitem) :
                     (Mitem) ? ((accumInt < Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Min= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredminu_vs") {
         reduction = (vm==1) ? (((uint32_t)accumInt < (uint32_t)Bitem) ? accumInt:Bitem) :
                     (Mitem) ? (((uint32_t)accumInt < (uint32_t)Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Min= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredand_vs") {
         reduction = (vm==1) ? (accumInt & Bitem):
                     (Mitem) ? (accumInt & Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  and= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredor_vs") {
         reduction = (vm==1) ? (accumInt | Bitem):
                     (Mitem) ? (accumInt | Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  or= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredxor_vs") {
         reduction = (vm==1) ? (accumInt ^ Bitem):
                     (Mitem) ? (accumInt ^ Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  xor= %d\n" ,Bitem, reduction);
    }

    return reduction;
}

/*
 * Integer Reductions :Missing cases
 */
int8_t
Datapath::computeInt8Reduction(int8_t accumInt,int8_t Bitem,uint8_t Mitem)
{
    int8_t reduction;
    std::string operation = insn->getName();
    numALU8_operations = numALU8_operations.value() + 1; // number of 8-bit ALU operations

    if (operation == "vredsum_vs") {
         reduction = (vm==1) ? accumInt + Bitem : (Mitem) ? accumInt + Bitem : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Acc= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredmax_vs") {
         reduction = (vm==1) ? ((accumInt > Bitem) ? accumInt:Bitem) :
                     (Mitem) ? ((accumInt > Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Max= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredmaxu_vs") {
         reduction = (vm==1) ? (((uint8_t)accumInt > (uint8_t)Bitem) ? accumInt:Bitem) :
                     (Mitem) ? (((uint8_t)accumInt > (uint8_t)Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Maxu= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredmin_vs") {
         reduction = (vm==1) ? ((accumInt < Bitem) ? accumInt:Bitem) :
                     (Mitem) ? ((accumInt < Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Min= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredminu_vs") {
         reduction = (vm==1) ? (((uint8_t)accumInt < (uint8_t)Bitem) ? accumInt:Bitem) :
                     (Mitem) ? (((uint8_t)accumInt < (uint8_t)Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Min= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredand_vs") {
         reduction = (vm==1) ? (accumInt & Bitem):
                     (Mitem) ? (accumInt & Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  and= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredor_vs") {
         reduction = (vm==1) ? (accumInt | Bitem):
                     (Mitem) ? (accumInt | Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  or= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredxor_vs") {
         reduction = (vm==1) ? (accumInt ^ Bitem):
                     (Mitem) ? (accumInt ^ Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  xor= %d\n" ,Bitem, reduction);
    }

    return reduction;
}

/*
 * Integer Reductions :Missing cases
 */
int16_t
Datapath::computeInt16Reduction(int16_t accumInt,int16_t Bitem,uint8_t Mitem)
{
    int16_t reduction;
    std::string operation = insn->getName();
    numALU16_operations = numALU16_operations.value() + 1; // number of 316bit ALU operations

    if (operation == "vredsum_vs") {
         reduction = (vm==1) ? accumInt + Bitem : (Mitem) ? accumInt + Bitem : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Acc= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredmax_vs") {
         reduction = (vm==1) ? ((accumInt > Bitem) ? accumInt:Bitem) :
                     (Mitem) ? ((accumInt > Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Max= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredmaxu_vs") {
         reduction = (vm==1) ? (((uint16_t)accumInt > (uint16_t)Bitem) ? accumInt:Bitem) :
                     (Mitem) ? (((uint16_t)accumInt > (uint16_t)Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Maxu= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredmin_vs") {
         reduction = (vm==1) ? ((accumInt < Bitem) ? accumInt:Bitem) :
                     (Mitem) ? ((accumInt < Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Min= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredminu_vs") {
         reduction = (vm==1) ? (((uint16_t)accumInt < (uint16_t)Bitem) ? accumInt:Bitem) :
                     (Mitem) ? (((uint16_t)accumInt < (uint16_t)Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  Min= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredand_vs") {
         reduction = (vm==1) ? (accumInt & Bitem):
                     (Mitem) ? (accumInt & Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  and= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredor_vs") {
         reduction = (vm==1) ? (accumInt | Bitem):
                     (Mitem) ? (accumInt | Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  or= %d\n" ,Bitem, reduction);
    }

    if (operation == "vredxor_vs") {
         reduction = (vm==1) ? (accumInt ^ Bitem):
                     (Mitem) ? (accumInt ^ Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %d  xor= %d\n" ,Bitem, reduction);
    }

    return reduction;
}


/*
 * Integer Reductions :Missing cases
 */
long int
Datapath::computeLongIntReduction(long int accumInt,long int Bitem,uint8_t Mitem)
{
    long int reduction;
    std::string operation = insn->getName();
    numALU64_operations = numALU64_operations.value() + 1; // number of 64-bit ALU operations

    if (operation == "vredsum_vs") {
         reduction = (vm==1) ? accumInt + Bitem : (Mitem) ? accumInt + Bitem : accumInt;
         DPRINTF(Datapath," Reduction: Source %ld  Acc= %ld\n" ,Bitem, reduction);
    }

    if (operation == "vredmax_vs") {
         reduction = (vm==1) ? ((accumInt > Bitem) ? accumInt:Bitem) :
                     (Mitem) ? ((accumInt > Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %ld  Max= %ld\n" ,Bitem, reduction);
    }

    if (operation == "vredmaxu_vs") {
         reduction = (vm==1) ? (((uint64_t)accumInt > (uint64_t)Bitem) ? accumInt:Bitem) :
                     (Mitem) ? (((uint64_t)accumInt > (uint64_t)Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %ld  Maxu= %ld\n" ,Bitem, reduction);
    }

    if (operation == "vredmin_vs") {
         reduction = (vm==1) ? ((accumInt < Bitem) ? accumInt:Bitem) :
                     (Mitem) ? ((accumInt < Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %ld  Min= %ld\n" ,Bitem, reduction);
    }

    if (operation == "vredminu_vs") {
         reduction = (vm==1) ? (((uint64_t)accumInt < (uint64_t)Bitem) ? accumInt:Bitem) :
                     (Mitem) ? (((uint64_t)accumInt < (uint64_t)Bitem) ? accumInt:Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %ld  Min= %ld\n" ,Bitem, reduction);
    }

    if (operation == "vredand_vs") {
         reduction = (vm==1) ? (accumInt & Bitem):
                     (Mitem) ? (accumInt & Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %ld  and= %ld\n" ,Bitem, reduction);
    }

    if (operation == "vredor_vs") {
         reduction = (vm==1) ? (accumInt | Bitem):
                     (Mitem) ? (accumInt | Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %ld  or= %ld\n" ,Bitem, reduction);
    }

    if (operation == "vredxor_vs") {
         reduction = (vm==1) ? (accumInt ^ Bitem):
                     (Mitem) ? (accumInt ^ Bitem) : accumInt;
         DPRINTF(Datapath," Reduction: Source %ld  xor= %ld\n" ,Bitem, reduction);
    }

    return reduction;
}

/*
 * Mask Operations
 */

long int 
Datapath::computeLongMaskLogicalOp(bool Aitem, bool Bitem,
        RiscvISA::VectorStaticInst* insn)
{
    long int Ditem=0;
    bool aux;
    std::string operation = insn->getName();

    if ((operation == "vmand_mm")) {
        aux = (Bitem & Aitem);
        DPRINTF(Datapath,"WB Instruction = %d & %d = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmnand_mm")) {
        aux = !(Bitem & Aitem);
        DPRINTF(Datapath,"WB Instruction = !(%d & %d) = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmandnot_mm")) {
        aux = (Bitem & !Aitem);
        DPRINTF(Datapath,"WB Instruction = %d & !%d = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmxor_mm")) {
        aux = (Bitem ^ Aitem);
        DPRINTF(Datapath,"WB Instruction = %d ^ %d = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmor_mm")) {
        aux = (Bitem | Aitem);
        DPRINTF(Datapath,"WB Instruction = %d | %d = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmnor_mm")) {
        aux = !(Bitem | Aitem);
        DPRINTF(Datapath,"WB Instruction = !(%d | %d) = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmornot_mm")) {
        aux = (Bitem | !Aitem);
        DPRINTF(Datapath,"WB Instruction = %d | !%d = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmxnor_mm")) {
        aux = !(Bitem ^ Aitem);
        DPRINTF(Datapath,"WB Instruction = !(%d | %d) = %d  \n",
            Bitem,Aitem, aux);
    }

    Ditem = (uint64_t)aux;
    return Ditem;
}

int
Datapath::computeIntMaskLogicalOp(bool Aitem, bool Bitem,
        RiscvISA::VectorStaticInst* insn)
{
    int Ditem=0;
    bool aux;
    std::string operation = insn->getName();

    if ((operation == "vmand_mm")) {
        aux = (Bitem & Aitem);
        DPRINTF(Datapath,"WB Instruction = %d & %d = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmnand_mm")) {
        aux = !(Bitem & Aitem);
        DPRINTF(Datapath,"WB Instruction = !(%d & %d) = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmandnot_mm")) {
        aux = (Bitem & !Aitem);
        DPRINTF(Datapath,"WB Instruction = %d & !%d = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmxor_mm")) {
        aux = (Bitem ^ Aitem);
        DPRINTF(Datapath,"WB Instruction = %d ^ %d = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmor_mm")) {
        aux = (Bitem | Aitem);
        DPRINTF(Datapath,"WB Instruction = %d | %d = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmnor_mm")) {
        aux = !(Bitem | Aitem);
        DPRINTF(Datapath,"WB Instruction = !(%d | %d) = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmornot_mm")) {
        aux = (Bitem | !Aitem);
        DPRINTF(Datapath,"WB Instruction = %d | !%d = %d  \n",
            Bitem,Aitem, aux);
    }

    if ((operation == "vmxnor_mm")) {
        aux = !(Bitem ^ Aitem);
        DPRINTF(Datapath,"WB Instruction = !(%d | %d) = %d  \n",
            Bitem,Aitem, aux);
    }

    Ditem = (uint64_t)aux;
    return Ditem;
}

double
Datapath::compute_cvt_f_x_64_op( long int Bitem, uint8_t Mitem,
    long int Dstitem,  RiscvISA::VectorStaticInst* insn)
{
    double Ditem=0;
    std::string operation = insn->getName();
    numFP64_operations = numFP64_operations.value() + 1; // number of 64-bit FP operations

    if ((operation == "vfcvt_f_x_v")) {
        Ditem = (double)Bitem;
        DPRINTF(Datapath,"WB Instruction =  cast (%d)  = %0.2lf\n",
            Bitem, Ditem);
    }

    if ((operation == "vfcvt_f_xu_v")) {
        Ditem = (double)(*(uint64_t*)&Bitem);
        DPRINTF(Datapath,"WB Instruction =  cast unsigned (%d)  = %0.2f\n",
            Bitem, Ditem);
    }

    return Ditem;
}

float
Datapath::compute_cvt_f_x_32_op( int Bitem, uint8_t Mitem, int Dstitem,
    RiscvISA::VectorStaticInst* insn)
{
    float Ditem=0;
    std::string operation = insn->getName();
    numFP32_operations = numFP32_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vfcvt_f_x_v")) {
        Ditem = (float)Bitem;
        DPRINTF(Datapath,"WB Instruction =  cast (%d)  = %0.2f\n",
            Bitem, Ditem);
    }

    if ((operation == "vfcvt_f_xu_v")) {
        Ditem = (float)(*(uint32_t*)&Bitem);
        DPRINTF(Datapath,"WB Instruction =  cast unsigned (%d)  = %0.2f\n",
            Bitem, Ditem);
    }

    return Ditem;
}

long int
Datapath::compute_cvt_x_f_64_op( double Bitem, uint8_t Mitem,
    double Dstitem, RiscvISA::VectorStaticInst* insn)
{
    long int Ditem=0;
    std::string operation = insn->getName();
    numFP64_operations = numFP64_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vfcvt_x_f_v")) {
        Ditem = (long int)Bitem;
        DPRINTF(Datapath,"WB Instruction =  cast (%0.2lf) = %d\n",
            Bitem, Ditem);
    }

    if ((operation == "vfcvt_xu_f_v")) {
        Ditem = (unsigned long int)Bitem;
        DPRINTF(Datapath,"WB Instruction =  cast unsigned (%0.2lf) = %d\n",
            Bitem, Ditem);
    }

    return Ditem;
}

int
Datapath::compute_cvt_x_f_32_op( float Bitem, uint8_t Mitem,
    float Dstitem, RiscvISA::VectorStaticInst* insn)
{
    int Ditem=0;
    std::string operation = insn->getName();
    numFP32_operations = numFP32_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vfcvt_x_f_v")) {
        Ditem = (unsigned int)Bitem;
        DPRINTF(Datapath,"WB Instruction =  cast (%0.2f) = %d\n",
            Bitem, Ditem);
    }

    if ((operation == "vfcvt_xu_f_v")) {
        Ditem = (unsigned int)Bitem;
        DPRINTF(Datapath,"WB Instruction =  cast unsigned (%0.2f) = %d\n",
            Bitem, Ditem);
    }

    return Ditem;
}

double
Datapath::compute_cvt_f64_x32_op( int Bitem, uint8_t Mitem, int Dstitem,
    RiscvISA::VectorStaticInst* insn)
{
    double Ditem=0.0;
    std::string operation = insn->getName();
    numFP64_operations = numFP64_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vfwcvt_f_x_v")) {
        Ditem = (double)Bitem;
        DPRINTF(Datapath,"WB Instruction =  cast (%d)  = %0.2f\n",
            Bitem, Ditem);
    }

    return Ditem;
}