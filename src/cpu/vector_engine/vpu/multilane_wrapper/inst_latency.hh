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

// NOTA IMPORTANTE: ESTE ARCHIVO DEBERIA SER ELIMINADO Y ESTA INFORMACIÓN PODERSE 
// PASAR DESDE DECODE, ASI NO TENEMOS QUE HACER UNA DECODIFICACION NUEVAMENTE PARA CADA CASO

//  SE PUEDE ELIMINAR IS_FP IS_INT SI EN STATIC DEFINIMOS ESO .. SABEMOS CUALES SON INT Y FP POR FUNC3 ... PODEMOS HACER GRUPOS
// Y LAS LATENCIAS PUES SE DEFINEN UNAS ESTANDAR PARA CADA TIPO. EN LUGAR DE INDIVIDUAL ....

void
Datapath::get_instruction_latency()
{
    std::string operation = insn->getName();

    /**************************************************************************
     * Floating point Operations
     *************************************************************************/
    if ((operation == "vfadd_vv") | (operation == "vfadd_vf")) {
        Oplatency           = 4;
    }

    if ((operation == "vfsub_vv") | (operation == "vfsub_vf")) {
        Oplatency           = 4;
    }

    if ((operation == "vfmul_vv") | (operation == "vfmul_vf")) {
        Oplatency           = 3;
    }

    if ((operation == "vfdiv_vv") ||(operation == "vfdiv_vf")) {
        Oplatency           = 14;
    }
    if (operation == "vfsqrt_v") {
        Oplatency           = 12;
    }

    /**************************************************************************
     * Vector Floating-Point MIN/MAX Instructions
     *************************************************************************/

    if (   (operation == "vfmin_vv") | (operation == "vfmin_vf")
        || (operation == "vfmax_vv") | (operation == "vfmax_vf")) {
        Oplatency           = 4;
    }

    /**************************************************************************
     * Vector Floating-Point Sign-Injection Instructions
     *************************************************************************/
    /*
    if (   (operation == "vfsgnj_vv") || (operation == "vfsgnj_vf")
        || (operation == "vfsgnjn_vv") || (operation == "vfsgnjn_vf")
        || (operation == "vfsgnjx_vv") || (operation == "vfsgnjx_vf")) {
        Oplatency           = 1;
    }
    */
    /**************************************************************************
     * Floating point reductions
     *************************************************************************/

    if (   (operation == "vfredsum_vs") || (operation == "vfredosum_vs") 
        || (operation == "vfredmax_vs") || (operation == "vfredmin_vs")) {
        Oplatency           = 4;
    }

    /**************************************************************************
     * Vector Floating-Point Compare Instructions
     *************************************************************************/

    if (   (operation == "vmfeq_vv") || (operation == "vmfeq_vf")
        || (operation == "vmfne_vv") || (operation == "vmfne_vf")
        || (operation == "vmflt_vv") || (operation == "vmflt_vf")
        || (operation == "vmfle_vv") || (operation == "vmfle_vf")
        || (operation == "vmfgt_vf")
        || (operation == "vmfge_vf") ) {
        Oplatency           = 4;
    }

    /**************************************************************************
     * 
     *************************************************************************/

    if ((operation == "vfmacc_vv") || (operation == "vfmacc_vf")) {
        Oplatency           = 6;
    }

    if (operation == "vfmadd_vv")   {
        Oplatency           = 6;
    }
    /*
    if (operation == "vfmerge_vf")   {
        Oplatency           = 1;
    }
    */
    /**************************************************************************
     * Single-Width Floating-Point/Integer Type-Convert Instructions
     *************************************************************************/

    if (   (operation == "vfcvt_x_f_v") || (operation == "vfcvt_xu_f_v")
        || (operation == "vfcvt_f_x_v") || (operation == "vfcvt_f_xu_v")) { 
        Oplatency           = 4;
    }

    /**************************************************************************
     * Integer Instructions
     *************************************************************************/
    /*
    if ((operation == "vadd_vv") || (operation == "vadd_vx") || (operation == "vadd_vi")) {
        Oplatency           = 1;
    }

    if ((operation == "vsub_vv") || (operation == "vsub_vx") || (operation == "vsub_vi")) {
        Oplatency           = 1;
    }

    if ((operation == "vmul_vv") || (operation == "vmul_vx")) {
        Oplatency           = 1;
    }
    */
    if ((operation == "vdiv_vv") || (operation == "vdiv_vx")) {
        Oplatency           = 41;
    }

    if ((operation == "vrem_vv") || (operation == "vrem_vx")) {
        Oplatency           = 41;
    }
    /*
    if ((operation == "vsll_vv") || (operation == "vsll_vx") || (operation == "vsll_vi")) {
        Oplatency           = 1;
    }

    if ((operation == "vsrl_vv") || (operation == "vsrl_vx") || (operation == "vsrl_vi")) {
        Oplatency           = 1;
    }

    if ((operation == "vand_vv") || (operation == "vand_vx") || (operation == "vand_vi")) {
        Oplatency           = 1;
    }

    if ((operation == "vor_vv") || (operation == "vor_vx") || (operation == "vor_vi")) {
        Oplatency           = 1;
    }

    if ((operation == "vxor_vv") || (operation == "vxor_vx") || (operation == "vxor_vi")) {
        Oplatency           = 1;
    }

    if ((operation == "vmerge_vv") | (operation == "vmerge_vx") | (operation == "vmerge_vi")) {
        Oplatency           = 1;
    }
    */
    /**************************************************************************
     * Vector Integer Min/Max Instructions
     *************************************************************************/
    /*
    if (   (operation == "vminu_vv") || (operation == "vminu_vx")
        || (operation == "vmin_vv") || (operation == "vmin_vx")
        || (operation == "vmaxu_vv") || (operation == "vmaxu_vx")
        || (operation == "vmax_vv") || (operation == "vmax_vx") ) {
        Oplatency           = 1;
    }
    */
    /**************************************************************************
     * Vector Integer Comparison Instructions
     *************************************************************************/
    /*
    if (   (operation == "vmseq_vv") || (operation == "vmseq_vx") || (operation == "vmseq_vi")
        || (operation == "vmsne_vv") || (operation == "vmsne_vx") || (operation == "vmsne_vi")
        || (operation == "vmsltu_vv") || (operation == "vmsltu_vx")
        || (operation == "vmslt_vv") || (operation == "vmslt_vx")
        || (operation == "vmsleu_vv") || (operation == "vmsleu_vx") || (operation == "vmsleu_vi")
        || (operation == "vmsle_vv") || (operation == "vmsle_vx") || (operation == "vmsle_vi")
        || (operation == "vmsgtu_vx") || (operation == "vmsgtu_vi")
        || (operation == "vmsgt_vx") || (operation == "vmsgt_vi") ) {
        Oplatency           = 1;
    }
    */
    /**************************************************************************
     * Mask Operations
     *************************************************************************/
    /*
    if ((operation == "vmand_mm") || (operation == "vmnand_mm") || (operation == "vmandnot_mm") ||
        (operation == "vmxor_mm") || (operation == "vmor_mm") || (operation == "vmnor_mm") ||
        (operation == "vmornot_mm") || (operation == "vmxnor_mm") ) {
        Oplatency           = 1;
    }
    if (operation == "vmpopc_m") {
        Oplatency           = 1;
    }

    if (operation == "vmfirst_m") {
        Oplatency           = 1;
    }
    */
    /**************************************************************************
     * Slide Operations
     *************************************************************************/

    if ((operation == "vslideup_vi") | (operation == "vslideup_vx")) {
        Oplatency =((slide_count%VectorLanes)== 0) ? 1:slide_count%VectorLanes;
    }
    /*
    if (operation == "vslide1up_vx") {
        Oplatency           = 1;
    }
    */
    if ((operation == "vslidedown_vi") | (operation == "vslidedown_vx")) {
        Oplatency =((slide_count%VectorLanes)== 0) ? 1:slide_count%VectorLanes;
    }
    /*
    if (operation == "vslide1down_vx") {
        Oplatency           = 1;
    }
    */
}