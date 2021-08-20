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

#include "cpu/vector_engine/vpu/multilane_wrapper/datapath.hh"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
#include <queue>

#include "cpu/vector_engine/vpu/multilane_wrapper/func_unit.hh"
#include "cpu/vector_engine/vpu/multilane_wrapper/inst_latency.hh"
#include "debug/Datapath.hh"
#include "debug/VectorEngine.hh"

Datapath::Datapath(
    DatapathParams *p) :
    TickedObject(p), VectorLanes(p->VectorLanes)
{
}

Datapath::~Datapath()
{
}


void
Datapath::startTicking(
    VectorLane& data_op_unit, RiscvISA::VectorStaticInst& insn,
    uint64_t src_count, uint64_t dst_count, uint64_t vsew,
    uint64_t slide_count, uint64_t src1,
    std::function<void(uint8_t*,uint8_t,bool)> data_callback)
{
    assert(!running);

    //copy over the configuration inputs
    this->vector_lane = &data_op_unit;
    this->insn = &insn;
    this->srcCount = src_count;
    this->dstCount = dst_count;
    this->vsew = vsew;
    this->slide_count = slide_count;
    this->src1 = src1;
    this->dataCallback = data_callback;
    //reset all state
    vecFuncs.clear();
    curSrcCount = 0;
    curDstCount = 0;
    first_elem = 0;
    accum_mask = 0;
    accumDp = 0.0;
    accumSp = 0.0;
    accumLongInt = 0;
    accumInt = 0;
    accumInt16 = 0;
    accumInt8 = 0;
    red_SrcCount = 0;
    slide_SrcCount = 0;
    srcB_data_slide_count = 0;
    //reset config
    is_FP           =0;
    is_INT          =0;
    is_convert      =0;
    is_slide        =0;

    vector_set      =0;

    is_mask_logical =0;
    is_FP_Comp      =0;
    is_INT_to_FP    =0;
    is_FP_to_INT    =0;
    reduction_first_done =0;
    slide_infligh   =0;


    std::string operation = this->insn->getName();

    /* Default Operation Latency
     * Note: In case the Operation takes more than 1 cycle to be executed
     * the instruction should be added to inst_latency.hh file to define 
     * the new lantecy
     */
    Oplatency       =1;
    /*get the instruction latency in case it is not 1*/
    get_instruction_latency();

    /* 4 main groups */
    is_FP         = this->insn->isFP();
    is_INT        = this->insn->isInt();
    is_convert    = this->insn->isConvert();
    is_slide      =  this->insn->is_slide();
    assert(is_FP || is_INT || is_slide || is_convert);

    /* Masked Operation? */
    vm = this->insn->vm();

    /* Number of sources used by the isntruction */
    arith1Src     = this->insn->arith1Src();
    arith2Srcs    = this->insn->arith2Srcs();
    arith3Srcs    = this->insn->arith3Srcs();
    /* Conversion between Int<->FP */
    isWidening    = this->insn->isWidening(); // Limited Widening support only for conversions
    isNarrowing   = this->insn->isNarrowing(); // Limited Widening support only for conversions
    is_INT_to_FP    = this->insn->isConvertIntToFP();
    is_FP_to_INT    = this->insn->isConvertFPToInt();

    /* isFPCompare is a FP subset */
    is_FP_Comp  = this->insn->isFPCompare();
    /* INT operation with an immediate */
    op_imm = (this->insn->func3()==3);
    /*reduction*/
    reduction = this->insn->is_reduction();
    /* Floating point reductions */
    fp_reduction = this->insn->is_reduction() && this->insn->isFP();
    /* Int reductions */
    int_reduction = this->insn->is_reduction() && this->insn->isInt();
    /* Vector Slides */
    vslideup =  this->insn->is_slideup();
    vslide1up = (operation == "vslide1up_vx");
    vslidedown = this->insn->is_slidedown();
    vslide1down = (operation == "vslide1down_vx");
    /* Vector Mask-Register Logical Instructions */
    vmpopc = (operation == "vmpopc_m");
    vmfirst = (operation == "vmfirst_m");
    is_mask_logical = this->insn->VectorMaskLogical();

    vector_set = ((operation == "vmerge_vx") || (operation == "vmerge_vi") || (operation == "vfmerge_vf")) && (vm==1);

    DATA_SIZE = vsew/8; // Esto cambiará para widening y narrowing
    DST_SIZE = (isWidening && (vsew == 32)) ? vsew/4 :vsew/8;  // Esto cambiará para widening y narrowing

    //Accumulator for reductions, vmpopc and vmfirst
    accum_mask =-1;

    

    uint64_t pc = this->insn->getPC();
    DPRINTF(Datapath,"Executing inst %s, pc 0x%lx, Oplatency = %d,"
        " DataType = %d-bit\n" , this->insn->getName(),*(uint64_t*)&pc,
        Oplatency , DATA_SIZE*8);
    start();
}

void
Datapath::stopTicking()
{
    assert(running);
    stop();
}

void
Datapath::regStats()
{
    TickedObject::regStats();

    numFP64_operations
        .name(name() + ".numFP64_operations")
        .desc("Number of 64-bit Floating-point operations");
    numFP32_operations
        .name(name() + ".numFP32_operations")
        .desc("Number of 32-bit Floating-point operations");
    numALU64_operations
        .name(name() + ".numALU64_operations")
        .desc("Number of 64-bit ALU operations");
    numALU32_operations
        .name(name() + ".numALU32_operations")
        .desc("Number of 32-bit ALU operations");
    numALU16_operations
        .name(name() + ".numALU16_operations")
        .desc("Number of 16-bit ALU operations");
    numALU8_operations
        .name(name() + ".numALU8_operations")
        .desc("Number of 8-bit ALU operations");
    numMUL64_operations
        .name(name() + ".numMUL64_operations")
        .desc("Number of 64-bit Integer Multiplication operations");
    numMUL32_operations
        .name(name() + ".numMUL32_operations")
        .desc("Number of 32-bit Integer Multiplication operations");
    numMUL16_operations
        .name(name() + ".numMUL16_operations")
        .desc("Number of 16-bit Integer Multiplication operations");
    numMUL8_operations
        .name(name() + ".numMUL8_operations")
        .desc("Number of 8-bit Integer Multiplication operations");
}

void
Datapath::evaluate()
{
    assert(running);
    for (auto vecFunc : vecFuncs) {
        --vecFunc->cyclesLeft;
    }

    if (vecFuncs.size()) {
      TimedFunc * vecFunc = vecFuncs.front();
      if (vecFunc->cyclesLeft == 0) {
          vecFuncs.pop_front();
          vecFunc->execute();
          delete vecFunc;
      }
    }

    uint64_t max_simd_items = VectorLanes*(sizeof(double)/DST_SIZE);
    uint64_t simd_size = std::min(max_simd_items, srcCount-curSrcCount);

    if (simd_size == 0)
    {
        return;
    }

    if ((vm==0)) // Masked Operations uses
    {
        if (reduction)
        {
            if ((vector_lane->MdataQ.size() < simd_size))
            {
                return;
            }
        }
        else if ((vector_lane->MdataQ.size() < simd_size)
            | (vector_lane->DstdataQ.size() < simd_size))
        {
            return;
        }
    }

    if (arith3Srcs) // 3 sources operation
    {
        if ( (vector_lane->AdataQ.size() < simd_size) |
            (vector_lane->BdataQ.size() < simd_size) |
            (vector_lane->DstdataQ.size() < simd_size))
        {
           return;
        }
    }
    else if (reduction)   // Reduction Operation
    {
        if ( (vector_lane->BdataQ.size() < simd_size) ||
            ((vector_lane->AdataQ.size() < 1) && !reduction))
        {
            return;
        }
    }
    else if (arith2Srcs)  // 2 sources operation
    {
        if((vector_lane->AdataQ.size() < simd_size) && vector_set) {
            return;
        } else if (((vector_lane->AdataQ.size() < simd_size) |
            (vector_lane->BdataQ.size() < simd_size) ) && !vector_set )
        {
            return;
        }
    }
    else if (is_slide)
    {
        DPRINTF(Datapath," Is slide \n");
        if (( vector_lane->BdataQ.size() < simd_size ) |
            slide_infligh | (vector_lane->DstdataQ.size() < simd_size))
        {
            return;
        }
    }
    else if (arith1Src)  // 1 source operation (src2)
    {
        if ( vector_lane->BdataQ.size() < simd_size )
        {
            return;
        }
    }

    uint8_t * Adata = (uint8_t *)malloc(simd_size*DATA_SIZE);
    uint8_t * Bdata = (uint8_t *)malloc(simd_size*DATA_SIZE);
    uint8_t * Mdata = (uint8_t *)malloc(simd_size);
    uint8_t * Dstdata = (uint8_t *)malloc(simd_size*DATA_SIZE);

    for (uint64_t i=0; i<simd_size; ++i) {

        /*
         * Reduction instruction
         */
        if (reduction)
        {
            if (vector_lane->AdataQ.size() > 0)
            {
                if (fp_reduction)
                {
                    uint8_t *Aitem = vector_lane->AdataQ.front();
                    memcpy(Adata+(i*DATA_SIZE), Aitem, DATA_SIZE);
                    vector_lane->AdataQ.pop_front();
                    delete[] Aitem;

                    if (vsew == 64) {
                        double Accitem = (double)((double*)Adata)[i] ;
                        accumDp = Accitem;
                    } else {
                        float Accitem = (float)((float*)Adata)[i] ;
                        accumSp = Accitem;
                    }
                    reduction_first_done=1;
                } else if (int_reduction) {
                    uint8_t *Aitem = vector_lane->AdataQ.front();
                    memcpy(Adata+(i*DATA_SIZE), Aitem, DATA_SIZE);
                    vector_lane->AdataQ.pop_front();
                    delete[] Aitem;

                    if (vsew == 64) {
                        uint64_t Accitem = (long int)((long int*)Adata)[i] ;
                        accumLongInt = Accitem;
                    } else if (vsew == 32) {
                        uint32_t Accitem = (int)((int*)Adata)[i] ;
                        accumInt = Accitem;
                    } else if (vsew == 16) {
                        int16_t Accitem = (int16_t)((int16_t*)Adata)[i] ;
                        accumInt16 = Accitem;
                    } else if (vsew == 8) {
                        int8_t Accitem = (int8_t)((int8_t*)Adata)[i] ;
                        accumInt8 = Accitem;
                    }
                    reduction_first_done=1;
                }
            }
        /*
         * If aritmetic instruction with only 1 source means that src1 is not used
         */
        } else if (!arith1Src)
        {
            uint8_t *Aitem = vector_lane->AdataQ.front();
            memcpy(Adata+(i*DATA_SIZE), Aitem, DATA_SIZE);
            vector_lane->AdataQ.pop_front();
            delete[] Aitem;
        }

        /*
         * Src2 is always ised by the arithmetic instructions
         */
        if(!vector_set)
        {
            uint8_t *Bitem = vector_lane->BdataQ.front();
            memcpy(Bdata+(i*DATA_SIZE), Bitem, DATA_SIZE);
            vector_lane->BdataQ.pop_front();
            delete[] Bitem;
        }
        /*
         * Mask register used by the instruction
         */
        if (vm==0)
        {
            uint8_t *Mitem = vector_lane->MdataQ.front();
            memcpy(Mdata+(i*1), Mitem, 1);
            vector_lane->MdataQ.pop_front();
            delete[] Mitem;
        }
        /*
         * Dstitem is used to read the old destination register.
         * This is used when:
         *       -the instruction uses the mask register meaning that it is neecesary
         *        to write the old element when the mask is 0 in the corresponding possition
         *       -the instruction uses 3 sources
         *       -the instruction is slide, the shifted position should be filled by the old
         *        destination
         * When reduction is enable, masked Op does not read old destination
         */
        if (((vm==0) & !reduction) | arith3Srcs | is_slide)
        {
            uint8_t *Dstitem = vector_lane->DstdataQ.front();
            memcpy(Dstdata+(i*DATA_SIZE), Dstitem, DATA_SIZE);
            vector_lane->DstdataQ.pop_front();
            delete[] Dstitem;
        }
    }

    // need to track how many we've read in case
    // the block of elements is partially full
    curSrcCount += simd_size;

    //queue up the vecNode results for the given latency
    vecFuncs.push_back(new TimedFunc(Oplatency,
        [this,simd_size,Adata,Bdata,Mdata,Dstdata]() {

        //create data result buffer
        uint8_t * Ddata = (uint8_t *)malloc(simd_size*DST_SIZE);

        if (is_slide)
        {
            for (int i=0; i<simd_size; ++i)
            {
                if (vsew == 64)
                {
                    uint64_t Bitem = ((uint64_t*)Bdata)[i] ;
                    srcB_data_slide_64[srcB_data_slide_count] = Bitem;
                }
                else
                {
                    uint32_t Bitem = ((uint32_t*)Bdata)[i] ;
                    srcB_data_slide_32[srcB_data_slide_count] = Bitem;
                }
                srcB_data_slide_count++;
            }

            for (int i=0; i<simd_size; ++i)
            {
                if (vsew == 64)
                {
                    uint64_t slide_1element = src1;
                    uint8_t Mitem = ((uint8_t*)Mdata)[i];
                    uint64_t Dstitem = ((uint64_t*)Dstdata)[i] ;
                    uint64_t Ditem;
                    if (vslideup | vslide1up)
                    {
                        if (vslide1up && (slide_SrcCount == 0))
                            Ditem = slide_1element;
                        else if (slide_SrcCount>=slide_count)
                            Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
                                srcB_data_slide_64[slide_SrcCount-slide_count]:
                                Dstitem;
                        else
                            Ditem = Dstitem;

                        DPRINTF(Datapath," vslideup: Ditem 0x%x "
                            "Source 0x%x Old= 0x%x ,slide_1element = 0x%x \n",
                            Ditem,
                            srcB_data_slide_64[slide_SrcCount-slide_count],
                            Dstitem , slide_1element);
                        if (vm==0){
                            DPRINTF(Datapath," WB Instruction is "
                                "masked vm(%d), old(0x%x)  \n",Mitem,Dstitem);
                        }
                    }
                   else if (vslidedown | vslide1down)
                    {
                        if (vslide1down && (slide_SrcCount+1 == srcCount))
                            Ditem = slide_1element;
                        else if (slide_SrcCount>=(srcCount-slide_count))
                            Ditem =  0;
                        else
                            Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
                                srcB_data_slide_64[slide_SrcCount] : Dstitem;

                        DPRINTF(Datapath," vslidedown: Ditem 0x%x "
                            "Source 0x%x Old= 0x%x ,slide_1element = 0x%x \n",
                            Ditem, srcB_data_slide_64[slide_SrcCount], Dstitem,
                            slide_1element);
                        if (vm==0) {
                            DPRINTF(Datapath," WB Instruction is "
                                "masked vm(%d), old(0x%x)  \n",Mitem,Dstitem);
                        }
                    }

                    memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem, DST_SIZE);
                    slide_SrcCount++;
                }
                else
                {
                    uint32_t slide_1element = src1;
                    uint8_t Mitem = ((uint8_t*)Mdata)[i];
                    uint32_t Dstitem = ((uint32_t*)Dstdata)[i] ;
                    uint32_t Ditem;
                    if (vslideup | vslide1up)
                    {
                        if (vslide1up && (slide_SrcCount == 0))
                            Ditem = slide_1element;
                        else if (slide_SrcCount>=slide_count)
                            Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
                                srcB_data_slide_32[slide_SrcCount-slide_count]:
                                Dstitem;
                        else
                            Ditem = Dstitem;

                        DPRINTF(Datapath,"vslideup: Ditem 0x%x "
                            "Source 0x%x  Old= 0x%x  \n" ,Ditem,
                            srcB_data_slide_32[slide_SrcCount-slide_count],
                            Dstitem);
                        if (vm==0) {
                            DPRINTF(Datapath,"WB Instruction is "
                                "masked vm(%d), old(0x%x)  \n",Mitem,Dstitem);
                        }
                    }
                    else if (vslidedown | vslide1down)
                    {
                        if (vslide1down && (slide_SrcCount+1 == srcCount))
                            Ditem = slide_1element;
                        else if (slide_SrcCount>=(srcCount-slide_count))
                            Ditem =  0;
                        else
                            Ditem = ((vm==1) || ((vm==0) && (Mitem==1))) ?
                                srcB_data_slide_32[slide_SrcCount] : Dstitem;

                        DPRINTF(Datapath,"vslidedown: Ditem 0x%x "
                            "Source 0x%x  Old= 0x%x  \n" ,Ditem,
                            srcB_data_slide_32[slide_SrcCount], Dstitem);
                        if (vm==0) {
                            DPRINTF(Datapath,"WB Instruction is "
                                "masked vm(%d), old(0x%x)  \n",Mitem,Dstitem);
                        }
                    }

                    memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem, DST_SIZE);
                    slide_SrcCount++;
                }
            }
        }
        else
        {
            for (int i=0; i<simd_size; ++i)
            {
                if (is_FP )  // FLOATING POINT OPERATION
                {
                    if (vsew == 64)
                    {
                        if (fp_reduction)
                        {
                            double Bitem = (double)((double*)Bdata)[i] ;
                            uint8_t Mitem = ((uint8_t*)Mdata)[i];
                            accumDp = computeDoubleFPReduction(accumDp,Bitem,Mitem);
                            red_SrcCount=red_SrcCount + 1;
                        } else {
                            double Aitem = (double)((double*)Adata)[i] ;
                            double Bitem = (double)((double*)Bdata)[i] ;
                            uint8_t Mitem = ((uint8_t*)Mdata)[i];
                            double Dstitem = (double)((double*)Dstdata)[i];
                            if (is_FP_Comp)
                            {
                                long int Ditem = compute_double_fp_comp_op(
                                    Aitem, Bitem, insn);
                                memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                    DST_SIZE);
                            } else {
                                double Ditem = compute_double_fp_op(Aitem,
                                    Bitem, Mitem, Dstitem, insn);
                                memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                    DST_SIZE);
                            }
                        }
                    }
                    else
                    {
                        if (fp_reduction)
                        {
                            float Bitem = (float)((float*)Bdata)[i];
                            uint8_t Mitem = ((uint8_t*)Mdata)[i];
                            accumSp = computeSingleFPReduction(accumSp,Bitem,Mitem);
                            red_SrcCount=red_SrcCount + 1;
                        } else {
                            float Aitem = (float)((float*)Adata)[i];
                            float Bitem = (float)((float*)Bdata)[i];
                            uint8_t Mitem = ((uint8_t*)Mdata)[i];
                            float Dstitem = (float)((float*)Dstdata)[i];
                            if (is_FP_Comp)
                            {
                                int Ditem = compute_float_fp_comp_op(Aitem,
                                    Bitem, insn);
                                memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                    DST_SIZE);
                            } else {
                                float Ditem = compute_float_fp_op(Aitem, Bitem,
                                    Mitem, Dstitem, insn);
                                memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                    DST_SIZE);
                            }
                        }
                    }

                    if (fp_reduction && (red_SrcCount==srcCount))
                    {
                        uint8_t *ndata = new uint8_t[DST_SIZE];
                        if (vsew == 64)
                        {
                            memcpy(ndata, (uint8_t*)&accumDp, DST_SIZE);
                            dataCallback(ndata, DST_SIZE , 0);
                        } else {
                            memcpy(ndata, (uint8_t*)&accumSp, DST_SIZE);
                            dataCallback(ndata, DST_SIZE , 0);
                        }
                    }
                }
                else if (is_INT)
                {
                    if (vsew == 64)
                    {
                        if (vmpopc | vmfirst)
                        {
                            if (vmpopc)
                            {
                                long int Bitem = (0x0000000000000001) &&
                                    (long int)((long int*)Bdata)[i] ;
                                uint8_t Mitem = ((uint8_t*)Mdata)[i];
                                accum_mask = (vm==1) ? accum_mask + Bitem :
                                    (Mitem) ? accum_mask + Bitem : accum_mask;
                                red_SrcCount=red_SrcCount + 1;
                                DPRINTF(Datapath," vmpopc: Source "
                                    "%ld  Acc= %ld  \n" ,Bitem, accum_mask);
                            } else if (vmfirst) {
                                int Bitem = (0x0000000000000001) &&
                                    (long int)((long int*)Bdata)[i] ;
                                uint8_t Mitem = ((uint8_t*)Mdata)[i];

                                first_elem = (Bitem == 0x0000000000000001);
                                accum_mask = ((vm==1) || ((vm==0) && (Mitem==1)))
                                        ? (first_elem) ? red_SrcCount :
                                        accum_mask : accum_mask;

                                if (first_elem)
                                {
                                    DPRINTF(Datapath," vmfirst: "
                                        "first active element found at %ld"
                                        " position \n" ,accum_mask);
                                    break;
                                }
                                red_SrcCount=red_SrcCount + 1;
                            }
                        } else if(is_mask_logical) {
                            bool Aitem = (bool)((bool*)Adata)[i];
                            bool Bitem = (bool)((bool*)Bdata)[i];
                            long int Ditem = computeLongMaskLogicalOp(Aitem,Bitem,insn);
                            memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                DST_SIZE);
                        } else if (int_reduction) {
                            long int Bitem = (long int)((long int*)Bdata)[i] ;
                            uint8_t Mitem = ((uint8_t*)Mdata)[i];
                            accumLongInt = computeLongIntReduction(accumLongInt,Bitem,Mitem);
                            red_SrcCount=red_SrcCount + 1;
                        } else {
                            long int Aitem = (long int)((long int*)Adata)[i];
                            long int Bitem = (long int)((long int*)Bdata)[i];
                            uint8_t Mitem = ((uint8_t*)Mdata)[i];
                            long int Dstitem =
                                (long int)((long int*)Dstdata)[i];
                            long int Ditem = compute_long_int_op(Aitem, Bitem,
                                Mitem, Dstitem, insn);
                            memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                DST_SIZE);
                        }
                    }
                    else if (vsew == 32)
                    {
                        if (vmpopc | vmfirst)
                        {
                            if (vmpopc)
                            {
                                int Bitem = (0x00000001) &&
                                    (int)((int*)Bdata)[i];
                                uint8_t Mitem = ((uint8_t*)Mdata)[i];
                                accum_mask = (vm==1) ? accum_mask + Bitem : (Mitem)
                                     ? accum_mask + Bitem : accum_mask;
                                red_SrcCount=red_SrcCount + 1;
                                DPRINTF(Datapath," vmpopc: Source %d"
                                    " Acc= %d  \n" ,Bitem, accum_mask);

                            } else if (vmfirst) {
                                int Bitem = (0x00000001) &&
                                    (int)((int*)Bdata)[i];
                                uint8_t Mitem = ((uint8_t*)Mdata)[i];

                                first_elem = (Bitem == 0x00000001);
                                accum_mask = ((vm==1) || ((vm==0) && (Mitem==1)))
                                    ? (first_elem) ? red_SrcCount :
                                    accum_mask : accum_mask;

                                if (first_elem)
                                {
                                    DPRINTF(Datapath," vmfirst: "
                                        "first active element found at %d "
                                        "position \n" ,accum_mask);
                                    break;
                                }

                                red_SrcCount=red_SrcCount + 1;
                            }
                        } else if(is_mask_logical) {
                            bool Aitem = (bool)((bool*)Adata)[i];
                            bool Bitem = (bool)((bool*)Bdata)[i];
                            int Ditem = computeIntMaskLogicalOp(Aitem,Bitem,insn);
                            memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                DST_SIZE);
                        } else if (int_reduction) {
                            int Bitem = (int)((int*)Bdata)[i] ;
                            uint8_t Mitem = ((uint8_t*)Mdata)[i];
                            accumInt = computeIntReduction(accumInt,Bitem,Mitem);
                            red_SrcCount=red_SrcCount + 1;
                        } else {
                            int Aitem = (int)((int*)Adata)[i] ;
                            int Bitem = (int)((int*)Bdata)[i] ;
                            uint8_t Mitem = ((uint8_t*)Mdata)[i];
                            int Dstitem = (int)((int*)Dstdata)[i] ;
                            int Ditem = compute_int_op(Aitem, Bitem, Mitem,
                                Dstitem, insn);
                            memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                                DST_SIZE);
                        }
                    }
                    else if (vsew == 16)
                    {
                        if (vmpopc | vmfirst)
                        {
                            if (vmpopc)
                            {
                                uint16_t Bitem = (0x0001) &&
                                    (uint16_t)((uint16_t*)Bdata)[i];
                                uint8_t Mitem = ((uint8_t*)Mdata)[i];
                                accum_mask = (vm == 1) ? accum_mask + Bitem :
                                    (Mitem) ? accum_mask + Bitem : accum_mask;
                                red_SrcCount = red_SrcCount + 1;
                                DPRINTF(Datapath, " vmpopc: Source "
                                    "%ld  Acc= %ld  \n", Bitem, accum_mask);
                            }
                            else if (vmfirst) {
                                int Bitem = (0x0001) &&
                                    (uint16_t)((uint16_t*)Bdata)[i];
                                uint8_t Mitem = ((uint8_t*)Mdata)[i];

                                first_elem = (Bitem == 0x0001);
                                accum_mask = ((vm == 1) || ((vm == 0) && (Mitem == 1)))
                                    ? (first_elem) ? red_SrcCount :
                                    accum_mask : accum_mask;

                                if (first_elem)
                                {
                                    DPRINTF(Datapath, " vmfirst: "
                                        "first active element found at %ld"
                                        " position \n", accum_mask);
                                    break;
                                }
                                red_SrcCount = red_SrcCount + 1;
                            }
                        }
                        else if (is_mask_logical) {
                            bool Aitem = (bool)((bool*)Adata)[i];
                            bool Bitem = (bool)((bool*)Bdata)[i];
                            uint16_t Ditem = computeLongMaskLogicalOp(Aitem, Bitem, insn);
                            memcpy(Ddata + (i * DST_SIZE), (uint8_t*)&Ditem,
                                DST_SIZE);
                        } else if (int_reduction) {
                            int16_t Bitem = (int16_t)((int16_t*)Bdata)[i] ;
                            uint8_t Mitem = ((uint8_t*)Mdata)[i];
                            accumInt16 = computeInt16Reduction(accumInt16,Bitem,Mitem);
                            red_SrcCount=red_SrcCount + 1;
                        } else {
                            uint16_t Aitem = (uint16_t)((uint16_t*)Adata)[i];
                            uint16_t Bitem = (uint16_t)((uint16_t*)Bdata)[i];
                            uint8_t Mitem = ((uint8_t*)Mdata)[i];
                            uint16_t Dstitem =
                                (uint16_t)((uint16_t*)Dstdata)[i];
                            uint16_t Ditem = compute_int16_op(Aitem, Bitem,
                                Mitem, Dstitem, insn);
                            memcpy(Ddata + (i * DST_SIZE), (uint8_t*)&Ditem,
                                DST_SIZE);
                        }
                    }
                    else if (vsew == 8)
                    {
                        if (vmpopc | vmfirst)
                        {
                            if (vmpopc)
                            {
                                int8_t Bitem = (0x01) &&
                                    (int8_t)((int8_t*)Bdata)[i];
                                uint8_t Mitem = ((uint8_t*)Mdata)[i];
                                accum_mask = (vm == 1) ? accum_mask + Bitem :
                                    (Mitem) ? accum_mask + Bitem : accum_mask;
                                red_SrcCount = red_SrcCount + 1;
                                DPRINTF(Datapath, " vmpopc: Source "
                                    "%ld  Acc= %ld  \n", Bitem, accum_mask);
                            }
                            else if (vmfirst) {
                                int Bitem = (0x01) &&
                                    (int8_t)((int8_t*)Bdata)[i];
                                uint8_t Mitem = ((uint8_t*)Mdata)[i];

                                first_elem = (Bitem == 0x01);
                                accum_mask = ((vm == 1) || ((vm == 0) && (Mitem == 1)))
                                    ? (first_elem) ? red_SrcCount :
                                    accum_mask : accum_mask;

                                if (first_elem)
                                {
                                    DPRINTF(Datapath, " vmfirst: "
                                        "first active element found at %ld"
                                        " position \n", accum_mask);
                                    break;
                                }
                                red_SrcCount = red_SrcCount + 1;
                            }
                        }
                        else if (is_mask_logical) {
                            bool Aitem = (bool)((bool*)Adata)[i];
                            bool Bitem = (bool)((bool*)Bdata)[i];
                            long int Ditem = computeLongMaskLogicalOp(Aitem, Bitem, insn);
                            memcpy(Ddata + (i * DST_SIZE), (uint8_t*)&Ditem,
                                DST_SIZE);
                        } else if (int_reduction) {
                            int8_t Bitem = (int8_t)((int8_t*)Bdata)[i] ;
                            uint8_t Mitem = ((uint8_t*)Mdata)[i];
                            accumInt8 = computeInt8Reduction(accumInt8,Bitem,Mitem);
                            red_SrcCount=red_SrcCount + 1;
                        } else {
                            int8_t Aitem = (int8_t)((int8_t*)Adata)[i];
                            int8_t Bitem = (int8_t)((int8_t*)Bdata)[i];
                            uint8_t Mitem = ((uint8_t*)Mdata)[i];
                            int8_t Dstitem =
                                (int8_t)((int8_t*)Dstdata)[i];
                            int8_t Ditem = compute_int8_op(Aitem, Bitem,
                                Mitem, Dstitem, insn);
                            memcpy(Ddata + (i * DST_SIZE), (uint8_t*)&Ditem,
                                DST_SIZE);
                        }
                    }

                    if (int_reduction && (red_SrcCount==srcCount))
                    {
                        uint8_t *ndata = new uint8_t[DST_SIZE];
                        if (vsew == 64) {
                            memcpy(ndata, (uint8_t*)&accumLongInt, DST_SIZE);
                            dataCallback(ndata, DST_SIZE , 0);
                        } else if (vsew == 32) {
                            memcpy(ndata, (uint8_t*)&accumInt, DST_SIZE);
                            dataCallback(ndata, DST_SIZE , 0);
                        } else if (vsew == 16) {
                            memcpy(ndata, (uint8_t*)&accumInt16, DST_SIZE);
                            dataCallback(ndata, DST_SIZE , 0);
                        } else if (vsew == 8) {
                            memcpy(ndata, (uint8_t*)&accumInt8, DST_SIZE);
                            dataCallback(ndata, DST_SIZE , 0);
                        }
                    }
                }
                else if (is_INT_to_FP)
                {
                    
                    if (isNarrowing && (vsew == 64))
                    {
                        assert(isNarrowing);
                    } else if (vsew == 64) {
                        long int Bitem = (long int)((long int*)Bdata)[i];
                        uint8_t Mitem = ((uint8_t*)Mdata)[i];
                        long int Dstitem = (long int)((long int*)Dstdata)[i];
                        double Ditem = compute_cvt_f_x_64_op(Bitem, Mitem,
                            Dstitem, insn);
                        memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem, DST_SIZE);
                    } else if(isWidening && (vsew == 32)){
                        int Bitem = (int)((int*)Bdata)[i] ;
                        uint8_t Mitem = ((uint8_t*)Mdata)[i];
                        int Dstitem = (int)((int*)Dstdata)[i] ;
                        double Ditem = compute_cvt_f64_x32_op( Bitem, Mitem,
                            Dstitem, insn);
                        memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem, DST_SIZE);
                    } else {
                        int Bitem = (int)((int*)Bdata)[i] ;
                        uint8_t Mitem = ((uint8_t*)Mdata)[i];
                        int Dstitem = (int)((int*)Dstdata)[i] ;
                        float Ditem = compute_cvt_f_x_32_op( Bitem, Mitem,
                            Dstitem, insn);
                        memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem, DST_SIZE);
                    } 
                }
                else if (is_FP_to_INT)
                {
                    if (vsew == 64)
                    {
                        double Bitem = (double)((double*)Bdata)[i] ;
                        uint8_t Mitem = ((uint8_t*)Mdata)[i];
                        double Dstitem = (double)((double*)Dstdata)[i] ;
                        long int Ditem = compute_cvt_x_f_64_op( Bitem, Mitem,
                            Dstitem, insn);
                        memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem,
                            DST_SIZE);
                    } else {
                        float Bitem = (float)((float*)Bdata)[i] ;
                        uint8_t Mitem = ((uint8_t*)Mdata)[i];
                        float Dstitem = (float)((float*)Dstdata)[i] ;
                        int Ditem = compute_cvt_x_f_32_op( Bitem, Mitem,
                            Dstitem, insn);
                        memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&Ditem, DST_SIZE);
                    }
                }
            }
        }
    //don't need these anymore
    delete[] Adata;
    delete[] Bdata;
    delete[] Mdata;
    delete[] Dstdata;


    if (!reduction)
    {
        if ( (vmpopc && (red_SrcCount==srcCount)) | (vmfirst & first_elem) |
            (vmfirst & (red_SrcCount==srcCount)) )
        {
            DPRINTF(Datapath," Elements= %d  \n" , accum_mask);
            //WB Data to Int Register  File
            uint8_t *ndata = new uint8_t[DST_SIZE];
            memcpy(ndata, (uint8_t*)&accum_mask, DST_SIZE);
            dataCallback(ndata, DST_SIZE , 0);
        }
        else if (!vmpopc & !vmfirst)
        {
            //WB Data to Vect Register  File
            for (int i=0; i<simd_size; ++i) {
                uint8_t *ndata = new uint8_t[DST_SIZE];
                memcpy(ndata, Ddata+(i*DST_SIZE), DST_SIZE);
                curDstCount++;
                bool _done = (  curDstCount == srcCount );
                dataCallback(ndata, DST_SIZE,_done);
            }
        }
        delete [] Ddata;
    }

    DPRINTF(Datapath," Leaving dataPath\n");

    }));
}

Datapath *
DatapathParams::create()
{
    return new Datapath(this);
}


