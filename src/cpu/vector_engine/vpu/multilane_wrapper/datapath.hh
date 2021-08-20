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

#ifndef __CPU_VECTOR_LANE_DATAPATH_HH__
#define __CPU_VECTOR_LANE_DATAPATH_HH__

#include <cmath>
#include <cstdint>
#include <functional>

#include "arch/riscv/insts/vector_static_inst.hh"
#include "base/statistics.hh"
#include "cpu/vector_engine/vpu/multilane_wrapper/vector_lane.hh"
#include "params/Datapath.hh"
#include "sim/faults.hh"
#include "sim/ticked_object.hh"

class VectorLane;

/**
 * Vector lane datapath
 */
class Datapath : public TickedObject
{
private:
    class TimedFunc {
      public:
        TimedFunc(uint64_t cyclesLeft, std::function<void(void)> execute) :
            cyclesLeft(cyclesLeft), execute(execute) {}
        ~TimedFunc() {}

        uint64_t cyclesLeft;
        std::function<void(void)> execute;
    };

public:
    Datapath(DatapathParams *p);
    ~Datapath();

    void get_instruction_latency();

    double compute_double_fp_op(double Aitem, double Bitem, uint8_t Mitem,
        double Dstitem, RiscvISA::VectorStaticInst* insn);
    double computeDoubleFPReduction(double accumDp,double Bitem,uint8_t Mitem);
    long int compute_double_fp_comp_op(double Aitem, double Bitem,
        RiscvISA::VectorStaticInst* insn);

    

    float compute_float_fp_op(float Aitem, float Bitem,  uint8_t Mitem,
        float Dstitem, RiscvISA::VectorStaticInst* insn);
    float computeSingleFPReduction(float accumSp,float Bitem,uint8_t Mitem);
    int compute_float_fp_comp_op(float Aitem, float Bitem ,
        RiscvISA::VectorStaticInst* insn);

    long int computeLongIntReduction(long int accumDp,long int Bitem,uint8_t Mitem);
    int computeIntReduction(int accumDp,int Bitem,uint8_t Mitem);
    int16_t computeInt16Reduction(int16_t accumDp,int16_t Bitem,uint8_t Mitem);
    int8_t computeInt8Reduction(int8_t accumDp,int8_t Bitem,uint8_t Mitem);

    long int compute_long_int_op(long int Aitem, long int Bitem,
        uint8_t Mitem, long int Dstitem, RiscvISA::VectorStaticInst* insn);
    int compute_int_op(int Aitem, int Bitem,  uint8_t Mitem, int Dstitem,
        RiscvISA::VectorStaticInst* insn);
    int16_t compute_int16_op(int16_t Aitem, int16_t Bitem, uint8_t Mitem, int16_t Dstitem,
        RiscvISA::VectorStaticInst* insn);
    int8_t compute_int8_op(int8_t Aitem, int8_t Bitem, uint8_t Mitem, int8_t Dstitem,
        RiscvISA::VectorStaticInst* insn);

    long int computeLongMaskLogicalOp(bool Aitem, bool Bitem,
        RiscvISA::VectorStaticInst* insn);
    int computeIntMaskLogicalOp(bool Aitem, bool Bitem,
        RiscvISA::VectorStaticInst* insn);

    double compute_cvt_f_x_64_op( long int Bitem, uint8_t Mitem,
        long int Dstitem, RiscvISA::VectorStaticInst* insn);
    float compute_cvt_f_x_32_op( int Bitem, uint8_t Mitem, int Dstitem,
        RiscvISA::VectorStaticInst* insn);

    long int compute_cvt_x_f_64_op( double Bitem, uint8_t Mitem,
        double Dstitem, RiscvISA::VectorStaticInst* insn);
    int compute_cvt_x_f_32_op( float Bitem, uint8_t Mitem, float Dstitem,
        RiscvISA::VectorStaticInst* insn);

    double compute_cvt_f64_x32_op( int Bitem, uint8_t Mitem, int Dstitem,
        RiscvISA::VectorStaticInst* insn);


    void startTicking(VectorLane& data_op_unit,
        RiscvISA::VectorStaticInst& insn, uint64_t src_count,
        uint64_t dst_count, uint64_t vsew, uint64_t slide_count,uint64_t src1,
        std::function<void(uint8_t*,uint8_t,bool)> data_callback);

    template <typename data_type>
    data_type compute_integer_op(data_type Aitem, data_type Bitem,
        uint8_t Mitem, data_type Dstitem, RiscvISA::VectorStaticInst* insn);

    void stopTicking();
    void evaluate() override;
    void regStats() override;

private:
    //Operation Latency
    uint64_t Oplatency;
    //Number of Vector Lanes
    const uint64_t VectorLanes;

    bool slide_infligh;
    bool reduction_first_done;

    /*********************************
     * reductions
     */
    bool reduction;
    bool int_reduction;
    bool fp_reduction;
    /*********************************
     * Slide Operations
     */
    bool vslideup,vslidedown;
    bool vslide1up,vslide1down;
    /*********************************
     * Operations with mask
     */
    bool vmpopc,vmfirst;
    /*********************************
     * Operation type
     */
    bool is_slide;
    bool is_mask_logical;
    bool is_FP;
    bool is_INT;
    bool is_INT_to_FP;
    bool is_FP_to_INT;
    bool is_convert;
    bool is_FP_Comp;
    bool isWidening;
    bool isNarrowing;
    /*********************************
     * Sources used  by the operation
     */
    bool vector_set;
    /*********************************
     * Sources used  by the operation
     */
    bool arith1Src;
    bool arith2Srcs;
    bool arith3Srcs;

    bool op_imm;
        // Masked Operation
    bool vm;

//--------------------------------------------------------------
private:
    //state passed in for current instruction
    VectorLane* vector_lane;
    RiscvISA::VectorStaticInst* insn;
    uint64_t srcCount;
    uint64_t dstCount;
    uint64_t src1;

    uint64_t slide_count;
    uint64_t vsew;
    std::function<void(uint8_t*,uint8_t,bool)> dataCallback;

    //internal state for current instruction
    std::deque<TimedFunc *> vecFuncs;
    std::deque<TimedFunc *> reduceFuncs;
    uint64_t curSrcCount;
    uint64_t curDstCount;

    bool first_elem;
    long int accum_mask;
    double accumDp;
    float  accumSp;
    long int accumLongInt;
    int accumInt;
    int16_t accumInt16;
    int8_t accumInt8;
    int red_SrcCount;
    int slide_SrcCount;
    int srcB_data_slide_count;

    uint64_t DATA_SIZE;
    uint64_t DST_SIZE;

    double srcB_data;
    // Auxiliar Slide array, it should be MVL
    // TODO: Pass the MVL parameter to initialize the size of these vectors
    uint64_t srcB_data_slide_64[256];
    uint32_t srcB_data_slide_32[256];
public:
    // Stats for McPAT
    Stats::Scalar numFP64_operations;
    Stats::Scalar numFP32_operations;
    Stats::Scalar numALU64_operations;
    Stats::Scalar numALU32_operations;
    Stats::Scalar numALU16_operations;
    Stats::Scalar numALU8_operations;
    Stats::Scalar numMUL64_operations;
    Stats::Scalar numMUL32_operations;
    Stats::Scalar numMUL16_operations;
    Stats::Scalar numMUL8_operations;
};

#endif //__CPU_VECTOR_LANE_DATAPATH_HH__