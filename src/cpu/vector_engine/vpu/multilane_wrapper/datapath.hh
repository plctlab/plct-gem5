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

    void get_instruction_info();

    double compute_double_fp_op(double Aitem, double Bitem, long int Mitem,
        double Dstitem, RiscvISA::VectorStaticInst* insn);
    long int compute_double_fp_comp_op(double Aitem, double Bitem,
        RiscvISA::VectorStaticInst* insn);

    float compute_float_fp_op(float Aitem, float Bitem,  int Mitem,
        float Dstitem, RiscvISA::VectorStaticInst* insn);
    int compute_float_fp_comp_op(float Aitem, float Bitem ,
        RiscvISA::VectorStaticInst* insn);

    long int compute_long_int_op(long int Aitem, long int Bitem,
        long int Mitem, long int Dstitem, RiscvISA::VectorStaticInst* insn);
    int compute_int_op(int Aitem, int Bitem,  int Mitem, int Dstitem,
        RiscvISA::VectorStaticInst* insn);

    double compute_cvt_f_x_64_op( long int Bitem, long int Mitem,
        long int Dstitem, RiscvISA::VectorStaticInst* insn);
    float compute_cvt_f_x_32_op( int Bitem, int Mitem, int Dstitem,
        RiscvISA::VectorStaticInst* insn);

    long int compute_cvt_x_f_64_op( double Bitem, long int Mitem,
        double Dstitem, RiscvISA::VectorStaticInst* insn);
    int compute_cvt_x_f_32_op( float Bitem, int Mitem, float Dstitem,
        RiscvISA::VectorStaticInst* insn);

    void startTicking(VectorLane& data_op_unit,
        RiscvISA::VectorStaticInst& insn, uint64_t src_count,
        uint64_t dst_count, uint64_t vsew, uint64_t slide_count,uint64_t src1,
        std::function<void(uint8_t*,uint8_t,bool)> data_callback);

    void stopTicking();

    void evaluate() override;

private:
    //Operation Latency
    uint64_t Oplatency;
    //Number of Vector Lanes
    const uint64_t VectorLanes;

    bool slide_infligh;
    bool vfredsum_done;

    /*********************************
     * Floating point reductions
     */
    bool vfredsum;
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
    bool is_FP;
    bool is_INT;
    bool is_INT_to_FP;
    bool is_FP_to_INT;
    bool is_FP_Comp;
    /*********************************
     * Sources used  by the operation
     */
    bool arith_src2;
    bool arith_src1_src2;
    bool arith_src1_src2_src3;

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
    long int accumInt;
    double accumDp;
    float  accumSp;
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
};

#endif //__CPU_VECTOR_LANE_DATAPATH_HH__
