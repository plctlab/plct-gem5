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

#include "cpu/vector_engine/vpu/multilane_wrapper/vector_lane.hh"

#include <cassert>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>

#include "arch/riscv/insts/vector_static_inst.hh"
#include "cpu/vector_engine/vmu/read_timing_unit.hh"
#include "cpu/vector_engine/vmu/write_timing_unit.hh"
#include "cpu/vector_engine/vpu/multilane_wrapper/datapath.hh"
#include "debug/VectorEngineInfo.hh"
#include "debug/VectorLane.hh"
#include "params/VectorLane.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"

VectorLane::VectorLane(const VectorLaneParams *p ) :
    SimObject(p),lane_id(p->lane_id),occupied(false),
    srcAReader(p->srcAReader), srcBReader(p->srcBReader),
    srcMReader(p->srcMReader),dstReader(p->dstReader), dstWriter(p->dstWriter),
    dataPath(p->dataPath)
{
    DPRINTF(VectorEngineInfo,"Created a new Cluster with id: %d\n",lane_id);
}

VectorLane::~VectorLane()
{
}

bool
VectorLane::isOccupied()
{
    return occupied;
}

void
VectorLane::issue(VectorEngine& vector_wrapper,
    RiscvISA::VectorStaticInst& insn,
    VectorDynInst *dyn_insn, ExecContextPtr& xc,uint64_t src1,
    uint64_t vtype,uint64_t vl,
    std::function<void(Fault fault)> done_callback)
{
    assert(!occupied);
    occupied = true;

    vectorwrapper = &vector_wrapper;

    // 0 = 8-bit , 1 = 16-bit , 2 = 32-bit , 3 = 64-bit , 4 = 128-bit
    uint64_t vsew;
    vsew = vectorwrapper->vector_config->get_vtype_vsew(vtype);
    uint8_t SIZE =  (vsew == 3) ? sizeof(double) :
                    (vsew == 2) ? sizeof(float) :
                    (vsew == 1) ? sizeof(uint16_t) :
                    (vsew == 0) ? sizeof(uint8_t) : 0;
    assert(SIZE != 0);
    assert((vsew == 3) || (vsew == 2)); // Only 64-bit and 32-bit Operations are supported

    //In this moment there are not implemented widening and narrowing,
    //then dst and src are similar sizes
    uint64_t DATA_SIZE = SIZE;
    uint64_t DST_SIZE = SIZE;

    uint64_t addr_src0;
    uint64_t addr_src1;
    uint64_t addr_src2;
    uint64_t addr_Mask;
    uint64_t addr_OldDst;
    bool location;

    bool move_to_core;
    move_to_core = (insn.getName() =="vfmv_fs");

    uint64_t i;
    // Masked operation
    masked_op = (insn.vm()==0);
    // OPIVI, OPIVX , OPFVF and OPMVX formats
    vx_op = (insn.func3()==4) || (insn.func3()==6);
    vf_op = (insn.func3()==5);
    vi_op = (insn.func3()==3);

    //address in bytes
    uint64_t mvl_bits =vectorwrapper->vector_config->get_max_vector_length_bits();
    addr_src0 = (uint64_t)dyn_insn->get_PDst() * mvl_bits / 8;
    addr_src1 = (uint64_t)dyn_insn->get_PSrc1() * mvl_bits / 8;
    addr_src2 = (uint64_t)dyn_insn->get_PSrc2() * mvl_bits / 8;
    addr_Mask = (uint64_t)dyn_insn->get_PMask() * mvl_bits / 8;
    addr_OldDst = (uint64_t)dyn_insn->get_POldDst() * mvl_bits / 8;
    location = 1;

    //Vector operation result is an scalar data.
    bool reduction = (insn.getName() =="vfredsum_vs");
    //how many items will get written to dst by the end of the operation
    uint64_t dst_count;
    uint64_t src1_count;
    uint64_t vl_count = vl;
    uint64_t mvl_element =
        vectorwrapper->vector_config->get_max_vector_length_elem(vsew);

    DPRINTF(VectorLane,"Executing instruction %s in cluster %d, , vl = %d , mvl =  %d\n",
        insn.getName(), lane_id , vl_count,mvl_element);

    /* mvl_element is used to write with "0" the tail elements */
    if (reduction)
    {
        dst_count = 1;
        src1_count = 1;
    } else {
        dst_count = mvl_element;
        src1_count = vl_count;
    }

    //no-ops are fine
    if (vl_count == 0) {
        occupied = false;
        done_callback(NoFault);
        return;
    }

    //Source operands used by the instruction
    arith1Src           = insn.arith1Src();
    arith2Srcs          = insn.arith2Srcs();
    arith3Srcs          = insn.arith3Srcs();
    vector_to_scalar    = insn.VectorToScalar();

    scalar_reg = insn.vd();

    this->Aread = 0;
    this->Bread = 0;
    this->Mread = 0;
    this->Dread = 0;
    this->Dstread =0;

    this->AdataQ.clear();
    this->BdataQ.clear();
    this->MdataQ.clear();
    this->DstdataQ.clear();

    dyn_insn->set_VectorStaticInst(&insn);

    if (move_to_core)
    {
        /*
         * move_to_core refers to the instructions only read the first
         * element of some vector register and send immediately to
         * the scalar reg,such as vfmv_fs and vmv_xs
         */
        srcBReader->initialize(vector_wrapper,1,DATA_SIZE,addr_src2,0,location,
            xc, [dyn_insn,done_callback,xc,DATA_SIZE,vl_count,this]
            (uint8_t*data, uint8_t size, bool done)
        {
            assert(size == DATA_SIZE);
            uint8_t *ndata = new uint8_t[DATA_SIZE];
            memcpy(ndata, data, DATA_SIZE);
            if (DATA_SIZE == 8)
            {
            scalar_data = (uint64_t)((uint64_t*)ndata)[0];
            }
            else if (DATA_SIZE == 4)
                {
                scalar_data = (uint64_t)((uint32_t*)ndata)[0];
                }
            xc->setFloatRegOperandBits(
                dyn_insn->get_VectorStaticInst(),0,scalar_data);
            ++this->Bread;
            DPRINTF(VectorLane,"VMove to Float Register: %d ,data: 0x%x \n",
                scalar_reg ,scalar_data);

            delete data;
            //assert(!done || (this->Bread == count));
            this->occupied = false;
            done_callback(NoFault);
        });
    }
    else
    {
        bool is_slide = insn.is_slide();
        uint64_t slide_count = 0;
        if (is_slide)
        {
            slide_count = (insn.func3() == 3) ? insn.vs1():
                (insn.func3() == 4) ? src1 : (insn.func3() == 6) ? 1 : 0;
            assert(slide_count < vl_count);
        }

        dataPath->startTicking(*this, insn, vl_count, dst_count, vsew,
        slide_count ,src1,
        [dyn_insn,done_callback,xc,mvl_element,vl_count,DST_SIZE,this]
        (uint8_t *data, uint8_t size, bool done)
        {
            assert(size == DST_SIZE);

            if (!vector_to_scalar)
            {
                this->dstWriter->queueData(data);
            }
            else
            {
                /*
                 * vector_to_scalar refers to the instructions which are
                 * executed in the datapath, such as vmfirst_m and vmpopc_m,
                 * There are instructions that also writes to the scalar reg,
                 * however, those instructions only read the first element of
                 * some vector register and send immediately to the scalar reg,
                 * such as vfmv_fs and vmv_xs.
                 */
                this->srcBReader->stop();
                uint8_t *ndata = new uint8_t[DST_SIZE];
                memcpy(ndata, data, DST_SIZE);
                if (DST_SIZE == 8)
                {
                scalar_data = (uint64_t)((uint64_t*)ndata)[0];
                }
                else if (DST_SIZE == 4)
                    {
                    scalar_data = (uint64_t)((uint32_t*)ndata)[0];
                    }
                xc->setIntRegOperand(
                    dyn_insn->get_VectorStaticInst(),0,scalar_data);
                DPRINTF(VectorLane,"Writting Int Register: %d ,data: 0x%x \n"
                    ,scalar_reg ,scalar_data);
                delete data;
                this->occupied = false;
                this->dataPath->stopTicking();
                done_callback(NoFault);
                DPRINTF(VectorLane,"Leaving vector_lane \n");
            }

            if (done){
                int zero_count = mvl_element-vl_count;
                uint8_t * ZeroData = (uint8_t *)malloc(zero_count*DST_SIZE);
                uint64_t zero_data = 0;
                for (int i=0; i<zero_count;i++){
                memcpy(ZeroData+(i*DST_SIZE), (uint8_t*)&zero_data, DST_SIZE);
                }
                for (int i=0; i<zero_count; i++) {
                    uint8_t *ndata = new uint8_t[DST_SIZE];
                    memcpy(ndata, ZeroData+(i*DST_SIZE), DST_SIZE);
                    this->dstWriter->queueData(ndata);
                    if (DST_SIZE==8){ DPRINTF(VectorLane,"queue Data 0x%x \n",
                        *(uint64_t *) ndata );}
                    if (DST_SIZE==4){ DPRINTF(VectorLane,"queue Data 0x%x \n",
                        *(uint32_t *) ndata );}
                    }
                delete [] ZeroData;
            }
        });

        if (!vector_to_scalar)
        {
            dstWriter->initialize(vector_wrapper,dst_count,DST_SIZE,addr_src0,
                location, xc,[done_callback,dst_count,this](bool done)
            {
                ++Dread;
                if (done) {
                    assert(this->Dread == dst_count);
                    this->occupied = false;
                    this->dataPath->stopTicking();
                    done_callback(NoFault);
                }
            });
        }

        if (vi_op | vx_op | vf_op)
        {
            bool imm_unsigned = (insn.getName() == "vsll_vi") || (insn.getName() == "vsrl_vi") || (insn.getName() == "vsra_vi");

            uint64_t immediate;
            immediate = (imm_unsigned) ? (uint64_t)insn.vs1() : ((insn.vs1()>=16) ? (0xfffffffffffffff0 | (uint64_t)insn.vs1()) : (uint64_t)insn.vs1());

            uint64_t scalar_data;
            scalar_data = (vi_op) ? immediate :
                          (vx_op | vf_op) ? src1 : 0;

            DPRINTF(VectorLane,"scalar data  0x%x vl_count %d\n" ,scalar_data, vl_count);

            //create data result buffer
            uint8_t * Ddata = (uint8_t *)malloc(vl_count*DST_SIZE);

            for (i=0; i<vl_count;i++){
                memcpy(Ddata+(i*DST_SIZE), (uint8_t*)&scalar_data, DST_SIZE);
                }

            for (i=0; i<vl_count; ++i) {
                uint8_t *ndata = new uint8_t[DST_SIZE];
                memcpy(ndata, Ddata+(i*DST_SIZE), DST_SIZE);
                this->AdataQ.push_back(ndata);
                if (DATA_SIZE==8){ DPRINTF(VectorLane,"queue Data srcAReader "
                    "0x%x , queue_size = %d \n" , *(uint64_t *) ndata ,
                    this->AdataQ.size());}
                if (DATA_SIZE==4){ DPRINTF(VectorLane,"queue Data srcAReader "
                    "0x%x , queue_size = %d \n" , *(uint32_t *) ndata ,
                    this->AdataQ.size());}
                }
                delete [] Ddata;
        }
        else if (!insn.arith1Src())
        {
            srcAReader->initialize(vector_wrapper,src1_count, DATA_SIZE,
                addr_src1,0,location, xc, [DATA_SIZE,src1_count,this]
                (uint8_t*data, uint8_t size, bool done)
            {
                assert(size == DATA_SIZE);
                uint8_t *ndata = new uint8_t[DATA_SIZE];
                memcpy(ndata, data, DATA_SIZE);
                this->AdataQ.push_back(ndata);
                if (DATA_SIZE==8){ DPRINTF(VectorLane,"queue Data srcAReader "
                    "0x%x , queue_size = %d \n" , *(uint64_t *) ndata ,
                    this->AdataQ.size());}
                if (DATA_SIZE==4){ DPRINTF(VectorLane,"queue Data srcAReader "
                    "0x%x , queue_size = %d \n" , *(uint32_t *) ndata ,
                    this->AdataQ.size());}
                ++this->Aread;
                delete data;
                assert(!done || (this->Aread == src1_count));
            });
        }

        // If the Op is slidedown, we start to read the register file starting
        // from slide_count element, other are not needed
        if (insn.is_slide())
        {
            //bool is_slideup = insn.is_slideup();
            bool is_slidedown = insn.is_slidedown();
            //count = (is_slidedown) ? count - slide_count: count;
            addr_src2 = (is_slidedown) ? addr_src2 + (slide_count * DATA_SIZE):
                addr_src2;
        }

        srcBReader->initialize(vector_wrapper,vl_count, DATA_SIZE, addr_src2,0,
            location, xc, [DATA_SIZE,vl_count,this]
            (uint8_t*data, uint8_t size, bool done)
        {
            assert(size == DATA_SIZE);
            uint8_t *ndata = new uint8_t[DATA_SIZE];
            memcpy(ndata, data, DATA_SIZE);
            this->BdataQ.push_back(ndata);
            if (DATA_SIZE==8){ DPRINTF(VectorLane,"queue Data srcBReader "
                "0x%x , queue_size = %d \n" , *(uint64_t *) ndata ,
                this->BdataQ.size());}
            if (DATA_SIZE==4){ DPRINTF(VectorLane,"queue Data srcBReader "
                "0x%x , queue_size = %d \n" , *(uint32_t *) ndata ,
                this->BdataQ.size());}
            ++this->Bread;
            delete data;
            assert(!done || (this->Bread == vl_count));
        });

        if (masked_op)
        {
            //DPRINTF(VectorLane,"Reading Source M \n" );
            srcMReader->initialize(vector_wrapper,vl_count,DATA_SIZE,addr_Mask,
                0,location, xc, [addr_Mask,DATA_SIZE,vl_count,this]
                (uint8_t*data, uint8_t size, bool done)
            {
                assert(size == DATA_SIZE);
                uint8_t *ndata = new uint8_t[DATA_SIZE];
                memcpy(ndata, data, DATA_SIZE);
                this->MdataQ.push_back(ndata);
                if (DATA_SIZE==8){ DPRINTF(VectorLane,"queue Data MaskReader "
                    "0x%x , queue_size = %d \n" , *(uint64_t *) ndata ,
                    this->MdataQ.size());}
                if (DATA_SIZE==4){ DPRINTF(VectorLane,"queue Data MaskReader "
                    "0x%x , queue_size = %d \n" , *(uint32_t *) ndata ,
                    this->MdataQ.size());}
                ++this->Mread;
                delete data;
                assert(!done || (this->Mread == vl_count));
            });
        }

        if ((!reduction & (masked_op))| arith3Srcs | is_slide)
            {
            //DPRINTF(VectorLane,"Reading Source DstOld \n" );
            //Leemos el old detination para el caso de mask Op
            dstReader->initialize(vector_wrapper,vl_count,DATA_SIZE,
                addr_OldDst,0,location,xc,[addr_OldDst,DATA_SIZE,vl_count,this]
                (uint8_t*data, uint8_t size, bool done)
            {
                assert(size == DATA_SIZE);
                uint8_t *ndata = new uint8_t[DATA_SIZE];
                memcpy(ndata, data, DATA_SIZE);
                this->DstdataQ.push_back(ndata);
                if (DATA_SIZE==8){ DPRINTF(VectorLane,"queue Data dstReader "
                    "0x%x , queue_size = %d \n" , *(uint64_t *) ndata ,
                    this->DstdataQ.size());}
                if (DATA_SIZE==4){ DPRINTF(VectorLane,"queue Data dstReader "
                    "0x%x , queue_size = %d \n" , *(uint32_t *) ndata ,
                    this->DstdataQ.size());}
                ++this->Dstread;
                delete data;
                assert(!done || (this->Dstread == vl_count));
            });
            }
    }
}


VectorLane *
VectorLaneParams::create()
{
    return new VectorLane(this);
}

