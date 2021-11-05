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

#include "cpu/vector_engine/vmu/vector_mem_unit.hh"

#include <cassert>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>

#include "base/logging.hh"
#include "debug/VectorMemUnit.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"

VectorMemUnit::VectorMemUnit(const VectorMemUnitParams *p) :
    SimObject(p),
    occupied(false),
    memReader(p->memReader),
    memReader_addr(p->memReader_addr),
    memWriter(p->memWriter)
{
}

VectorMemUnit::~VectorMemUnit()
{
}

bool VectorMemUnit::isOccupied()
{
    return occupied;
}

void VectorMemUnit::issue(VectorEngine& vector_wrapper,
    RiscvISA::VectorStaticInst& insn,VectorDynInst *dyn_insn,
    ExecContextPtr& xc, uint64_t src1,uint64_t src2,uint64_t vtype,
    uint64_t vl,std::function<void(Fault fault)> done_callback)
{
    /**
     * ========================================================
     *  Vector Load/Store Addressing Modes   - version 1.0
     * ========================================================
     * mop [1:0] encoding for loads
     *      0 0 unit-stride                 VLE<EEW>
     *      0 1 indexed-unordered           VLUXEI<EEW>
     *      1 0 strided                     VLSE<EEW>
     *      1 1 indexed-ordered             VLOXEI<EEW>
     *
     * mop [1:0] encoding for stores
     *      0 0 unit-stride                 VSE<EEW>
     *      0 1 indexed-unordered           VSUXEI<EEW>
     *      1 0 strided                     VSSE<EEW>
     *      1 1 indexed-ordered             VSOXEI<EEW>
     */

    /**
     * lumop [4:0]
     * 0 0 0 0 0 unit-stride load
     * 0 1 0 0 0 unit-stride, whole register load
     * 0 1 0 1 1 unit-stride, mask load, EEW=8
     * 1 0 0 0 0 unit-stride fault-only-first
     * x x x x x reserved
     *
     * sumop [4:0]
     * 0 0 0 0 0 unit-stride store
     * 0 1 0 0 0 unit-stride, whole register store
     * 0 1 0 1 1 unit-stride, mask store, EEW=8
     * x x x x x reserved
     */

    assert(!occupied);
    occupied = true;

    vectorwrapper = &vector_wrapper;



    uint8_t mop = insn.mop();
    uint8_t lumop = insn.lumop();
    uint8_t sumop = insn.sumop();

    bool indexed = (mop == MopType::indexed_unordered ||
                    mop == MopType::indexed_ordered );
    bool strided = (mop == MopType::strided);
    // bool ordered = (mop == MopType::indexed_ordered);
    uint64_t stride =  (strided) ? src2 : 1;

    std::stringstream mem_mop;
    switch (mop)
    {
    case MopType::unit_stride:
        mem_mop << "unit_stride ";
        break;
    case MopType::indexed_unordered:
        mem_mop << "indexed-unordered ";
        break;
    case MopType::strided:
        mem_mop << "strided(" << stride << ") ";
        break;
    case MopType::indexed_ordered:
        mem_mop << "indexed_ordered ";
        break;
    default:
        panic("not supported mop");
    }
    if (insn.isLoad()) {
        switch (static_cast<LumopType>(lumop))
        {
        case LumopType::unit_stride_load:
            mem_mop << "";
            /* code */
            break;
        case LumopType::whole_reg:
            mem_mop << "whole register ";
            break;
        case LumopType::mask:
            mem_mop << "mask ";
            break;
        case LumopType::fault_only_first:
            mem_mop << "fault only first ";
            break;
        default:
            break;
        }
    } else if (insn.isStore()) {
        switch (static_cast<SumopType>(sumop))
        {
        case SumopType::unit_stride_load:
            mem_mop << "";
            /* code */
            break;
        case SumopType::whole_reg:
            mem_mop << "whole register ";
            break;
        case SumopType::mask:
            mem_mop << "mask ";
            break;
        default:
            break;
        }
    }

    uint64_t vl_count = vl;
    if (insn.isLoad() &&
        static_cast<LumopType>(lumop) == LumopType::whole_reg) {
        vl_count = vectorwrapper->vector_config->get_mvl_lmul1_bits() / 8;
        DPRINTF(VectorMemUnit, "vl: %d, mvl: %d", vl, vl_count);
    }
    else if (insn.isStore() &&
        static_cast<SumopType>(sumop) == SumopType::whole_reg) {
        vl_count = vectorwrapper->vector_config->get_mvl_lmul1_bits() / 8;
        DPRINTF(VectorMemUnit, "vl: %d, mvl: %d", vl, vl_count);
    }
    uint64_t vsew = vectorwrapper->vector_config->get_vtype_sew(vtype);
    /* destination data type size in bytes */
    uint8_t DST_SIZE = vsew/8;
    assert(DST_SIZE != 0);
    //If vl_count == 0 then callback, means that the VL = 0
    if (vl_count == 0) {
        occupied = false;
        done_callback(NoFault);
        return;
    }

    uint64_t mem_addr0;
    bool  location0;
    uint64_t mem_addr;
    bool  location;

    uint64_t mvl_bits =
        vectorwrapper->vector_config->get_max_vector_length_bits(vtype);
    uint64_t mvl_elem =
        vectorwrapper->vector_config->get_max_vector_length_elem(vtype);

    if (insn.isLoad())
    {
        
        mem_addr0 = (uint64_t)dyn_insn->get_renamed_dst() * mvl_bits / 8;
        location0 = 1; // 1 Vecor Register

        DPRINTF(VectorMemUnit,"Vector Load %s to Register v%d, vl:%lu\n",
        mem_mop.str(),dyn_insn->get_renamed_dst() ,vl_count);

        //NOTE: need to initialize the writer BEFORE the reader!
        memWriter->initialize(vector_wrapper,mvl_elem,DST_SIZE,mem_addr0,
            0,1,location0, xc,[done_callback,this](bool done){
            if (done) {
                this->occupied = false;
                done_callback(NoFault);
            }
        });

        mem_addr = src1;
        location = 0;
        DPRINTF(VectorMemUnit,"Vector Load %s from Base Memory Addrs: 0x%lx\n",
             mem_mop.str(),mem_addr );

        if (indexed)
        {
            uint64_t mem_addr1 = (uint64_t)dyn_insn->get_renamed_src2() * mvl_bits/8;
            DPRINTF(VectorMemUnit,"Vector Load Index from Vs2 Reg Addrs: "
                "0x%lx\n",mem_addr1 );
            memReader_addr->initialize(vector_wrapper,vl_count, DST_SIZE,mem_addr1,
            0,1,location0,xc,[DST_SIZE,this]
            (uint8_t*data, uint8_t size, bool done)
            {
                uint8_t *ndata = new uint8_t[DST_SIZE];
                memcpy(ndata, data, DST_SIZE);
                if (DST_SIZE==8) {
                    DPRINTF(VectorMemUnit,"queue Data index addr 0x%x \n",
                        *(uint64_t *) ndata );
                }
                if (DST_SIZE==4) {
                    DPRINTF(VectorMemUnit,"queue Data index addr 0x%x \n",
                        *(uint32_t *) ndata );
                }
                if (DST_SIZE==2) {
                    DPRINTF(VectorMemUnit,"queue Data index addr 0x%x \n",
                        *(uint16_t *) ndata );
                }
                if (DST_SIZE==1) {
                    DPRINTF(VectorMemUnit,"queue Data index addr 0x%x \n",
                        *(uint8_t *) ndata );
                }
                this->memReader->queueData(ndata);
                delete[] data;
            });
        }

        memReader->initialize(vector_wrapper,vl_count, DST_SIZE,mem_addr,
        mop,stride,location, xc,[mvl_elem,vl_count,DST_SIZE,this]
            (uint8_t*data, uint8_t size, bool done)
        {
            uint8_t *ndata = new uint8_t[DST_SIZE];
            memcpy(ndata, data, DST_SIZE);
            if (DST_SIZE==8) {
                DPRINTF(VectorMemUnit,"queue Data 0x%x \n",
                    *(uint64_t *) ndata );
            }
            if (DST_SIZE==4) {
                DPRINTF(VectorMemUnit,"queue Data 0x%x \n"
                ,*(uint32_t *) ndata );
            }
            if (DST_SIZE==2) {
                DPRINTF(VectorMemUnit,"queue Data 0x%x \n"
                ,*(uint16_t *) ndata );
            }
            if (DST_SIZE==1) {
                DPRINTF(VectorMemUnit,"queue Data 0x%x \n"
                ,*(uint8_t *) ndata );
            }
            this->memWriter->queueData(ndata);
            delete[] data;
            // Fill remaining elements with 0
            if (done){
                int zero_count = mvl_elem-vl_count;
                uint8_t * ZeroData =(uint8_t *)malloc(zero_count*DST_SIZE);
                uint64_t zero_data = 0;
                for (int i=0; i<zero_count;i++) {
                    memcpy(ZeroData+(i*DST_SIZE),
                        (uint8_t*)&zero_data,DST_SIZE);
                }
                for (int i=0; i<zero_count; i++) {
                    uint8_t *ndata = new uint8_t[DST_SIZE];
                    memcpy(ndata, ZeroData+(i*DST_SIZE), DST_SIZE);
                    this->memWriter->queueData(ndata);
                    if (DST_SIZE==8) {
                        DPRINTF(VectorMemUnit,"queue Data ""0x%x \n",
                            *(uint64_t *) ndata );
                    }
                    if (DST_SIZE==4) {
                        DPRINTF(VectorMemUnit,"queue Data ""0x%x \n",
                            *(uint32_t *) ndata );
                    }
                    if (DST_SIZE==2) {
                        DPRINTF(VectorMemUnit,"queue Data ""0x%x \n",
                            *(uint16_t *) ndata );
                    }
                    if (DST_SIZE==1) {
                        DPRINTF(VectorMemUnit,"queue Data ""0x%x \n",
                            *(uint8_t *) ndata );
                    }
                }
                delete [] ZeroData;
            }
        });
    } else if (insn.isStore()) {

        mem_addr0 = src1;
        location0 = 0; // 0 = Memoria

        DPRINTF(VectorMemUnit,"Vector Store %s to Memory Addrs: 0x%lx\n",
             mem_mop.str(),mem_addr0 );

        //NOTE: need to initialize the writer BEFORE the reader!
        memWriter->initialize(vector_wrapper,vl_count,DST_SIZE,mem_addr0,
            mop,stride,location0, xc,[done_callback,this](bool done) {
            if (done) {
                this->occupied = false;
                done_callback(NoFault);
            }
        });

        if (indexed)
        {
            uint64_t mem_addr1 = (uint64_t)dyn_insn->get_renamed_src2() * mvl_bits/8;
            location0 = 1; // 1 Vecor Register
            DPRINTF(VectorMemUnit,"Vector Store: Index from vector register v%d\n",
                dyn_insn->get_renamed_src2());
            memReader_addr->initialize(vector_wrapper,vl_count, DST_SIZE,mem_addr1,
            0,1,location0,xc,[DST_SIZE,this]
            (uint8_t*data, uint8_t size, bool done)
            {
                uint8_t *ndata = new uint8_t[DST_SIZE];
                memcpy(ndata, data, DST_SIZE);
                if (DST_SIZE==8) {
                    DPRINTF(VectorMemUnit,"queue Addrs index addr 0x%x \n",
                        *(uint64_t *) ndata );
                }
                if (DST_SIZE==4) {
                    DPRINTF(VectorMemUnit,"queue Addrs index addr 0x%x \n",
                        *(uint32_t *) ndata );
                }
                if (DST_SIZE==2) {
                    DPRINTF(VectorMemUnit,"queue Addrs index addr 0x%x \n",
                        *(uint16_t *) ndata );
                }
                if (DST_SIZE==1) {
                    DPRINTF(VectorMemUnit,"queue Addrs index addr 0x%x \n",
                        *(uint8_t *) ndata );
                }
                this->memWriter->queueAddrs(ndata);
                delete[] data;
            });
        }

        mem_addr = (uint64_t)dyn_insn->get_renamed_src3() * mvl_bits / 8;
        location = 1;
        DPRINTF(VectorMemUnit,"Vector Store: data from vector register v%d\n",
                dyn_insn->get_renamed_src3());

        memReader->initialize(vector_wrapper,vl_count, DST_SIZE,mem_addr,
            0,1,location, xc,[DST_SIZE,this](uint8_t*data, uint8_t size, bool done)
        {
            uint8_t *ndata = new uint8_t[DST_SIZE];
            memcpy(ndata, data, DST_SIZE);
            if (DST_SIZE==8) {
                DPRINTF(VectorMemUnit,"queue Data %X \n",*(uint64_t *)ndata);
            }
            if (DST_SIZE==4) {
                DPRINTF(VectorMemUnit,"queue Data %X \n",*(uint32_t *)ndata);
            }
            if (DST_SIZE==2) {
                DPRINTF(VectorMemUnit,"queue Data %X \n",*(uint16_t *)ndata);
            }
            if (DST_SIZE==1) {
                DPRINTF(VectorMemUnit,"queue Data %X \n",*(uint8_t *)ndata);
            }

            this->memWriter->queueData(ndata);
            delete[] data;
        });
    } else {
        panic("invalid Memory Operation type, insn=%#h \n", insn.machInst);
    }
}


VectorMemUnit *
VectorMemUnitParams::create()
{
    return new VectorMemUnit(this);
}
