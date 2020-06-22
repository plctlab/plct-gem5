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

#include <cstdint>

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
    ExecContext *xc, uint64_t src1,uint64_t vtype,uint64_t vl,
    std::function<void(Fault fault)> done_callback)
{
    assert(!occupied);
    occupied = true;

    vectorwrapper = &vector_wrapper;

    uint64_t vl_count = vl ;

    //uint64_t vlmul = vt(vtype,0,2);
    uint64_t vsew = vt(vtype,2,3);
    //uint64_t vediv = vt(vtype,5,2);

    uint8_t DST_SIZE = (vsew == 3) ?
        sizeof(double) : (vsew == 2) ? sizeof(float) : 0;
    assert(DST_SIZE != 0);

    uint8_t mop = insn.mop();
    bool gather_op = (mop ==3);

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
        vectorwrapper->vector_csr->get_max_vector_length_bits();
    uint64_t mvl_elem =
        vectorwrapper->vector_csr->get_max_vector_length_elem(vsew);

    if (insn.isLoad())
    {
        mem_addr0 = (uint64_t)dyn_insn->get_PDst() * mvl_bits / 8;
        location0 = 1; // 1 Vecor Register

        DPRINTF(VectorMemUnit,"Vector Load to Register Addrs: 0x%lx size:%lu\n"
            ,mem_addr0 ,vl_count);

        //NOTE: need to initialize the writer BEFORE the reader!
        memWriter->initialize(vector_wrapper,mvl_elem,DST_SIZE,mem_addr0,
            location0, xc,[done_callback,this](bool done){
            if (done) {
                this->occupied = false;
                done_callback(NoFault);
            }
        });

        mem_addr = src1;
        location = 0;
        DPRINTF(VectorMemUnit,"Vector Load from Base Memory Addrs: 0x%lx\n",
            mem_addr );

        if (gather_op)
        {
            uint64_t mem_addr1 = (uint64_t)dyn_insn->get_PSrc2() * mvl_bits/8;
            DPRINTF(VectorMemUnit,"Vector Load Index from Vs2 Reg Addrs: "
                "0x%lx\n",mem_addr1 );
            memReader_addr->initialize(vector_wrapper,vl_count, DST_SIZE,
            mem_addr1,0,location0,xc,[DST_SIZE,this]
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
                this->memReader->queueData(ndata);
                delete[] data;
            });

            memReader->initialize(vector_wrapper,vl_count, DST_SIZE,mem_addr,
                mop,location, xc,[mvl_elem,vl_count,DST_SIZE,this]
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
                    }
                    delete [] ZeroData;
                }
            });
        } else {

        memReader->initialize(vector_wrapper,vl_count, DST_SIZE,mem_addr,mop,
            location, xc,[mvl_elem,vl_count,DST_SIZE,this]
            (uint8_t*data, uint8_t size, bool done)
        {
            uint8_t *ndata = new uint8_t[DST_SIZE];
            memcpy(ndata, data, DST_SIZE);
            if (DST_SIZE==8) {
                DPRINTF(VectorMemUnit,"queue Data 0x%x \n",
                *(uint64_t *) ndata );}
            if (DST_SIZE==4) {
                DPRINTF(VectorMemUnit,"queue Data 0x%x \n",
                *(uint32_t *) ndata );}
            this->memWriter->queueData(ndata);
            delete[] data;
            // Fill remaining elements with 0
            if (done) {
                int zero_count = mvl_elem-vl_count;
                uint8_t * ZeroData = (uint8_t *)malloc(zero_count*DST_SIZE);
                uint64_t zero_data = 0;
                for (int i=0; i<zero_count;i++) {
                memcpy(ZeroData+(i*DST_SIZE), (uint8_t*)&zero_data, DST_SIZE);
                }
                for (int i=0; i<zero_count; i++) {
                    uint8_t *ndata = new uint8_t[DST_SIZE];
                    memcpy(ndata, ZeroData+(i*DST_SIZE), DST_SIZE);
                    this->memWriter->queueData(ndata);
                    if (DST_SIZE==8) {
                        DPRINTF(VectorMemUnit,"queue Data 0x%x \n",
                            *(uint64_t *) ndata );
                    }
                    if (DST_SIZE==4) {
                        DPRINTF(VectorMemUnit,"queue Data 0x%x \n",
                            *(uint32_t *) ndata );
                    }
                    }
                delete [] ZeroData;
            }

        });
        }
    } else if (insn.isStore()) {

        mem_addr0 = src1;
        location0 = 0; // 0 = Memoria

        DPRINTF(VectorMemUnit,"Vector Store to Memory Addrs: 0x%lx ,int reg :"
            " x%d ,count= %lu \n",mem_addr0, dyn_insn->get_PSrc1() , vl_count);

        //NOTE: need to initialize the writer BEFORE the reader!
        memWriter->initialize(vector_wrapper,vl_count,DST_SIZE,mem_addr0,
            location0, xc,[done_callback,this](bool done) {
            if (done) {
                this->occupied = false;
                done_callback(NoFault);
            }
        });

        mem_addr = (uint64_t)dyn_insn->get_PDst() * mvl_bits / 8;
        location = 1;
        DPRINTF(VectorMemUnit,"Vector Store from Register Addrs: "
            "0x%lx\n",mem_addr);

        memReader->initialize(vector_wrapper,vl_count, DST_SIZE,mem_addr,mop,
            location, xc,[DST_SIZE,this](uint8_t*data, uint8_t size, bool done)
        {
            uint8_t *ndata = new uint8_t[DST_SIZE];
            memcpy(ndata, data, DST_SIZE);
            if (DST_SIZE==8) {
                DPRINTF(VectorMemUnit,"queue Data 0x%x \n",*(uint64_t *)ndata);
            }
            if (DST_SIZE==4) {
                DPRINTF(VectorMemUnit,"queue Data 0x%x \n",*(uint32_t *)ndata);
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

