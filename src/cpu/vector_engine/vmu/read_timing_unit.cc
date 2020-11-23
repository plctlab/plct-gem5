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

#include "cpu/vector_engine/vmu/read_timing_unit.hh"

#include <algorithm>
#include <cassert>
#include <cstring>

#include "arch/riscv/insts/vector_static_inst.hh"
#include "base/types.hh"
#include "debug/MemUnitReadTiming.hh"

/**
 * Memory Unit - Read
 */
MemUnitReadTiming::MemUnitReadTiming(MemUnitReadTimingParams *p) :
    TickedObject(p), channel(p->channel), cacheLineSize(p->cacheLineSize),
    VRF_LineSize(p->VRF_LineSize), done(false)
{
}

MemUnitReadTiming::~MemUnitReadTiming()
{
}

void
MemUnitReadTiming::evaluate()
{
    assert(running);
    if (!done) { readFunction(); }
    if (done) {
        done = false;
        stop();
    }
}

void
MemUnitReadTiming::regStats()
{
    TickedObject::regStats();

    Cache_line_r_req
        .name(name() + ".Cache_line_r_req")
        .desc("Number of cache lines read requested");
}

void
MemUnitReadTiming::queueData(uint8_t *data)
{
    assert(running && !done);
    dataQ.push_back(data);
}

void
MemUnitReadTiming::initialize(VectorEngine& vector_wrapper, uint64_t count,
    uint64_t DST_SIZE,uint64_t mem_addr, uint8_t mop,uint64_t stride, bool location,
    ExecContextPtr& xc, std::function<void(uint8_t*,
    uint8_t,bool)> on_item_load) {

    assert(!running && !done);
    assert(count > 0);
    assert(!dataQ.size());

    vectorwrapper = &vector_wrapper;

    uint64_t SIZE = DST_SIZE;

    uint64_t    vaddr       = mem_addr;
    int32_t     vstride     = stride;

    //reset vecIndex
    vecIndex = 0;

    auto fin = [on_item_load,SIZE,count,this]
        (uint64_t i, std::vector<uint64_t> line_offsets)
    {
        return [on_item_load,SIZE,count,i,line_offsets,this]
            (uint8_t*data, uint8_t size)
        {
            for (uint64_t j=0; j< line_offsets.size(); ++j) {
                bool _done = ( (i+j+1) == count );
                uint8_t *ndata = new uint8_t[SIZE];
                memcpy(ndata, data + line_offsets[j], SIZE);
                DPRINTF(MemUnitReadTiming,
                    "calling on_item_load with size %d. 'done'=%d\n"
                    ,SIZE, _done);
                on_item_load(ndata, SIZE, _done);
            }
        };
    };

    readFunction = [location,vaddr,vstride,SIZE,mop,
        count,on_item_load,xc,fin,this]()
    {
        std::vector<uint64_t> line_offsets;

        //scratch and cache could use different line sizes
        uint64_t line_size;
        switch ((int)location) {
            case 0: line_size = cacheLineSize; break;
            case 1: line_size = VRF_LineSize; break;
            default: panic("invalid location"); break;
        }

        uint64_t line_addr;
        uint8_t items_in_line;
        uint64_t addr;
        uint64_t i = this->vecIndex;

        if (mop != 3) //no indexed operation
        {
            //we can always read the first item
            addr = vaddr + SIZE*(i*vstride);
            line_addr = addr - (addr % line_size);
            line_offsets.push_back(addr % line_size);
            items_in_line = 1;

            //try to read more items in the same cache-line
            for (uint8_t j=1; j<(line_size/SIZE) && (i+j)<count; ++j) {
                uint64_t next_addr = vaddr + SIZE*((i+j)*vstride );
                uint64_t next_line_addr = next_addr - (next_addr % line_size);

                if (next_line_addr == line_addr) {
                    items_in_line += 1;
                    line_offsets.push_back(next_addr % line_size);
                } else {
                    break;
                }
            }
        } else { //indexed operation

            uint64_t can_get = this->dataQ.size();
            if (!can_get) {
                DPRINTF(MemUnitReadTiming, "try_read dataQ Addrs empty\n");
                return false;
            }
            uint64_t got = std::min(line_size/SIZE, can_get);
            uint8_t *buf = new uint8_t[got*SIZE];
            for (uint8_t i=0; i<got; ++i) {
                memcpy(buf+SIZE*i, this->dataQ[i], SIZE);
            }

            uint64_t index_addr;
            if (SIZE == 8) {
                index_addr = (uint64_t)((uint64_t*)buf)[0];
            } else if (SIZE == 4) {
                index_addr = (uint64_t)((uint32_t*)buf)[0];
            } else if (SIZE == 2) {
                index_addr = (uint16_t)((uint16_t*)buf)[0];
            } else if (SIZE == 1) {
                index_addr = (uint8_t)((uint8_t*)buf)[0];
            } else{
                panic("invalid mem req SIZE");
            }
            addr = vaddr + index_addr;
            line_addr = addr - (addr % line_size);
            line_offsets.push_back(addr % line_size);
            items_in_line = 1;

            DPRINTF(MemUnitReadTiming, "Trying to get mora than 1 element"
                " from a line\n");
            DPRINTF(MemUnitReadTiming, "reading addr  %#x ,line_addr %#x "
                "with %d \n",addr, line_addr, items_in_line);

            //try to read more items in the same cache-line
            for (uint8_t j=1; j<got; j++) {
                if (SIZE == 8) {
                    index_addr = (uint64_t)((uint64_t*)buf)[j];
                } else if (SIZE == 4) {
                    index_addr = (uint64_t)((uint32_t*)buf)[j];
                } else if (SIZE == 2) {
                    index_addr = (uint16_t)((uint16_t*)buf)[j];
                } else if (SIZE == 1) {
                    index_addr = (uint8_t)((uint8_t*)buf)[j];
                } else {
                    panic("invalid mem req SIZE"); break;
                }

                uint64_t next_addr = vaddr + index_addr;
                uint64_t next_line_addr = next_addr - (next_addr % line_size);

                DPRINTF(MemUnitReadTiming, "next_addr  %#x  \n",next_addr);
                DPRINTF(MemUnitReadTiming, "next_line_addr  %#x ,line_addr "
                    "%#x  \n",next_line_addr, line_addr);

                if (next_line_addr == line_addr) {
                    items_in_line += 1;
                    line_offsets.push_back(next_addr % line_size);
                } else {
                    break;
                }
                DPRINTF(MemUnitReadTiming,"items_in_line %d\n",items_in_line);
            }
        delete buf;
        } //end indexed operation

        //try to read the line
        DPRINTF(MemUnitReadTiming, "reading line_addr %#x with %d items for "
            "addr %#x\n",line_addr, items_in_line, addr);

        bool success;
        if (location == 0) { // location = 0 = MEMORY
            Cache_line_r_req++;
            success = vectorwrapper->readVectorMem(line_addr, line_size,
                 xc->tcBase(),this->channel, fin(i, line_offsets));
        } else {
            success = vectorwrapper->readVectorReg(line_addr, line_size,
                this->channel, fin(i, line_offsets));
        }
        if (success) {
            if (mop == 3) {
                for (uint8_t j=0; j<items_in_line; j++) {
                    this->dataQ.pop_front();
                    }
            }
            this->vecIndex += items_in_line;
            this->done = (this->vecIndex == count);
            return true;
        }
        return false;
    };

    if (readFunction() && done) {
        done = false;
    } else {
        start();
    }
}

MemUnitReadTiming *
MemUnitReadTimingParams::create()
{
    return new MemUnitReadTiming(this);
}
