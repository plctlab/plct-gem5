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

#include "cpu/vector_engine/vpu/rob/reorder_buffer.hh"

#include <bitset>
#include <cstdint>
#include <deque>
#include <functional>

#include "debug/ReorderBuffer.hh"
#include "params/ReorderBuffer.hh"
#include "sim/faults.hh"
#include "sim/ticked_object.hh"

ReorderBuffer::ReorderBuffer(ReorderBufferParams *p):
TickedObject(p),occupied(false), ROB_Size(p->ROB_Size)
{
    for (int i=0 ; i<ROB_Size ; i++) {
        rob.push_back(new rob_entry(0,0));
    }
    tail=0;
    head=0;
    valid_elements=0;
}

ReorderBuffer::~ReorderBuffer()
{
}


void
ReorderBuffer::startTicking(VectorEngine& vector_wrapper)
{
    DPRINTF(ReorderBuffer,"ReorderBuffer StartTicking \n");
    this->vectorwrapper = &vector_wrapper;
    start();
}

void
ReorderBuffer::stopTicking()
{
    DPRINTF(ReorderBuffer,"ReorderBuffer StopTicking \n");
    stop();
}

bool
ReorderBuffer::isOccupied()
{
    return occupied;
}

void
ReorderBuffer::regStats()
{
    TickedObject::regStats();

    VectorROBentriesUsed
        .name(name() + ".VectorROBentriesUsed")
        .desc("Max number of ROB entries used during execution");
}

void
ReorderBuffer::evaluate()
{
    assert(running);
    assert((valid_elements >= 0) && (valid_elements <= ROB_Size));
    if ( valid_elements==0) {
        stopTicking(); return;
    }
    /* For statistics*/
    if ((double)valid_elements > VectorROBentriesUsed.value()) {
        VectorROBentriesUsed = valid_elements;
    }

    if (rob[head]->executed)
    {
        DPRINTF(ReorderBuffer,"Commiting ROB entry %d \n",head);
        if (rob[head]->valid_old_dst)
        {
        DPRINTF(ReorderBuffer,"Freeing up old_dst %d \n",rob[head]->old_dst);
        vectorwrapper->vector_rename->set_frl(rob[head]->old_dst);
        }
        if (head == ROB_Size-1) {
            head=0;
        } else {
            head++;
        }
        valid_elements--;
    }
}

bool
ReorderBuffer::rob_full()
{
    return (valid_elements==ROB_Size);
}

bool
ReorderBuffer::rob_empty()
{
    return (valid_elements==0);
}

uint32_t
ReorderBuffer::set_rob_entry(uint32_t old_dst, bool valid_old_dst)
{
    assert(valid_elements < ROB_Size);

    rob[tail]->old_dst = old_dst;
    rob[tail]->valid_old_dst = valid_old_dst;
    rob[tail]->executed = 0;
    uint32_t return_tail = tail;
    if (valid_old_dst) {
        DPRINTF(ReorderBuffer,"Setting the ROB entry %d  with an old dst %d \n"
            ,tail,old_dst);
    } else {
        DPRINTF(ReorderBuffer,"Setting the ROB entry %d without old dst %d \n"
            ,tail);
    }

    if (tail == ROB_Size-1) {
        tail=0;
    } else {
        tail++;
    }

    valid_elements ++;

    return return_tail;
}

void
ReorderBuffer::set_rob_entry_executed(uint32_t idx)
{
    assert(idx < ROB_Size);
    DPRINTF(ReorderBuffer,"Setting the ROB entry %d as executed \n",idx);
    rob[idx]->executed = 1;
}

ReorderBuffer *
ReorderBufferParams::create()
{
    return new ReorderBuffer(this);
}