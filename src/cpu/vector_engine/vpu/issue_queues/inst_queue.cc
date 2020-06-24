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

#include "cpu/vector_engine/vpu/issue_queues/inst_queue.hh"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
#include <queue>

#include "debug/InstQueue.hh"
#include "debug/VectorValidBit.hh"

InstQueue::InstQueue(InstQueueParams *p) :
TickedObject(p), occupied(false),
OoO_queues(p->OoO_queues),
vector_mem_queue_size(p->vector_mem_queue_size),
vector_arith_queue_size(p->vector_arith_queue_size)
{
    //idle_count_by_dependency = 0;
}

InstQueue::~InstQueue()
{
}

bool
InstQueue::isOccupied()
{
    return occupied;
}

bool
InstQueue::arith_queue_full()
{
    return (Instruction_Queue.size() >= vector_arith_queue_size);
}

bool
InstQueue::mem_queue_full()
{
    return (Memory_Queue.size() >= vector_mem_queue_size);
}

void
InstQueue::startTicking(
    VectorEngine& vector_wrapper/*,
    std::function<void()> dependencie_callback*/)
{
//    assert(!running);
//    DPRINTF(InstQueue,"InstQueue startTicking \n");

    //copy over the configuration inputs
    this->vectorwrapper = &vector_wrapper;
    //this->dependencie_callback = dependencie_callback;

//    this->insn = &insn;
//    this->xc = xc;
//    this->dataCallback = data_callback;

    start();
}


void
InstQueue::stopTicking()
{
    DPRINTF(InstQueue,"InstQueue StopTicking \n");
    stop();
}

void
InstQueue::regStats()
{
    TickedObject::regStats();

    idle_count_by_dependency
        .name(name() + ".idle_count_by_dependency")
        .desc("Class of vector idle time because of memory")
        ;
    VectorMemQueueSlotsUsed
        .name(name() + ".VectorMemQueueSlotsUsed")
        .desc("Number of mem queue entries used during execution");
    VectorArithQueueSlotsUsed
        .name(name() + ".VectorArithQueueSlotsUsed")
        .desc("Number of arith queue entries used during execution");
}

void
InstQueue::evaluate()
{
    if ( (Instruction_Queue.size()==0) && (Memory_Queue.size()==0) )
    {
        stopTicking();
        DPRINTF(InstQueue,"Instruction Queue can not Issue more instructions"
            " because is empty \n");
        return;
    }

    if (Instruction_Queue.size()!=0 && vectorwrapper->cluster_available())
    {
        /* For statistics */
        int inst_queue_size = Instruction_Queue.size();
        if ((double)inst_queue_size > VectorArithQueueSlotsUsed.value()) {
            VectorArithQueueSlotsUsed = inst_queue_size;
        }

        uint64_t pc = 0;
        uint64_t src1=0;
        uint64_t src2=0;
        uint64_t src3=0;
        uint64_t mask=0;

        bool masked_op=0;
        bool mask_ready=0;
        bool src1_ready=0;
        bool src2_ready=0;
        bool src3_ready=0;
        bool srcs_ready=0;
        Inst_Queue * Instruction = Instruction_Queue.front();
        uint64_t queue_slot = 0;

        int queue_size = (OoO_queues) ? Instruction_Queue.size() : 1;

        for (int i=0 ; i< queue_size ; i++)
        {
            Instruction = Instruction_Queue[i];

            if (Instruction->insn.isSetVL() && i>0){
                srcs_ready = 0;
                break;
            }

            bool working = 0;
            for (int j=0 ; j< vectorwrapper->num_clusters ; j++)
            {
                working = working ||
                    vectorwrapper->vector_lane[j]->isOccupied();
            }
            if (Instruction->insn.isSetVL() && working){
                srcs_ready = 0;
                break;
            }

            src1 = Instruction->dyn_insn->get_PSrc1();
            src2 = Instruction->dyn_insn->get_PSrc2();
            src3 = Instruction->dyn_insn->get_POldDst();
            mask = Instruction->dyn_insn->get_PMask();

            masked_op = (Instruction->insn.vm()==0) &&
                !Instruction->insn.isSetVL();

            if (masked_op) {
                mask_ready = vectorwrapper->vector_reg_validbit->
                    get_preg_valid_bit(mask);
            } else {
                mask_ready = 1;
            }

            if (Instruction->insn.arith_src1() ||
                Instruction->insn.arith_src1_src2() ||
                Instruction->insn.arith_src1_src2_src3()) {
                src1_ready = vectorwrapper->vector_reg_validbit->
                    get_preg_valid_bit(src1);
            }
            else {
                src1_ready =1;
            }

            if (Instruction->insn.arith_src2() ||
                Instruction->insn.arith_src1_src2() ||
                Instruction->insn.arith_src1_src2_src3()){
                src2_ready = vectorwrapper->vector_reg_validbit->
                    get_preg_valid_bit(src2);
            } else {
                src2_ready =1;
            }

            if (Instruction->insn.arith_src1_src2_src3()) {
                src3_ready = vectorwrapper->vector_reg_validbit->
                    get_preg_valid_bit(src3);
            } else {
                src3_ready =1;
            }

            srcs_ready = src1_ready && src2_ready && src3_ready &&
                mask_ready && !Instruction->issued;

            if (srcs_ready) {
                queue_slot = i;
                pc = Instruction->insn.getPC();
                DPRINTF(InstQueue,"Instruction %s , pc 0x%lx\n",
                    Instruction->insn.getName() , *(uint64_t*)&pc);
                DPRINTF(InstQueue,"Issuing Queue slot %d\n",queue_slot);
                Instruction->issued = 1;
                break;
            }
        }

        if (srcs_ready)
        {
            Instruction_Queue.erase(Instruction_Queue.begin()+queue_slot);

            uint64_t pc_arith = Instruction->insn.getPC();
            DPRINTF(InstQueue,"Issuing Arith_Instruction %s, rd %d, pc 0x%lx\n"
                ,Instruction->insn.getName(),Instruction->dyn_insn->get_PDst(),
                *(uint64_t*)&pc_arith);

            //Instruction_Queue.pop_front();
            vectorwrapper->issue(Instruction->insn,Instruction->dyn_insn,
                Instruction->xc,Instruction->src1,Instruction->src2,0,0,
                [Instruction,masked_op/*,queue_slot*/,pc,this](Fault f) {

                // Setting the Valid Bit
                bool wb_enable = !Instruction->insn.write_to_scalar_reg()
                    && !Instruction->insn.isSetVL();
                uint64_t Dst = Instruction->dyn_insn->get_PDst();
                if (wb_enable)
                {
                DPRINTF(VectorValidBit,"Set Valid bit to reg: %lu  with "
                    "value:%lu\n",Dst,1);
                vectorwrapper->vector_reg_validbit->set_preg_valid_bit(Dst,1);
                }
                //Setting the executed bit in the ROB
                uint16_t rob_entry = Instruction->dyn_insn->get_rob_entry();
                vectorwrapper->vector_rob->set_rob_entry_executed(rob_entry);

                DPRINTF(InstQueue,"Executed instruction %s , pc 0x%lx\n",
                    Instruction->insn.getName() , *(uint64_t*)&pc);
                DPRINTF(InstQueue,"Mem Queue Size %d\n",
                    Instruction_Queue.size());
                if (Instruction->insn.write_to_scalar_reg()) {
                    Instruction->dependencie_callback();
                }

                //delete Instruction->xc;
                delete Instruction->dyn_insn;
                delete Instruction;
                this->occupied = false;
            });
        }
        else
        {
            idle_count_by_dependency ++;
            DPRINTF(InstQueue,"Sources not ready");
        }
    }
    // Stores are executed in order
    // however it is possible to advance loads OoO
    if ((Memory_Queue.size()!=0)
        && !vectorwrapper->vector_memory_unit->isOccupied()) {

        /* For statistics */
        int mem_queue_size = Memory_Queue.size();
        if ((double)mem_queue_size > VectorMemQueueSlotsUsed.value()) {
            VectorMemQueueSlotsUsed = mem_queue_size;
        }

        //occupied = true;
        uint64_t pc = 0;
        bool isStore = 0;
        bool isLoad = 0;
        uint64_t Dst=0;
        uint64_t src2=0;
        uint8_t mop=0;
        bool gather_op=0;
        uint64_t src_ready=0;
        bool ambiguous_dependency = 0;

        Inst_Queue * Mem_Instruction = Memory_Queue.front();
        uint64_t queue_slot = 0;
        int queue_size = (OoO_queues) ? Memory_Queue.size() : 1;
        //int min = std::min(queue_size ,32);
        for (int i=0 ; i< queue_size ; i++)
        {

            Mem_Instruction = Memory_Queue[i];
            isLoad = Mem_Instruction->insn.isLoad();
            isStore = Mem_Instruction->insn.isStore();
            Dst = Mem_Instruction->dyn_insn->get_PDst();
            src2 = Mem_Instruction->dyn_insn->get_PSrc2();
            mop = Mem_Instruction->insn.mop();
            gather_op = (mop ==3);
            // If the instruction is indexed we stop looking for the next
            // instructions, check dependencies for indexed is too expensive
            if ( (gather_op || isStore) && i>0){
                src_ready = 0;
                break;
            }

            /*
            // Improve it
            if ((i!=0) && isLoad)
            {
            for (int j=i ; j>0 ; j--)
                {
                Inst_Queue *  Mem_Instruction_dep = Memory_Queue[j-1];
                ambiguous_dependency = ambiguous_dependency |
                    (Mem_Instruction_dep->src1 == Mem_Instruction->src1);
                delete Mem_Instruction_dep;
                }
            } */

            src_ready = (isStore) ?
                vectorwrapper->vector_reg_validbit->get_preg_valid_bit(Dst)
                :(gather_op) ?
                vectorwrapper->vector_reg_validbit->get_preg_valid_bit(src2)
                : (isLoad) ?
                !Mem_Instruction->issued && !ambiguous_dependency : 0;

            if (src_ready) {
                queue_slot = i;
                pc = Mem_Instruction->insn.getPC();
                DPRINTF(InstQueue,"Mem_Instruction %s , pc 0x%lx\n",
                    Mem_Instruction->insn.getName() , *(uint64_t*)&pc);
                DPRINTF(InstQueue,"Issuing Queue slot %d\n",queue_slot);
                Mem_Instruction->issued = 1;
                break;
            }
        }

        if (src_ready)
        {
            //Memory_Queue.erase(Memory_Queue.begin()+queue_slot);
            Memory_Queue.erase(Memory_Queue.begin()+queue_slot);
            uint64_t pc_mem = Mem_Instruction->insn.getPC();
            DPRINTF(InstQueue,"Issuing Mem_Instruction %s , rd %d, pc 0x%lx\n",
                Mem_Instruction->insn.getName() ,Dst, *(uint64_t*)&pc_mem);

            vectorwrapper->issue(Mem_Instruction->insn,
                Mem_Instruction->dyn_insn,Mem_Instruction->xc,
                Mem_Instruction->src1,Mem_Instruction->src2,
                Mem_Instruction->rename_vtype,Mem_Instruction->rename_vl,
                [Dst,Mem_Instruction,this](Fault f) {

                bool wb_enable = !Mem_Instruction->insn.isStore();

                // SETTING VALID BIT
                if (wb_enable)
                {
                DPRINTF(VectorValidBit,"Set Valid bit to reg: %lu "
                    "with value:%lu\n",Dst,1);
                vectorwrapper->vector_reg_validbit->set_preg_valid_bit(Dst,1);
                }

                //Setting the executed bit in the ROB
                uint16_t rob_entry =
                    Mem_Instruction->dyn_insn->get_rob_entry();
                vectorwrapper->vector_rob->set_rob_entry_executed(rob_entry);

                DPRINTF(InstQueue,"Executed Mem_Instruction %s, rd %d\n",
                    Mem_Instruction->insn.getName(),Dst);
                DPRINTF(InstQueue,"Mem Queue Size %d\n",Memory_Queue.size());
                //Memory_Queue.pop_front();
                //delete Mem_Instruction->xc;
                delete Mem_Instruction->dyn_insn;
                delete Mem_Instruction;
                this->occupied = false;
                //DPRINTF(VectorEngine,"Commit Ends\n");
                });
        }
        else
        {
            DPRINTF(InstQueue,"Sources not ready");
        }
    }
}

InstQueue *
InstQueueParams::create()
{
    return new InstQueue(this);
}

