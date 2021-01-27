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
#include "debug/InstQueueInst.hh"
#include "debug/InstQueueRenInst.hh"

InstQueue::InstQueue(InstQueueParams *p) :
TickedObject(p), occupied(false),
OoO_queues(p->OoO_queues),
vector_mem_queue_size(p->vector_mem_queue_size),
vector_arith_queue_size(p->vector_arith_queue_size)
{
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
    this->vectorwrapper = &vector_wrapper;
    start();
}


void
InstQueue::stopTicking()
{
    stop();
}

void
InstQueue::regStats()
{
    TickedObject::regStats();

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

        uint64_t src1=0;
        uint64_t src2=0;
        uint64_t src3=0;
        uint64_t mask=0;

        bool masked_op=0;
        bool vx_op=0;
        bool vf_op=0;
        bool vi_op=0;
        bool mask_ready=0;
        bool src1_ready=0;
        bool src2_ready=0;
        bool src3_ready=0;
        bool srcs_ready=0;
        QueueEntry * Instruction = Instruction_Queue.front();
        uint64_t queue_slot = 0;

        int queue_size = (OoO_queues) ? Instruction_Queue.size() : 1;

        for (int i=0 ; i< queue_size ; i++)
        {
            Instruction = Instruction_Queue[i];

            src1 = Instruction->dyn_insn->get_renamed_src1();
            src2 = Instruction->dyn_insn->get_renamed_src2();
            src3 = Instruction->dyn_insn->get_renamed_old_dst();
            mask = Instruction->dyn_insn->get_renamed_mask();

            masked_op = (Instruction->insn.vm()==0);

            /*
             * Instructions with Scalar operands set the src1_ready signal
             */
            vx_op = (Instruction->insn.func3()==4) || (Instruction->insn.func3()==6);
            vf_op = (Instruction->insn.func3()==5);
            vi_op = (Instruction->insn.func3()==3);

            if (masked_op) {
                mask_ready = vectorwrapper->vector_reg_validbit->
                    get_preg_valid_bit(mask);
            } else {
                mask_ready = 1;
            }

            if ((Instruction->insn.arith2Srcs() ||
                Instruction->insn.arith3Srcs()) && !( vx_op || vf_op || vi_op)) {
                src1_ready = vectorwrapper->vector_reg_validbit->
                    get_preg_valid_bit(src1);
            }
            else {
                src1_ready =1;
            }

            if (Instruction->insn.arith1Src() ||
                Instruction->insn.arith2Srcs() ||
                Instruction->insn.arith3Srcs()){
                src2_ready = vectorwrapper->vector_reg_validbit->
                    get_preg_valid_bit(src2);
            } else {
                src2_ready =1;
            }

            if (Instruction->insn.arith3Srcs()) {
                src3_ready = vectorwrapper->vector_reg_validbit->
                    get_preg_valid_bit(src3);
            } else {
                src3_ready =1;
            }

            srcs_ready = src1_ready && src2_ready && src3_ready &&
                mask_ready && !Instruction->issued;

            if (srcs_ready) {
                queue_slot = i;
                Instruction->issued = 1;
                break;
            }
        }

        if (srcs_ready)
        {
            /*printing the issued instruction*/
            printArithInst(Instruction->insn,Instruction->dyn_insn);
            /*removing the instruction from the queue*/
            Instruction_Queue.erase(Instruction_Queue.begin()+queue_slot);
            /*Issuing the instruction*/
            vectorwrapper->issue(Instruction->insn,Instruction->dyn_insn,
                Instruction->xc,Instruction->src1,Instruction->src2,
                 Instruction->rename_vtype,Instruction->rename_vl,
                [Instruction,this](Fault f) {

                // Setting the Valid Bit
                bool wb_enable = !Instruction->insn.VectorToScalar();
                uint64_t renamed_dst = Instruction->dyn_insn->get_renamed_dst();
                if (wb_enable)
                {
                vectorwrapper->vector_reg_validbit->set_preg_valid_bit(renamed_dst,1);
                }
                //Setting the executed bit in the ROB
                uint16_t rob_entry = Instruction->dyn_insn->get_rob_entry();
                vectorwrapper->vector_rob->set_rob_entry_executed(rob_entry);

                DPRINTF(InstQueue,"Executed instruction %s\n",
                    Instruction->insn.getName());
                DPRINTF(InstQueue,"Arithmetic queue Size %d\n",
                    Instruction_Queue.size());
                
                if (Instruction->insn.VectorToScalar()) {
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
            //DPRINTF(InstQueue,"Sources not ready\n");
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

        bool isStore = 0;
        bool isLoad = 0;
        uint64_t src3=0;
        uint64_t src2=0;
        uint8_t mop=0;
        bool indexed_op=0;
        uint64_t src_ready=0;
        bool ambiguous_dependency = 0;

        QueueEntry * Mem_Instruction = Memory_Queue.front();
        uint64_t queue_slot = 0;
        int queue_size = (OoO_queues) ? Memory_Queue.size() : 1;
        //int min = std::min(queue_size ,32);
        for (int i=0 ; i< queue_size ; i++)
        {

            Mem_Instruction = Memory_Queue[i];
            isLoad = Mem_Instruction->insn.isLoad();
            isStore = Mem_Instruction->insn.isStore();
            src3 = Mem_Instruction->dyn_insn->get_renamed_src3();
            src2 = Mem_Instruction->dyn_insn->get_renamed_src2();
            mop = Mem_Instruction->insn.mop();
            indexed_op = (mop == 3) || (mop == 7);

            // If the instruction is indexed we stop looking for the next
            // instructions, check dependencies for indexed is too expensive
            if ( (indexed_op || isStore) && i>0){
                src_ready = 0;
                break;
            }

            /*
            // Improve it
            if ((i!=0) && isLoad)
            {
            for (int j=i ; j>0 ; j--)
                {
                QueueEntry *  Mem_Instruction_dep = Memory_Queue[j-1];
                ambiguous_dependency = ambiguous_dependency |
                    (Mem_Instruction_dep->src1 == Mem_Instruction->src1);
                delete Mem_Instruction_dep;
                }
            } */

            /* TODO : bug aqui ... debo evaluar bien los casos de indexed strided y unitstride ahora soportados*/
            src_ready = (isStore && !indexed_op) ?
                vectorwrapper->vector_reg_validbit->get_preg_valid_bit(src3) :
                (isStore && indexed_op) ?
                vectorwrapper->vector_reg_validbit->get_preg_valid_bit(src3) &&
                vectorwrapper->vector_reg_validbit->get_preg_valid_bit(src2) :
                (isLoad && !indexed_op) ?
                !Mem_Instruction->issued && !ambiguous_dependency :
                (isLoad && indexed_op) ?
                vectorwrapper->vector_reg_validbit->get_preg_valid_bit(src2):0;

            if (src_ready) {
                queue_slot = i;
                Mem_Instruction->issued = 1;
                break;
            }
        }

        if (src_ready)
        {
            /*printing the issued instruction*/
            printMemInst(Mem_Instruction->insn,Mem_Instruction->dyn_insn);
            /*removing the instruction from the queue*/
            Memory_Queue.erase(Memory_Queue.begin()+queue_slot);
            /*Issuing the instruction*/
            vectorwrapper->issue(Mem_Instruction->insn,
                Mem_Instruction->dyn_insn,Mem_Instruction->xc,
                Mem_Instruction->src1,Mem_Instruction->src2,
                Mem_Instruction->rename_vtype,Mem_Instruction->rename_vl,
                [Mem_Instruction,this](Fault f) {

                bool wb_enable = !Mem_Instruction->insn.isStore();
                uint64_t renamed_dst = Mem_Instruction->dyn_insn->get_renamed_dst();
                // SETTING VALID BIT
                if (wb_enable)
                {
                vectorwrapper->vector_reg_validbit->set_preg_valid_bit(renamed_dst,1);
                }

                //Setting the executed bit in the ROB
                uint16_t rob_entry =
                    Mem_Instruction->dyn_insn->get_rob_entry();
                vectorwrapper->vector_rob->set_rob_entry_executed(rob_entry);

                DPRINTF(InstQueue,"Executed instruction %s\n",
                    Mem_Instruction->insn.getName());
                DPRINTF(InstQueue,"Memory queue Size %d\n",Memory_Queue.size());
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
            //DPRINTF(InstQueue,"Sources not ready\n");
        }
    }
}

void
InstQueue::printMemInst(RiscvISA::VectorStaticInst& insn,VectorDynInst *vector_dyn_insn)
{
    uint64_t pc = insn.getPC();
    bool indexed = (insn.mop() ==3);
    bool masked_op = (insn.vm()==0);

    uint32_t PDst = vector_dyn_insn->get_renamed_dst();
    uint32_t POldDst = vector_dyn_insn->get_renamed_old_dst();
    uint32_t Pvs2 = vector_dyn_insn->get_renamed_src2();
    uint32_t Pvs3 = vector_dyn_insn->get_renamed_src3();
    uint32_t PMask = vector_dyn_insn->get_renamed_mask();

    std::stringstream mask_ren;
    if (masked_op) {
        mask_ren << "v" << PMask << ".m";
    } else {
        mask_ren << "";
    }

    if (insn.isLoad())
    {
        if (indexed){
            DPRINTF(InstQueueInst,"issuing inst: %s v%d v%d       PC 0x%X\n",
                insn.getName(),insn.vd(),insn.vs2(),*(uint64_t*)&pc);
            DPRINTF(InstQueueRenInst,"issuing renamed inst: %s v%d v%d %s  old_dst v%d"
                "      PC 0x%X\n",insn.getName(),PDst,Pvs2,mask_ren.str(),POldDst,
                *(uint64_t*)&pc);
        } else {
            DPRINTF(InstQueueInst,"issuing inst: %s v%d       PC 0x%X\n"
                ,insn.getName(),insn.vd(),*(uint64_t*)&pc);
            DPRINTF(InstQueueRenInst,"issuing renamed inst: %s v%d %s  old_dst v%d     "
                " PC 0x%X\n",insn.getName(),PDst,mask_ren.str(),POldDst,*(uint64_t*)&pc);
        }
    }
    else if (insn.isStore())
    {
         if (indexed){
            DPRINTF(InstQueueInst,"issuing inst: %s v%d v%d       PC 0x%X\n",
                insn.getName(),insn.vd(),insn.vs2(),*(uint64_t*)&pc);
            DPRINTF(InstQueueRenInst,"issuing renamed inst: %s v%d v%d %s     PC 0x%X\n",
                insn.getName(),Pvs3,Pvs2,mask_ren.str(),*(uint64_t*)&pc);
        } else {
            DPRINTF(InstQueueInst,"issuing inst: %s v%d       PC 0x%X\n",
                insn.getName(),insn.vd(),*(uint64_t*)&pc );
            DPRINTF(InstQueueRenInst,"issuing renamed inst: %s v%d %s       PC 0x%X\n",
                insn.getName(),Pvs3,mask_ren.str(),*(uint64_t*)&pc);
        }
        
    } else {
        panic("Invalid Vector Instruction insn=%#h\n", insn.machInst);
    }
}

void
InstQueue::printArithInst(RiscvISA::VectorStaticInst& insn,VectorDynInst *vector_dyn_insn)
{
    uint64_t pc = insn.getPC();
    bool masked_op = (insn.vm()==0);
    bool vx_op = (insn.func3()==4) || (insn.func3()==6);
    bool vf_op = (insn.func3()==5);
    bool vi_op = (insn.func3()==3);

    std::string masked;
    if(masked_op) {masked = "v0.m";} else {masked = "   ";}
    std::string reg_type;
    if(insn.VectorToScalar()==1) {reg_type = "x";} else {reg_type = "v";}

    std::string scr1_type;
    scr1_type = (vx_op) ? "x" :
                (vf_op) ? "f" :
                (vi_op) ? " " : "v";

    uint32_t PDst = (insn.VectorToScalar()==1) ? insn.vd() :
                    vector_dyn_insn->get_renamed_dst();
    uint32_t POldDst = vector_dyn_insn->get_renamed_old_dst();
    uint32_t Pvs1 = (vx_op || vf_op || vi_op) ? insn.vs1() :
                    vector_dyn_insn->get_renamed_src1();
    uint32_t Pvs2 = vector_dyn_insn->get_renamed_src2();
    uint32_t PMask = vector_dyn_insn->get_renamed_mask();

    std::stringstream mask_ren;
    if (masked_op) {
        mask_ren << "v" << PMask << ".m";
    } else {
        mask_ren << "";
    }

    if (insn.arith1Src()) {
        DPRINTF(InstQueueInst,"issuing inst: %s %s%d v%d %s           PC 0x%X\n",
            insn.getName(),reg_type,insn.vd(),insn.vs2(),masked,*(uint64_t*)&pc );
        DPRINTF(InstQueueRenInst,"issuing renamed inst: %s %s%d v%d %s  old_dst v%d     "
            "    PC 0x%X\n",insn.getName(),reg_type,PDst,Pvs2,mask_ren.str(),
            POldDst,*(uint64_t*)&pc);
    }
    else if (insn.arith2Srcs()) {
        DPRINTF(InstQueueInst,"issuing inst: %s %s%d v%d %s%d %s       PC 0x%X\n",
            insn.getName(),reg_type,insn.vd(),insn.vs2(),scr1_type,insn.vs1(),
            masked,*(uint64_t*)&pc );
        DPRINTF(InstQueueRenInst,"issuing renamed inst: %s %s%d v%d %s%d %s  old_dst v%d"
            "        PC 0x%X\n",insn.getName(),reg_type,PDst,Pvs2,scr1_type,Pvs1,
            mask_ren.str(),POldDst,*(uint64_t*)&pc);
    }
    else if (insn.arith3Srcs()) {
        DPRINTF(InstQueueInst,"issuing inst: %s %s%d v%d %s%d %s       PC 0x%X\n",
            insn.getName(),reg_type,insn.vd(),insn.vs2(),scr1_type,insn.vs1(),
            masked,*(uint64_t*)&pc );
        DPRINTF(InstQueueRenInst,"issuing renamed inst: %s %s%d v%d %s%d v%d %s "
            "old_dst v%d         PC 0x%X\n",insn.getName(),reg_type,PDst,Pvs2,scr1_type,
            Pvs1,POldDst,mask_ren.str(),POldDst,*(uint64_t*)&pc);
    } else {
        panic("Invalid Vector Instruction insn=%#h\n", insn.machInst);
    }
}


InstQueue *
InstQueueParams::create()
{
    return new InstQueue(this);
}

