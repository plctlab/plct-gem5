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

#include "cpu/vector_engine/vector_engine.hh"

#include <cassert>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "base/types.hh"
#include "cpu/translation.hh"
#include "debug/VectorEngine.hh"
#include "debug/VectorEngineInfo.hh"
#include "debug/VectorInst.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"

VectorEngine::VectorEngine(VectorEngineParams *p) :
SimObject(p),
vector_csr(p->vector_csr),
VectorCacheMasterId(p->system->getMasterId(this, name() + ".vector_cache")),
vectormem_port(name() + ".vector_mem_port", this, p->vector_rf_ports),
vector_reg(p->vector_reg),
uniqueReqId(0),
num_clusters(p->num_clusters),
num_lanes(p->num_lanes),
vector_rob(p->vector_rob),
vector_lane(p->vector_lane),
vector_memory_unit(p->vector_memory_unit),
vector_inst_queue(p->vector_inst_queue),
vector_rename(p->vector_rename),
vector_reg_validbit(p->vector_reg_validbit),
last_vtype(0),
last_vl(0)
{
    //create independent ports
    for (uint8_t i=0; i< p->vector_rf_ports; ++i) {
        VectorRegMasterIds.push_back(p->system->getMasterId(this, name()
            + ".vector_reg" + std::to_string(i)));
        VectorRegPorts.push_back(VectorRegPort(name()
            + ".vector_reg_port", this, i));
    }

    DPRINTF(VectorEngineInfo,"VPU created:\n");
    DPRINTF(VectorEngineInfo,"Vector Renaming Enabled\n");
    DPRINTF(VectorEngineInfo,"Number of Physical Registers: %d \n"
        ,vector_rename->PhysicalRegs );
    DPRINTF(VectorEngineInfo,"Maximum VL: %d-bits\n"
        , vector_csr->get_max_vector_length_bits() );
    DPRINTF(VectorEngineInfo,"Register File size: %dKB\n"
        , (float)vector_reg->get_size()/1024.0 );
    DPRINTF(VectorEngineInfo,"Vector Reorder buffer size: %d\n"
        , vector_rob->ROB_Size );
    DPRINTF(VectorEngineInfo,"Vector Arithmetic Queue Size: %d \n"
        ,vector_inst_queue->vector_arith_queue_size );
    DPRINTF(VectorEngineInfo,"Vector Memory Queue Size: %d \n"
        ,vector_inst_queue->vector_mem_queue_size );
    if (vector_inst_queue->OoO_queues)
        DPRINTF(VectorEngineInfo,"Out-of-Order Issue Logic\n");
    else
        DPRINTF(VectorEngineInfo,"In-Order Issue Logic\n");
    if (num_lanes<num_clusters) {
        panic("Invalid VPU Configuration\n");
    }
    DPRINTF(VectorEngineInfo,"Number of Clusters: %d\n",num_clusters);
    DPRINTF(VectorEngineInfo,"Lanes per Cluster: %d\n",num_lanes/num_clusters);
    DPRINTF(VectorEngineInfo,"Vector Memory Unit Port connected to l2 bus\n");
}

VectorEngine::~VectorEngine()
{
}


void
VectorEngine::regStats()
{
    SimObject::regStats();

    VectorArithmeticIns
        .name(name() + ".VectorArithmeticIns")
        .desc("Number vector arithmetic instructions");
    VectorMemIns
        .name(name() + ".VectorMemIns")
        .desc("Number vector arithmetic instructions");
    VectorConfigIns
        .name(name() + ".VectorConfigIns")
        .desc("Number vector configuration instructions");
    SumVL
        .name(name() + ".SumVL")
        .desc("Sum of all instruction's vector length to obtain the average ");
    AverageVL
        .name(name() + ".AverageVL")
        .desc("SumVL divided by the number of instructions")
        .precision(1);
    AverageVL = SumVL/(VectorMemIns+VectorArithmeticIns);

}


bool
VectorEngine::requestGrant(RiscvISA::VectorStaticInst *insn)
{
    bool rob_entry_available = !vector_rob->rob_full();
    bool queue_slots_available = ((insn->isVectorInstArith()
        || insn->isSetVL()) && !vector_inst_queue->arith_queue_full())
        || (insn->isVectorInstMem()
        && !vector_inst_queue->mem_queue_full());

    return  !vector_rename->frl_empty() && queue_slots_available
        && rob_entry_available;
}

bool
VectorEngine::isOccupied()
{
    bool cluster_ocuppied = 0;
    for (int i=0 ; i< num_clusters ; i++)
    {
        cluster_ocuppied = cluster_ocuppied || vector_lane[i]->isOccupied();
    }

    bool data_in_queues = 0;
    data_in_queues = (vector_inst_queue->Instruction_Queue.size() > 0)
        || (vector_inst_queue->Memory_Queue.size() > 0);

    bool rob_empty = vector_rob->rob_empty();

    return (cluster_ocuppied || data_in_queues
        || vector_memory_unit->isOccupied()
        || vector_inst_queue->isOccupied() || !rob_empty);
}

bool
VectorEngine::cluster_available()
{
    bool available = 0;

    for (int i=0 ; i< num_clusters ; i++)
    {
        available = available || !vector_lane[i]->isOccupied();
    }

    return available;
}

void
VectorEngine::printConfigInst(RiscvISA::VectorStaticInst& insn, uint64_t src1,uint64_t src2)
{
    uint64_t pc = insn.getPC();
    DPRINTF(VectorInst,"%s vl:%d, vtype:%d       PC 0x%X\n",insn.getName(),src1,src2,*(uint64_t*)&pc );
}

void
VectorEngine::printMemInst(RiscvISA::VectorStaticInst& insn)
{
    uint64_t pc = insn.getPC();
    bool gather_op = (insn.mop() ==3);

    if (insn.isLoad())
    {
        if (gather_op){
            DPRINTF(VectorInst,"%s v%d v%d       PC 0x%X\n",insn.getName(),insn.vd(),insn.vs2(),*(uint64_t*)&pc );
        } else {
            DPRINTF(VectorInst,"%s v%d       PC 0x%X\n",insn.getName(),insn.vd(),*(uint64_t*)&pc );
        }
    }
    else if (insn.isStore())
    {
        DPRINTF(VectorInst,"%s v%d       PC 0x%X\n",insn.getName(),insn.vd(),*(uint64_t*)&pc );
    } else {
        panic("Invalid Vector Instruction insn=%#h\n", insn.machInst);
    }
}

void
VectorEngine::printArithInst(RiscvISA::VectorStaticInst& insn,uint64_t src1)
{
    uint64_t pc = insn.getPC();
    std::string masked;
    if(insn.vm()==0) {masked = "v0.m";} else {masked = "   ";}
    std::string reg_type;
    if(insn.write_to_scalar_reg()==1) {reg_type = "x";} else {reg_type = "v";}

    if (insn.arith_src1()) {
        DPRINTF(VectorInst,"%s %s%d v%d %s           PC 0x%X\n",insn.getName(),reg_type,insn.vd(),insn.vs1(),masked,*(uint64_t*)&pc );
    }
    else if (insn.arith_src2()) {
        DPRINTF(VectorInst,"%s %s%d v%d %s           PC 0x%X\n",insn.getName(),reg_type,insn.vd(),insn.vs2(),masked,*(uint64_t*)&pc );
    }
    else if (insn.arith_src1_src2()) {
        DPRINTF(VectorInst,"%s %s%d v%d v%d %s       PC 0x%X\n",insn.getName(),reg_type,insn.vd(),insn.vs1(),insn.vs2(),masked,*(uint64_t*)&pc );
    }
    else if (insn.arith_src1_src2_src3()) {
        DPRINTF(VectorInst,"%s %s%d v%d v%d %s       PC 0x%X\n",insn.getName(),reg_type,insn.vd(),insn.vs1(),insn.vs2(),masked,*(uint64_t*)&pc );
    }
    else if (insn.arith_no_src()) {
         DPRINTF(VectorInst,"%s v%d val:0x%X %s      PC 0x%X\n",insn.getName(),insn.vd(),src1,masked,*(uint64_t*)&pc );
    } else {
        panic("Invalid Vector Instruction insn=%#h\n", insn.machInst);
    }
}

void
VectorEngine::dispatch(RiscvISA::VectorStaticInst& insn, ExecContextPtr& xc,
    uint64_t src1,uint64_t src2,std::function<void()> dependencie_callback)
{
    //Be sure that the instruction was added to some group in base.isa
    if (insn.isVectorInstArith()) {
        assert( insn.arith_no_src() | insn.arith_src1() | insn.arith_src2()
            | insn.arith_src1_src2() | insn.arith_src1_src2_src3() );
    }

    if ((vector_inst_queue->Instruction_Queue.size()==0)
        && (vector_inst_queue->Memory_Queue.size()==0)) {
        vector_inst_queue->startTicking(*this/*,dependencie_callback*/);
    }

    uint64_t PDst,POldDst;
    uint64_t Pvs1,Pvs2;
    uint64_t vd;
    uint64_t vs1,vs2;
    uint64_t PMask;
    vd = insn.vd();
    vs1 = insn.vs1();
    vs2 = insn.vs2();
    //vs3 = insn.vs3();
    bool masked_op;
    masked_op = (insn.vm()==0);

    uint8_t mop = insn.mop();
    bool gather_op = (mop ==3);

    bool has_dst = !insn.write_to_scalar_reg();

    VectorDynInst *vector_dyn_insn = new VectorDynInst(); //inicializar ...


    if (insn.isSetVL()) {
        rename_vtype = src2;
        rename_vl = src1;
    }
    else if (insn.isVectorInstMem()) {
        if (insn.isLoad())
        {
            if (gather_op)
            {
                Pvs2 = vector_rename->get_preg_rat(vs2);
                vector_dyn_insn->set_PSrc2(Pvs2);
            }
            // //Physical  Mask
            PMask = masked_op ? vector_rename->get_preg_rat(0) :1024;
            vector_dyn_insn->set_PMask(PMask);
            //Physical Destination
            PDst = vector_rename->get_frl();
            vector_dyn_insn->set_PDst(PDst);
            //Physical Old Destination
            POldDst = vector_rename->get_preg_rat(vd);
            vector_dyn_insn->set_POldDst(POldDst);
            // Setting the New Destination in the RAT structure
            vector_rename->set_preg_rat(vd,PDst);

            if (masked_op) {
                DPRINTF(VectorRename,"%s v%lu , Odst v%lu , mask v%lu\n"
                    ,insn.getName(), PDst, POldDst, PMask);
            } else {
                DPRINTF(VectorRename,"%s v%lu , Odst v%lu\n"
                    ,insn.getName(), PDst, POldDst);
            }

            DPRINTF(VectorValidBit,"Set Valid-bit %d : %d\n",PDst,0);
            vector_reg_validbit->set_preg_valid_bit(PDst,0);
        }
        else if (insn.isStore())
        {
            //Physical source
            PDst = vector_rename->get_preg_rat(vd);
            vector_dyn_insn->set_PDst(PDst);
            // MASKED NOT IMPLEMENTED FOR STORES
            if (masked_op) {
                DPRINTF(VectorRename,"%s v%lu ,  , mask %lu\n"
                    ,insn.getName(), PDst, PMask);
            } else {
                DPRINTF(VectorRename,"%s v%lu \n",insn.getName(), PDst);
            }
        }
    }
    else if (insn.isVectorInstArith()) {
        if (insn.arith_src1()) {
            //Physical  Mask
            PMask = masked_op ? vector_rename->get_preg_rat(0) :1024;
            vector_dyn_insn->set_PMask(PMask);
            //Physical Destination
            PDst = vector_rename->get_frl();
            vector_dyn_insn->set_PDst(PDst);
            //Physical Old Destination
            POldDst = vector_rename->get_preg_rat(vd);
            vector_dyn_insn->set_POldDst(POldDst);
            //Physical Src1
            Pvs1 = vector_rename->get_preg_rat(vs1);
            vector_dyn_insn->set_PSrc1(Pvs1);
            // Set the New Destination in the RAT structure
            vector_rename->set_preg_rat(vd,PDst);

            if (masked_op) {
                DPRINTF(VectorRename,"%s v%lu v%lu, Odst v%lu, mask v%lu\n"
                    ,insn.getName(), PDst,Pvs1, POldDst, PMask);
            } else {
                DPRINTF(VectorRename,"%s v%lu v%lu, Odst v%lu\n"
                    ,insn.getName(), PDst,Pvs1, POldDst);}

            DPRINTF(VectorValidBit,"Set Valid-bit %d :%d\n",PDst,0);
            vector_reg_validbit->set_preg_valid_bit(PDst,0);
        }
        else if (insn.arith_src2()) {
            //Physical  Mask
            PMask = masked_op ? vector_rename->get_preg_rat(0) :1024;
            vector_dyn_insn->set_PMask(PMask);
            //Physical Destination
            PDst = (has_dst) ? vector_rename->get_frl() :1024;
            vector_dyn_insn->set_PDst(PDst);
            //Physical Old Destination
            POldDst = (has_dst) ? vector_rename->get_preg_rat(vd) : 2014;
            vector_dyn_insn->set_POldDst(POldDst);
            //Physical Src2
            Pvs2 = vector_rename->get_preg_rat(vs2);
            vector_dyn_insn->set_PSrc2(Pvs2);
            // Set the New Destination in the RAT structure
            if (has_dst) { vector_rename->set_preg_rat(vd,PDst); }

            if (has_dst) {
                if (masked_op) {
                    DPRINTF(VectorRename,"%s v%lu v%lu, Odst v%lu, mask "
                        "%lu\n",insn.getName(), PDst,Pvs2, POldDst, PMask);
                } else {
                    DPRINTF(VectorRename,"%s v%lu v%lu, Odst v%lu\n"
                        ,insn.getName(), PDst,Pvs2, POldDst);
                }
            } else {
                if (masked_op) {
                    DPRINTF(VectorRename,"%s v%lu, mask %lu\n"
                        ,insn.getName(),Pvs2, PMask);
                } else {
                    DPRINTF(VectorRename,"%s v%lu\n",insn.getName(),Pvs2);
                }
            }

            if (has_dst) {
                DPRINTF(VectorValidBit,"Set Valid-bit %d: %d\n",PDst,0);
                vector_reg_validbit->set_preg_valid_bit(PDst,0);
            }
        }
        else if (insn.arith_src1_src2() | insn.arith_src1_src2_src3()) {
            //Physical  Mask
            PMask = masked_op ? vector_rename->get_preg_rat(0) :1024;
            vector_dyn_insn->set_PMask(PMask);
            //Physical Destination
            PDst = vector_rename->get_frl();
            vector_dyn_insn->set_PDst(PDst);
            //Physical Old Destination
            POldDst = vector_rename->get_preg_rat(vd);
            vector_dyn_insn->set_POldDst(POldDst);
            //Physical Src1
            Pvs1 = vector_rename->get_preg_rat(vs1);
            vector_dyn_insn->set_PSrc1(Pvs1);
            //Physical Src2
            Pvs2 = vector_rename->get_preg_rat(vs2);
            vector_dyn_insn->set_PSrc2(Pvs2);
            // Set the New Destination in the RAT structure
            vector_rename->set_preg_rat(vd,PDst);

            if (insn.arith_src1_src2()) {
                if (masked_op) {
                    DPRINTF(VectorRename,"%s v%lu v%lu v%lu, Odst v%lu ,"
                        " mask %lu\n",insn.getName(),
                        PDst,Pvs1,Pvs2, POldDst, PMask);
                } else {
                    DPRINTF(VectorRename,"%s v%lu v%lu v%lu, Odst v%lu\n"
                        ,insn.getName(), PDst,Pvs1,Pvs2, POldDst);}
            } else {
                if (masked_op) {
                    DPRINTF(VectorRename,"%s v%lu v%lu v%lu v%lu, Odst v%lu"
                        ", mask %lu\n",insn.getName(),
                        PDst,Pvs1,Pvs2,POldDst, POldDst, PMask);
                } else {
                    DPRINTF(VectorRename,"%s v%lu v%lu v%lu v%lu, Odst v%lu"
                        "\n",insn.getName(), PDst,Pvs1,Pvs2,POldDst, POldDst);}
            }

            DPRINTF(VectorValidBit,"Set Valid-bit %d: %d\n",PDst,0);
            vector_reg_validbit->set_preg_valid_bit(PDst,0);
        }
        else if (insn.arith_no_src()) {
            //Physical  Mask
            PMask = masked_op ? vector_rename->get_preg_rat(0) :1024;
            vector_dyn_insn->set_PMask(PMask);
            //Physical Destination
            PDst = vector_rename->get_frl();
            vector_dyn_insn->set_PDst(PDst);
            //Physical Old Destination
            POldDst = vector_rename->get_preg_rat(vd);
            vector_dyn_insn->set_POldDst(POldDst);
            // Set the New Destination in the RAT structure
            vector_rename->set_preg_rat(vd,PDst);

            if (masked_op) {
                DPRINTF(VectorRename,"%s v%lu , Odst v%lu , mask v%lu\n"
                    ,insn.getName(), PDst, POldDst, PMask);
            } else {
                DPRINTF(VectorRename,"%s v%lu , Odst v%lu\n"
                    ,insn.getName(), PDst, POldDst);
            }

            DPRINTF(VectorValidBit,"Set Valid-bit %d: %d\n",PDst,0);
            vector_reg_validbit->set_preg_valid_bit(PDst,0);
        } else {
            panic("Invalid Vector Instruction insn=%#h\n", insn.machInst);
        }
    }

    if (vector_rob->rob_empty()) {
        vector_rob->startTicking(*this);
    }

    if (insn.isSetVL()) {
        dependencie_callback();
        uint32_t rob_entry = vector_rob->set_rob_entry(0 , 0);
        vector_dyn_insn->set_rob_entry(rob_entry);
        vector_inst_queue->Instruction_Queue.push_back(
            new InstQueue::QueueEntry(insn,vector_dyn_insn,xc,
            NULL,src1,src2,0,0));
        printConfigInst(insn,src1,src2);
    }
    else if (insn.isVectorInstMem()) {
        dependencie_callback();
        uint32_t rob_entry = vector_rob->set_rob_entry(
            vector_dyn_insn->get_POldDst(), insn.isLoad());
        vector_dyn_insn->set_rob_entry(rob_entry);
        vector_inst_queue->Memory_Queue.push_back(
            new InstQueue::QueueEntry(insn,vector_dyn_insn,xc,
                NULL,src1,src2,rename_vtype,rename_vl));
        printMemInst(insn);
    }
    else if (insn.isVectorInstArith()) {
        if (has_dst) {
            dependencie_callback();
            uint32_t rob_entry = vector_rob->set_rob_entry(
                vector_dyn_insn->get_POldDst() , 1);
            vector_dyn_insn->set_rob_entry(rob_entry);
            vector_inst_queue->Instruction_Queue.push_back(
                new InstQueue::QueueEntry(insn,vector_dyn_insn,xc,
                NULL,src1,src2,rename_vtype,rename_vl));
        } else {
            uint32_t rob_entry = vector_rob->set_rob_entry(0 , 0);
            vector_dyn_insn->set_rob_entry(rob_entry);
            vector_inst_queue->Instruction_Queue.push_back(
                new InstQueue::QueueEntry(insn,vector_dyn_insn,xc,
                dependencie_callback,src1,src2,rename_vtype,rename_vl));
        }
        printArithInst(insn,src1);
    } else {
        panic("Invalid Vector Instruction, insn=%X\n", insn.machInst);
    }
}

void
VectorEngine::issue(RiscvISA::VectorStaticInst& insn,VectorDynInst *dyn_insn,
    ExecContextPtr& xc ,uint64_t src1 ,uint64_t src2,uint64_t ren_vtype,
    uint64_t ren_vl, std::function<void(Fault fault)> done_callback) {

    uint64_t pc = insn.getPC();
    if (insn.isSetVL())
    {
        VectorConfigIns++;
        vector_csr->set_vector_length(src1);
        vector_csr->set_vtype(src2);
        done_callback(NoFault);
    }
    else if (insn.isVectorInstMem())
    {
        VectorMemIns++;
        DPRINTF(VectorEngine,"Sending instruction %s to VMU, pc 0x%lx\n"
            ,insn.getName() , *(uint64_t*)&pc );
        vector_memory_unit->issue(*this,insn,dyn_insn, xc,src1,ren_vtype,
            ren_vl, done_callback);

        SumVL = SumVL.value() + vector_csr->vector_length_in_bits(ren_vl,ren_vtype);
    }
    else if (insn.isVectorInstArith()) {

        SumVL = SumVL.value() + vector_csr->vector_length_in_bits(ren_vl,ren_vtype);
        VectorArithmeticIns++;
        uint8_t lane_id_available = 0;
        for (int i=0 ; i< num_clusters ; i++) {
            if (!vector_lane[i]->isOccupied()) { lane_id_available = i; }
        }
        DPRINTF(VectorEngine,"Sending instruction %s v%d v%d v%d, old v%d ,to "
            "cluster %d, pc 0x%lx\n",insn.getName() ,dyn_insn->get_PDst(),
            dyn_insn->get_PSrc1(), dyn_insn->get_PSrc2(),
            dyn_insn->get_POldDst(), lane_id_available , *(uint64_t*)&pc);
        vector_lane[lane_id_available]->issue(*this,insn,dyn_insn, xc, src1,
            done_callback);
    } else {
        panic("Invalid Vector Instruction, insn=%#h\n", insn.machInst);
    }
}

Port &
VectorEngine::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "vector_reg_port")
        return VectorRegPorts[idx];
    else if (if_name == "vector_mem_port")
        return getVectorMemPort();
    else
        return getPort(if_name, idx);
}

VectorEngine::VectorMemPort::VectorMemPort(const std::string& name,
    VectorEngine* owner, uint8_t channels) :
    MasterPort(name, owner), owner(owner)
{
    //create the queues for each of the channels to the Vector Cache
    for (uint8_t i=0; i<channels; ++i) {
        laCachePktQs.push_back(std::deque<PacketPtr>());
    }
}

VectorEngine::VectorMemPort::~VectorMemPort()
{
}

VectorEngine::VectorMemPort::Tlb_Translation::Tlb_Translation(
    VectorEngine *owner):
    event(this, true), owner(owner)
{
}

VectorEngine::VectorMemPort::Tlb_Translation::~Tlb_Translation()
{
}

void
VectorEngine::VectorMemPort::Tlb_Translation::finish(const Fault &_fault,
    const RequestPtr &_req, ThreadContext *_tc, BaseTLB::Mode _mode)
{
    fault = _fault;
}

void
VectorEngine::VectorMemPort::Tlb_Translation::finish(const Fault _fault,
    uint64_t latency)
{
    fault = _fault;
}

void
VectorEngine::VectorMemPort::Tlb_Translation::markDelayed()
{
    panic("Tlb_Translation::markDelayed not implemented");
}

std::string
VectorEngine::VectorMemPort::Tlb_Translation::name()
{
    //TODO: IDK what to put here
    return "Tlb_Translation";
}

void
VectorEngine::VectorMemPort::Tlb_Translation::translated()
{
}

// submits a request for translation to the execCacheTlb
// MEMCPY data if present, so caller must deallocate it himself!
bool
VectorEngine::VectorMemPort::startTranslation(Addr addr, uint8_t *data,
    uint64_t size, BaseTLB::Mode mode, ThreadContext *tc, uint64_t req_id,
    uint8_t channel)
{
    Process * p = tc->getProcessPtr();
    Addr page1 = p->pTable->pageAlign(addr);
    Addr page2 = p->pTable->pageAlign(addr+size-1);
    assert(page1 == page2);

    //NOTE: need to make a buffer for reads so cache can write to it!
    uint8_t *ndata = new uint8_t[size];
    if (data != nullptr) {
        assert(mode == BaseTLB::Write);
        memcpy(ndata, data, size);
    } else {
      //put a pattern here for debugging
      memset(ndata, 'Z', size);
    }
    MemCmd cmd = (mode==BaseTLB::Write) ? MemCmd::WriteReq :
        MemCmd::ReadReq;

    //virtual address request constructor (copied the data port request)
    //const int asid = 0;
    const Addr pc = tc->instAddr();
    RequestPtr req = std::make_shared<Request>(addr, size, 0,
        owner->VectorCacheMasterId, pc, tc->contextId());

    BaseCPU *cpu = tc->getCpuPtr();

    req->taskId(cpu->taskId());

    //start translation
    Tlb_Translation *translation = new Tlb_Translation(owner);

    tc->getDTBPtr()->translateTiming(req, tc, translation , mode);

    if (translation->fault == NoFault){
        PacketPtr pkt = new VectorPacket(req, cmd, req_id, channel);
        pkt->dataDynamic(ndata);

        if (!sendTimingReq(pkt)) {
            laCachePktQs[channel].push_back(pkt);
            }

        delete translation;
        return true;
    }else{
        translation->fault->invoke(tc, NULL);
        delete translation;
        return false;
        }
}


bool
VectorEngine::VectorMemPort::sendTimingReadReq(Addr addr, uint64_t size,
    ThreadContext *tc, uint64_t req_id, uint8_t channel)
{
    return startTranslation(addr, nullptr, size, BaseTLB::Read, tc, req_id,
        channel);
}

bool
VectorEngine::VectorMemPort::sendTimingWriteReq(Addr addr,
    uint8_t *data, uint64_t size, ThreadContext *tc, uint64_t req_id,
    uint8_t channel)
{
    return startTranslation(addr, data, size, BaseTLB::Write, tc, req_id,
        channel);
}

bool
VectorEngine::VectorMemPort::recvTimingResp(PacketPtr pkt)
{
    VectorPacketPtr la_pkt = dynamic_cast<VectorPacketPtr>(pkt);
    assert(la_pkt != nullptr);
    owner->recvTimingResp(la_pkt);
    return true;
}

void
VectorEngine::VectorMemPort::recvReqRetry()
{
    //TODO: must be a better way to figure out which channel specified the
    //      retry
    for (auto& laCachePktQ : laCachePktQs) {
        //assert(laCachePktQ.size());
        while (laCachePktQ.size() && sendTimingReq(laCachePktQ.front())) {
            // memory system takes ownership of packet
            laCachePktQ.pop_front();
        }
    }
}

VectorEngine::VectorRegPort::VectorRegPort(const std::string& name,
    VectorEngine* owner, uint64_t channel) :
    MasterPort(name, owner), owner(owner), channel(channel)
{
}

VectorEngine::VectorRegPort::~VectorRegPort()
{
}

bool
VectorEngine::VectorRegPort::sendTimingReadReq(Addr addr, uint64_t size,
    uint64_t req_id)
{
    //physical addressing
    RequestPtr req = std::make_shared<Request>
        (addr,size,0,owner->VectorRegMasterIds[channel]);
    PacketPtr pkt = new VectorPacket(req, MemCmd::ReadReq, req_id);

    //make data for cache to put data into
    uint8_t *ndata = new uint8_t[size];
    memset(ndata, 'Z', size);
    pkt->dataDynamic(ndata);

    if (!sendTimingReq(pkt)) {
        //delete pkt->req;
        delete ndata;
        delete pkt;
        return false;
    }
    return true;
}

// memcpy of data, so the caller can delete it when it pleases
bool
VectorEngine::VectorRegPort::sendTimingWriteReq(Addr addr, uint8_t *data,
    uint64_t size, uint64_t req_id)
{
    //physical addressing
    RequestPtr req = std::make_shared<Request>
        (addr,size,0,owner->VectorRegMasterIds[channel]);
    VectorPacketPtr pkt = new VectorPacket(req, MemCmd::WriteReq, req_id);

    //make copy of data here
    uint8_t *ndata = new uint8_t[size];
    memcpy(ndata, data, size);
    pkt->dataDynamic(ndata);

    if (!sendTimingReq(pkt)){
        //delete pkt->req;
        delete ndata;
        delete pkt;
        return false;
    }
    return true;
}

bool
VectorEngine::VectorRegPort::recvTimingResp(PacketPtr pkt)
{
    VectorPacketPtr vector_pkt = dynamic_cast<VectorPacketPtr>(pkt);
    assert(vector_pkt != nullptr);
    owner->recvTimingResp(vector_pkt);
    return true;
}

void
VectorEngine::VectorRegPort::recvReqRetry()
{
    //do nothing, caller will manually resend request
}

void
VectorEngine::recvTimingResp(VectorPacketPtr vector_pkt)
{
    //find the associated request in the queue and associate with vector_pkt
    assert(vector_PendingReqQ.size());
    bool found = false;
    for (Vector_ReqState * pending : vector_PendingReqQ) {
        if (pending->getReqId() == vector_pkt->reqId){
            pending->setPacket(vector_pkt);
            found = true;
            break;
        }
    }
    assert(found);

    //commit each request in order they were issued
    while (vector_PendingReqQ.size() &&
        vector_PendingReqQ.front()->isMatched())
    {
        Vector_ReqState * pending = vector_PendingReqQ.front();
        vector_PendingReqQ.pop_front();
        pending->executeCallback();
        //NOTE: pending deletes packet. packet calls delete[] on the data
        delete pending;
    }
}


bool
VectorEngine::writeVectorMem(Addr addr, uint8_t *data, uint32_t size,
    ThreadContext *tc, uint8_t channel, std::function<void(void)> callback)
{
    DPRINTF(VectorEngine, "inside writeVectorMem\n");
    uint64_t id = (uniqueReqId++);
    Vector_ReqState *pending = new Vector_W_ReqState(id, callback);
    vector_PendingReqQ.push_back(pending);
    if (!vectormem_port.sendTimingWriteReq(addr, data, size, tc, id, channel)){
        delete vector_PendingReqQ.back();
        vector_PendingReqQ.pop_back();
        return false;
    }
    return true;
}

bool
VectorEngine::writeVectorReg(Addr addr, uint8_t *data,
    uint32_t size, uint8_t channel,
    std::function<void(void)> callback)
{
    DPRINTF(VectorEngine, "writeVectorReg got %d bytes to write at %#x\n"
        ,size, addr);
    uint64_t id = (uniqueReqId++);
    Vector_ReqState *pending = new Vector_W_ReqState(id, callback);
    vector_PendingReqQ.push_back(pending);
    if (!VectorRegPorts[channel].sendTimingWriteReq(addr,data,size,id)) {
        delete vector_PendingReqQ.back();
        vector_PendingReqQ.pop_back();
        return false;
    }
    return true;
}

bool
VectorEngine::readVectorMem(Addr addr, uint32_t size,
    ThreadContext *tc, uint8_t channel,
    std::function<void(uint8_t*,uint8_t)> callback)
{
    uint64_t id = (uniqueReqId++);
    Vector_ReqState *pending = new Vector_R_ReqState(id, callback);
    vector_PendingReqQ.push_back(pending);
    if (!vectormem_port.sendTimingReadReq(addr, size, tc, id, channel)) {
        delete vector_PendingReqQ.back();
        vector_PendingReqQ.pop_back();
        return false;
    }
    return true;
}

bool
VectorEngine::readVectorReg(Addr addr, uint32_t size,
    uint8_t channel,
    std::function<void(uint8_t*,uint8_t)> callback)
{
    uint64_t id = (uniqueReqId++);
    Vector_ReqState *pending = new Vector_R_ReqState(id,callback);
    vector_PendingReqQ.push_back(pending);
    if (!VectorRegPorts[channel].sendTimingReadReq(addr,size,id)) {
        delete vector_PendingReqQ.back();
        vector_PendingReqQ.pop_back();
        return false;
    }
    return true;
}

VectorEngine *
VectorEngineParams::create()
{
    return new VectorEngine(this);
}