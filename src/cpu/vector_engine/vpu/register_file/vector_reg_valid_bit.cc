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

#include "cpu/vector_engine/vpu/register_file/vector_reg_valid_bit.hh"

#include "debug/VectorValidBit.hh"

/**
 * Valid bits
 */
VectorValidBit::VectorValidBit(VectorValidBitParams *p):
SimObject(p), PhysicalRegs(p->PhysicalRegs)
{
    for (uint64_t i=0; i<32; i++)
        {
            reg_valid_bit.push_back(1);
        }
    for (uint64_t i=32; i<PhysicalRegs; i++)
        {
            reg_valid_bit.push_back(0);
        }
}

VectorValidBit::~VectorValidBit()
{
}

void
VectorValidBit::set_preg_valid_bit(int idx , int val)
{
    assert(idx <= PhysicalRegs);
    //DPRINTF(VectorValidBit,"Setting valid bit %d: %d\n",idx,val);
    //if (Validbit_queue.size()==0) {
    //    startTicking();
    //}
    //Validbit_queue.push_back(new validbit_queue(idx,val));

    reg_valid_bit[idx] = val;
    DPRINTF(VectorValidBit, "Setting the validbit reg %d with %d\n"
            ,idx,val);
}

int
VectorValidBit::get_preg_valid_bit(int idx)
{
    assert((idx <= PhysicalRegs));
    return reg_valid_bit[idx];
}

//bool
//VectorValidBit::isOccupied()
//{
//    return occupied;
//}

//void
//VectorValidBit::startTicking()
//{
//    //DPRINTF(VectorValidBit,"VectorValidBit StartTicking \n");
//    start();
//}


//void
//VectorValidBit::stopTicking()
//{
//    //DPRINTF(VectorValidBit,"VectorValidBit StopTicking \n");
//    stop();
//}
/*
void
VectorValidBit::regStats()
{
  TickedObject::regStats();
  ClockedObject::regStats();
}
*/

//void
//VectorValidBit::evaluate()
//{
//    assert(running);
//    if ( Validbit_queue.size()==0) {
//        stopTicking();
//        return;
//    }
//
//    validbit_queue * set_new_vb = Validbit_queue.front();
//
//    if (set_new_vb->cyclesLeft == 0) {
//        reg_valid_bit[set_new_vb->idx] = set_new_vb->val;
//        DPRINTF(VectorValidBit, "Setting the validbit reg %d with %d\n"
//            ,set_new_vb->idx,set_new_vb->val);
//        Validbit_queue.pop_front();
//        delete set_new_vb;
//    } else {
//        set_new_vb->cyclesLeft --;
//    }
//}

void
VectorValidBit::print_valid_bit() {
    // Empty
}

VectorValidBit *
VectorValidBitParams::create()
{
    return new VectorValidBit(this);
}