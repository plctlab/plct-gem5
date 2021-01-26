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

#ifndef __CPU_VECTOR_VB_H__
#define __CPU_VECTOR_VB_H__

#include <bitset>
#include <cstdint>
#include <deque>
#include <functional>

#include "base/statistics.hh"
#include "debug/VectorValidBit.hh"
#include "params/VectorValidBit.hh"
#include "sim/faults.hh"
//#include "sim/ticked_object.hh"
#include "sim/sim_object.hh"

class VectorValidBit : public SimObject
{
public:
    VectorValidBit(VectorValidBitParams *p);
    ~VectorValidBit();

    //void startTicking();
    //void stopTicking();
    //bool isOccupied();

    //void regStats() override;
    //void evaluate() override;

protected:
    //bool occupied;
    const uint64_t PhysicalRegs;

/*    class validbit_queue {
          public:
            validbit_queue(uint64_t idx, uint64_t val):
            idx(idx),val(val),cyclesLeft(0) {}
            ~validbit_queue() {}

            uint64_t idx;
            uint64_t val;
            uint64_t cyclesLeft;
        };

    std::deque<validbit_queue *> Validbit_queue;
*/
    std::vector<int> reg_valid_bit;
public:
    int get_preg_valid_bit(int idx);
    void set_preg_valid_bit(int idx , int val);
    void print_valid_bit();
};



#endif // __CPU_VECTOR_VB_H__


