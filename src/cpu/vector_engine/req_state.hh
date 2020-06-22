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
 */

#ifndef __CPU_VECTOR_REQ_STATE_HH__
#define __CPU_VECTOR_REQ_STATE_HH__

#include <cassert>
#include <cstdint>
#include <functional>

#include "mem/packet.hh"
#include "mem/request.hh"

// basic state for a request to either the vector_reg or vector_mem. uniquely
// identifies any read/write with a unique reqId. A callback must be executed
// on the completion of the request
class Vector_ReqState
{
  public:
    Vector_ReqState(uint64_t req_id) :
        reqId(req_id), matched(false), pkt(nullptr)
    {}
    virtual ~Vector_ReqState() {
        if (pkt != nullptr) {
            assert(pkt->req != nullptr);
            //delete pkt->req;
            delete pkt;
        }
    }

    //read/write have different callback type-definitions. so abstract it here
    virtual void executeCallback() = 0;

    uint64_t getReqId() { return reqId; }
    bool isMatched() { return matched; }
    void setPacket(PacketPtr _pkt) {
        assert(!matched);
        matched = true;
        pkt = _pkt;
    }

  protected:
    uint64_t reqId;
    bool matched;
    PacketPtr pkt;
};


class Vector_R_ReqState : public Vector_ReqState
{
  public:
    Vector_R_ReqState(uint64_t req_id,
        std::function<void(uint8_t*data, uint8_t size)> callback) :
        Vector_ReqState(req_id), callback(callback) {}
    ~Vector_R_ReqState() {}

    void executeCallback() override {
        assert(matched);
        callback(pkt->getPtr<uint8_t>(), pkt->getSize());
    }

  private:
    std::function<void(uint8_t*data, uint8_t size)> callback;
};

class Vector_W_ReqState : public Vector_ReqState
{
  public:
    Vector_W_ReqState(uint64_t req_id, std::function<void(void)> callback) :
        Vector_ReqState(req_id), callback(callback) {}
    ~Vector_W_ReqState() {}

    void executeCallback() override {
        assert(matched);
        callback();
    }

  private:
    std::function<void(void)> callback;
};

#endif // __CPU_VECTOR_REQ_STATE_HH__
