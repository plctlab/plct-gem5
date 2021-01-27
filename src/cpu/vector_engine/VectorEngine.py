# Copyright (c) 2020 Barcelona Supercomputing Center
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Cristobal Ramirez


from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject



class VectorEngine(SimObject):
    type = 'VectorEngine'
    cxx_header = "cpu/vector_engine/vector_engine.hh"

    vector_config = Param.VectorConfig("Vector CSR Copy")

    vector_reg = Param.VectorRegister("Vector Register");
    vector_reg_port = VectorMasterPort("Vector Register Port")
    vector_rf_ports = Param.Unsigned(1,"number of VRF ports")

    vector_mem_port = MasterPort("Vector Accelerator Memory Port")

    vector_lane = VectorParam.VectorLane("Vector Lane")
    num_clusters = Param.Unsigned(1,"Number of independent execution clusters")
    num_lanes = Param.Unsigned(8,"Number of lanes")
    vector_rob = Param.ReorderBuffer("Vector Reorder Buffer")
    vector_memory_unit = Param.VectorMemUnit("Vector Memory Unit ")
    vector_inst_queue = Param.InstQueue("Vector Instruction Queue")
    vector_rename = Param.VectorRename("Vector Renaming unit")
    vector_reg_validbit = Param.VectorValidBit("Vector Validbit unit")

    system = Param.System(Parent.any, "system object")
