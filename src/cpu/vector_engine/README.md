# RISC-V Research Platform

## Overview

RISC-V Research Platform is a tool is composed by two main tools: RISC-V Vector Architecture model implemented on gem5 simulator and a benchmark framework, which enables researchers to test novel ideas on vector architectures.

### RISC-V Vector Architecture model (gem5):

We have extended the gem5 simulator to model a decoupled vector architecture. By having a decoupled
design, it is possible to attach the VPU to different CPU models (In-order or Out-of-Order); at the same time,
it is also possible to take advantage of the customization provided by the parametrized-based vector
architecture model, allowing us to obtain a VPU design capable to achieve a tradeoff between performance,
energy efficiency and area, and to build a design that fits with the researcher requirements. For example, the
CPU can be attached to a VPU designed for HPC, by setting a design for very large vectors (256 64-bit
elements), composed of a renaming unit capable to support 64 physical registers, a vector arithmetic and a
memory queue with sixteen entries each, these features can be setup to work with say, eight lanes. In contrast,
the VPU can also be targeted for the embedded market segment by setting a design for short vectors (8 64-
bit elements), reducing the number of available physical registers, and with only one lane configuration.

The model is based on the Working draft of the proposed RISC-V V vector extension.
Stable release, v0.7.1 : https://github.com/riscv/riscv-v-spec

If you use this software or a modified version of it for your research, please cite the paper: Cristóbal Ramirez, César Hernandez, Oscar Palomar, Osman Unsal, Marco Ramírez, and Adrián Cristal. 2020. A RISC-V Simulator and Benchmark Suite for Designing and Evaluating Vector Architectures. ACM Trans. Archit. Code Optim. 17, 4, Article 38 (October 2020), 29 pages. https://doi.org/10.1145/3422667

### Vectorized Benchmark Suite:

To help evaluate these architectures, we developed a novel Vectorized Benchmark Suite which
covers the different possible scenarios that may occur within different vector architecture designs that can
operate from short MVL to very large MVL, taking into account the different modules that can be evaluated
in a vector architecture such as the lanes, the interconnection between lanes and the memory management.

The RISC-V Vectorized Benchmark Suite is a collection composed of seven data-parallel applications from different domains. The suite focuses on benchmarking vector microarchitectures; nevertheless, it can be used as well for Multimedia SIMD microarchitectures. . Applications are VLA; therefore, applications can be tested using
short, medium and large VLs. Current implementation is targeting RISC-V Architectures; however, it can be easily ported to any Vector/SIMD ISA thanks to a wrapper library which we developed to map vector intrinsics and math functions to the target architecture.

The benchmark suite with all its applications and input sets is available as open source free of charge. Some of the benchmark programs have their own licensing terms which might limit their use in some cases.

You can download the Vectorized Benchmark Suite here: https://github.com/RALC88/riscv-vectorized-benchmark-suite
You can find precompiled binaries to test the Vector Architecture model.

### Building the RISC-V System
```
scons -j9 build/RISCV/gem5.opt
```
### Testing

After download the Vectorized Benchmark Suite you can start testing the model. Here you can see an example about how to configure de vector engine an execute the application.

Setting the gem5.opt location
```
cd gem5
export TOP=$PWD
export GEM5=$TOP/gem5
export PATH=$PATH:$GEM5/build/RISCV
```

Now in the riscv-vectorized-benchmark-suite folder:

Setting example application dir
```
cd riscv-vectorized-benchmark-suite
export APPS=$PWD
```
Serial version  of Blackscholes application
```
echo > output_serial.txt
gem5.opt $GEM5/configs/example/riscv_vector_engine.py --cmd="$APPS/_blackscholes/bin/blackscholes_serial 1 $APPS/_blackscholes/input/in_256.input output_serial.txt"
```

Vectorized version of Blacscholes
```
echo > output_vector.txt
gem5.opt $GEM5/configs/example/riscv_vector_engine.py --cmd="$APPS/_blackscholes/bin/blackscholes_vector 1 $APPS/_blackscholes/input/in_256.input output_vector.txt"
```

Running an specific configuration:
```
echo > output_vector.txt
gem5.opt $GEM5/configs/example/riscv_vector_engine.py --cmd="$APPS/_blackscholes/bin/blackscholes_vector 1 $APPS/_blackscholes/input/in_256.input output_vector.txt" --v_lanes=8  --max_vl=16384 --l2_size=1024kB --VRF_physical_regs=64 --OoO_queues=1  --mem_queue_size=16 --arith_queue_size=16 --rob_size=32
```

It is possible to take advantage of the customization provided by the parametrized-based vector
architecture model, allowing us to obtain a VPU design capable to achieve a tradeoff between performance,
energy efficiency and area, and to build a design that fits with the researcher requirements. For example, the
CPU can be attached to a VPU designed for HPC, by setting a design for very large vectors (256 64-bit
elements), composed of a renaming unit capable to support 64 physical registers, a vector arithmetic and a
memory queue with sixteen entries each, these features can be setup to work with say, eight lanes. In contrast,
the VPU can also be targeted for the embedded market segment by setting a design for short vectors (8 64-
bit elements), reducing the number of available physical registers, and with only one lane configuration.

## Contact
Cristobal Ramirez Lazo: cristobal.ramirez@bsc.es
PhD. Student at UPC Barcelona
BSC - Barcelona Supercomputing Center