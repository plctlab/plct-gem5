
# plct-gem5

## Overview

This repository is forked from https://github.com/RALC88/gem5/tree/develop. The original implementation is based on rvv-0.7.1 and it has been updated to be based on rvv-1.0 in this repository.

The implementation is based on the working draft of the proposed [RISC-V V vector extension v1.0 Spec](https://github.com/riscv/riscv-v-spec)

If you use this software or a modified version of it for your research, please cite the paper:
Cristóbal Ramirez, César Hernandez, Oscar Palomar, Osman Unsal, Marco Ramírez, and Adrián Cristal. 2020. A RISC-V Simulator and Benchmark Suite for Designing and Evaluating Vector Architectures. ACM Trans. Archit. Code Optim. 17, 4, Article 38 (October 2020), 29 pages. https://doi.org/10.1145/3422667

## Build gem5 for RVV

In current implementation, vector engine will be built by default when you build gem5 for RISC-V.
```
scons build/RISCV/gem5.opt
```

## Run program with gem5

To run binaries you can use following command:
```
${GEM5_DIR}build/RISCV/gem5.opt ${GEM5_DIR}configs/example/riscv_vector_engine.py --cmd="$program $program_args"
```

## Contributors and Contacts

Yin Zhang   zhangyin2018@iscas.ac.cn

Xuan Hu     huxuan21@mails.ucas.ac.cn

Liutong Han liutong2020@iscas.ac.cn


Original author:

Cristóbal Ramírez Lazo: cristobal.ramirez@bsc.es
PhD. Student at UPC Barcelona
BSC - Barcelona Supercomputing Center


# Original README

This is the gem5 simulator.

The main website can be found at http://www.gem5.org

A good starting point is http://www.gem5.org/about, and for
more information about building the simulator and getting started
please see http://www.gem5.org/documentation and
http://www.gem5.org/documentation/learning_gem5/introduction.

To build gem5, you will need the following software: g++ or clang,
Python (gem5 links in the Python interpreter), SCons, SWIG, zlib, m4,
and lastly protobuf if you want trace capture and playback
support. Please see http://www.gem5.org/documentation/general_docs/building
for more details concerning the minimum versions of the aforementioned tools.

Once you have all dependencies resolved, type 'scons
build/<ARCH>/gem5.opt' where ARCH is one of ARM, NULL, MIPS, POWER, SPARC,
or X86. This will build an optimized version of the gem5 binary (gem5.opt)
for the the specified architecture. See
http://www.gem5.org/documentation/general_docs/building for more details and
options.

The basic source release includes these subdirectories:
   - configs: example simulation configuration scripts
   - ext: less-common external packages needed to build gem5
   - src: source code of the gem5 simulator
   - system: source for some optional system software for simulated systems
   - tests: regression tests
   - util: useful utility programs and files

To run full-system simulations, you will need compiled system firmware
(console and PALcode for Alpha), kernel binaries and one or more disk
images.

If you have questions, please send mail to gem5-users@gem5.org

Enjoy using gem5 and please share your modifications and extensions.
