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

See [README](README)