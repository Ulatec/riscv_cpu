# 5-Stage Pipelined RISC-V CPU (RV32I)

A synthesizable, educational RISC-V RV32I core with a classic 5-stage pipeline (IF, ID, EX, MEM, WB). Verified with simulation (VCD waveforms) and synthesized with Yosys; optional FPGA targets.

## Features
- RV32I subset (list which opcodes pass)
- Hazard handling (forwarding + simple stall logic)
- Branch/JAL/JALR control
- Testbench + sample firmware

## Quick start
```bash
make sim          # build + run simulation (Icarus)
make wave         # open waveform (gtkwave) if installed
make synth        # run yosys synthesis and write reports
