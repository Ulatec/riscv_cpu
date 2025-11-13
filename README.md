# 5-Stage Pipelined RISC-V CPU (RV32I)

A synthesizable, educational RISC-V RV32I core with a classic 5-stage pipeline (IF, ID, EX, MEM, WB). Verified with simulation (VCD waveforms) and synthesized with Synthesized in Vivado 2024.2 targeting Arty S7-50.

## Features
- RV32I subset (list which opcodes pass)
    - R-type: add
    - I-type: addi
    - U-type: lui
    - J-type: jal
- Hazard handling (forwarding + simple stall logic)
    - Data Forwarding Unit: Resolves Read-After-Write (RAW) hazards from:
        - EX/MEM stage to EX stage
        - MEM/WB stage to EX stage
    - Register File Bypass: Resolves RAW hazards from the WB stage to the ID stage (for simultaneous write/read).
    - (Stall logic for Load-Use hazards is pending verification)
- Control Flow
    - Successfully executes jal instructions with correct target address calculation.
- Testbench + sample firmware
    - Includes a testbench.v for loading firmware and dumping VCD waveforms.

## Firmware
The CPU’s instruction memory is pre-loaded from `fw/firmware.mem`, a plain text file of 32-bit hex words suitable for Verilog `$readmemh`.  


### FPGA (Arty S7-50) Quickstart

1. Open Vivado and create a new RTL project.
2. Add all Verilog files from `src/` as sources.
3. Add `fw/firmware.mem` as a Memory Initialization File.
4. Add `constr/board_arty_s7.xdc` as the constraints file.
5. Set `top_fpga` as the top module.
6. Run Synthesis → Implementation → Generate Bitstream.
7. Program the Arty; LED0 should blink, LED3 indicates out-of-reset.