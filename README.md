# 5-Stage Pipelined RISC-V CPU (RV32I)

A synthesizable, educational RISC-V RV32I core with a classic 5-stage pipeline (IF, ID, EX, MEM, WB). Verified with simulation (VCD waveforms) and synthesized with XXX; optional FPGA targets.

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
