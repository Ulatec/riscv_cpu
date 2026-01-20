# RISC-V RV32IM SoC Implementation
## 5-Stage Pipelined Processor with SoC Integration

A hardware implementation of a RISC-V RV32IM system-on-chip in Verilog, featuring a 5-stage pipeline, data forwarding, multiply/divide operations, full CSR support, trap handling, timer/external interrupts, and a 16550-compatible UART.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Memory Map](#memory-map)
- [Supported Instructions](#supported-instructions)
- [Pipeline Stages](#pipeline-stages)
- [Modules](#modules)
- [Getting Started](#getting-started)
- [Design Decisions](#design-decisions)
- [Testing](#testing)
- [Future Enhancements](#future-enhancements)
- [Resources](#resources)

---

## Overview

This project implements a RISC-V RV32IM (32-bit integer base instruction set with multiply/divide extension) system-on-chip with a 5-stage pipeline architecture. The SoC integrates the CPU core with RAM, a CLINT timer unit, a PLIC interrupt controller, and a 16550-compatible UART for serial communication.

### Key Highlights

- **Complete RV32IM ISA**: All 40 base integer instructions + 8 M-extension instructions
- **5-Stage Pipeline**: IF, ID, EX, MEM, WB stages with proper hazard handling
- **Data Forwarding**: EX-to-EX and MEM-to-EX forwarding eliminates most data hazards
- **M-Extension**: Full multiply/divide support with RISC-V spec-compliant edge cases
- **Complete CSR Support**: All 6 CSR instructions with atomic read-modify-write
- **Trap Handling**: ECALL, EBREAK, and MRET with proper mstatus management
- **Timer Interrupts**: CLINT with mtime/mtimecmp for timer-based preemption
- **External Interrupts**: PLIC with 31 sources, priority/threshold, claim/complete
- **UART**: NS16550-compatible serial port with 16-byte TX/RX FIFOs
- **SoC Integration**: Unified memory bus with MMIO address decoding

---

## Features

### Instruction Support

#### RV32I Base (40 instructions)
- **R-Type**: ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU
- **I-Type Arithmetic**: ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU
- **Load Instructions**: LW, LH, LB, LHU, LBU
- **Store Instructions**: SW, SH, SB
- **Branch Instructions**: BEQ, BNE, BLT, BGE, BLTU, BGEU
- **Jump Instructions**: JAL, JALR
- **Upper Immediate**: LUI, AUIPC

#### RV32M Extension (8 instructions)
- **Multiply**: MUL, MULH, MULHSU, MULHU
- **Divide**: DIV, DIVU
- **Remainder**: REM, REMU

#### CSR Instructions (6 instructions)
- **Register-based**: CSRRW, CSRRS, CSRRC
- **Immediate-based**: CSRRWI, CSRRSI, CSRRCI

#### System Instructions
- **ECALL**: Environment call (triggers trap with mcause=11)
- **EBREAK**: Breakpoint (triggers trap with mcause=3)
- **MRET**: Return from machine-mode trap handler

### Interrupt System

- **Timer Interrupts**: CLINT provides mtime counter and mtimecmp comparator. When mtime >= mtimecmp, a machine timer interrupt (MTIP) is raised.
- **Software Interrupts**: MSIP register in CLINT for inter-hart software interrupts.
- **External Interrupts**: PLIC manages up to 31 interrupt sources with programmable priority and threshold. Supports claim/complete protocol for interrupt servicing.
- **Interrupt Flow**: `mip & mie & mstatus.MIE -> interrupt pending -> trap`

### Pipeline Features

- **Data Forwarding**: EX-to-EX and MEM-to-EX forwarding paths
- **Separate Store Forwarding**: Independent forwarding for store data (rs2)
- **Branch Resolution**: All 6 branch conditions evaluated in MEM stage
- **Jump Support**: JAL and JALR with proper PC+4 link and target calculation
- **Pipeline Flushing**: Automatic flush on control flow changes (branches, jumps, traps)

### M-Extension Edge Cases (RISC-V Specification)

| Operation | Division by Zero | Overflow (MIN_INT / -1) |
|-----------|-----------------|-------------------------|
| DIV | -1 | MIN_INT |
| DIVU | 2^32-1 | N/A |
| REM | dividend | 0 |
| REMU | dividend | N/A |

### CSR Support

Full machine-mode CSR implementation:

**Machine Information Registers** (read-only):
| Address | Name | Description |
|---------|------|-------------|
| 0xF11 | mvendorid | Vendor ID (0 for non-commercial) |
| 0xF12 | marchid | Architecture ID |
| 0xF13 | mimpid | Implementation ID |
| 0xF14 | mhartid | Hardware thread ID |

**Machine Trap Setup**:
| Address | Name | Description |
|---------|------|-------------|
| 0x300 | mstatus | Machine status (MIE, MPIE, MPP fields) |
| 0x301 | misa | ISA and extensions |
| 0x304 | mie | Machine interrupt enable |
| 0x305 | mtvec | Trap vector base address |

**Machine Trap Handling**:
| Address | Name | Description |
|---------|------|-------------|
| 0x340 | mscratch | Scratch register for trap handlers |
| 0x341 | mepc | Exception program counter |
| 0x342 | mcause | Trap cause code |
| 0x343 | mtval | Trap value (bad address/instruction) |
| 0x344 | mip | Interrupt pending |

**Performance Counters** (64-bit, read-only):
| Address | Name | Description |
|---------|------|-------------|
| 0xC00 | cycle | Cycle counter (low 32 bits) |
| 0xC80 | cycleh | Cycle counter (high 32 bits) |
| 0xC02 | instret | Instructions retired (low 32 bits) |
| 0xC82 | instreth | Instructions retired (high 32 bits) |

---

## Architecture

### SoC Block Diagram

```
                        +------------------+
                        |    RISC-V CPU    |
                        |    (RV32IM)      |
                        |  5-stage pipeline|
                        +--------+---------+
                                 |
                          Memory Bus (32-bit)
                                 |
            +----+----+----------+----------+----+
            |         |          |               |
     +------+--+ +----+---+ +---+----+  +-------+--+
     |  RAM    | | CLINT  | |  PLIC  |  |  UART    |
     | (configurable) | Timer  | | ExtIRQ |  | 16550  |
     | 0x80000000| 0x02000000| 0x0C000000| 0x10000000|
     +---------+ +--------+ +--------+  +----------+
```

### Pipeline Overview

```
+----------+    +----------+    +----------+    +----------+    +----------+
|    IF    | -> |    ID    | -> |    EX    | -> |   MEM    | -> |    WB    |
|  Fetch   |    |  Decode  |    | Execute  |    |  Memory  |    |  Write   |
+----------+    +----------+    +----------+    +----------+    +----------+
     |               |              |               |               |
   PC Reg      Register File      ALU           Data Mem       Reg Write
               Control Unit    (incl. M-ext)    CSR File
                                               Trap Logic
```

### Data Forwarding Paths

```
                    +---------------------------+
                    |     Forwarding Unit       |
                    +---------------------------+
                              |
                +-------------+-------------+
                |             |             |
                v             v             v
            EX/MEM        MEM/WB       ID/EX
            Result        Result        Inputs
                |             |
                +-------------+----------> ALU Inputs
                                          Store Data
```

### Trap Handling Flow

```
ECALL/EBREAK/Interrupt detected (MEM stage)
         |
         v
+---------------------------+
|  Save mepc <- trap_pc     |
|  Save mcause <- cause     |
|  mstatus.MPIE <- MIE      |
|  mstatus.MIE <- 0         |
|  mstatus.MPP <- M-mode    |
+---------------------------+
         |
         v
    next_pc <- mtvec
         |
         v
   [Trap Handler Code]
         |
         v
       MRET
         |
         v
+---------------------------+
|  mstatus.MIE <- MPIE      |
|  mstatus.MPIE <- 1        |
+---------------------------+
         |
         v
    next_pc <- mepc
```

---

## Memory Map

| Address Range | Region | Description |
|---------------|--------|-------------|
| 0x02000000 - 0x0200FFFF | CLINT | Timer (mtime @ 0x0200BFF8, mtimecmp @ 0x02004000) + MSIP |
| 0x0C000000 - 0x0CFFFFFF | PLIC | External interrupt controller (31 sources) |
| 0x10000000 - 0x1000001F | UART | 16550-compatible serial (mmio32, 4-byte register spacing) |
| 0x80000000 + | RAM | Main memory (configurable size via RAM_SIZE parameter) |

### UART Register Map (NS16550, 4-byte spacing)

| Offset | Read | Write | Description |
|--------|------|-------|-------------|
| 0x00 | RBR (Receive Buffer) | THR (Transmit Hold) | Data register |
| 0x04 | IER | IER | Interrupt Enable |
| 0x08 | IIR (Interrupt ID) | FCR (FIFO Control) | FIFO status/control |
| 0x0C | LCR | LCR | Line Control (DLAB, word length, parity) |
| 0x10 | MCR | MCR | Modem Control |
| 0x14 | LSR | - | Line Status (TX empty, RX ready) |

---

## Supported Instructions

### Instruction Format Summary

| Type | Format | Examples |
|------|--------|----------|
| R-Type | `op rd, rs1, rs2` | ADD, SUB, MUL, DIV |
| I-Type | `op rd, rs1, imm` | ADDI, LW, JALR |
| S-Type | `op rs2, imm(rs1)` | SW, SH, SB |
| B-Type | `op rs1, rs2, label` | BEQ, BNE, BLT |
| U-Type | `op rd, imm` | LUI, AUIPC |
| J-Type | `op rd, label` | JAL |

### R-Type (Register-Register Operations)
```assembly
ADD   rd, rs1, rs2    # rd = rs1 + rs2
SUB   rd, rs1, rs2    # rd = rs1 - rs2
AND   rd, rs1, rs2    # rd = rs1 & rs2
OR    rd, rs1, rs2    # rd = rs1 | rs2
XOR   rd, rs1, rs2    # rd = rs1 ^ rs2
SLL   rd, rs1, rs2    # rd = rs1 << rs2[4:0]
SRL   rd, rs1, rs2    # rd = rs1 >> rs2[4:0] (logical)
SRA   rd, rs1, rs2    # rd = rs1 >> rs2[4:0] (arithmetic)
SLT   rd, rs1, rs2    # rd = (rs1 < rs2) ? 1 : 0 (signed)
SLTU  rd, rs1, rs2    # rd = (rs1 < rs2) ? 1 : 0 (unsigned)
```

### I-Type Arithmetic
```assembly
ADDI  rd, rs1, imm    # rd = rs1 + imm
ANDI  rd, rs1, imm    # rd = rs1 & imm
ORI   rd, rs1, imm    # rd = rs1 | imm
XORI  rd, rs1, imm    # rd = rs1 ^ imm
SLLI  rd, rs1, shamt  # rd = rs1 << shamt
SRLI  rd, rs1, shamt  # rd = rs1 >> shamt (logical)
SRAI  rd, rs1, shamt  # rd = rs1 >> shamt (arithmetic)
SLTI  rd, rs1, imm    # rd = (rs1 < imm) ? 1 : 0 (signed)
SLTIU rd, rs1, imm    # rd = (rs1 < imm) ? 1 : 0 (unsigned)
```

### Load Instructions
```assembly
LW   rd, imm(rs1)     # rd = MEM[rs1 + imm] (word)
LH   rd, imm(rs1)     # rd = MEM[rs1 + imm] (halfword, sign-extended)
LB   rd, imm(rs1)     # rd = MEM[rs1 + imm] (byte, sign-extended)
LHU  rd, imm(rs1)     # rd = MEM[rs1 + imm] (halfword, zero-extended)
LBU  rd, imm(rs1)     # rd = MEM[rs1 + imm] (byte, zero-extended)
```

### Store Instructions
```assembly
SW   rs2, imm(rs1)    # MEM[rs1 + imm] = rs2 (word)
SH   rs2, imm(rs1)    # MEM[rs1 + imm] = rs2[15:0] (halfword)
SB   rs2, imm(rs1)    # MEM[rs1 + imm] = rs2[7:0] (byte)
```

### Branch Instructions
```assembly
BEQ   rs1, rs2, label # if (rs1 == rs2) PC = PC + offset
BNE   rs1, rs2, label # if (rs1 != rs2) PC = PC + offset
BLT   rs1, rs2, label # if (rs1 < rs2) PC = PC + offset (signed)
BGE   rs1, rs2, label # if (rs1 >= rs2) PC = PC + offset (signed)
BLTU  rs1, rs2, label # if (rs1 < rs2) PC = PC + offset (unsigned)
BGEU  rs1, rs2, label # if (rs1 >= rs2) PC = PC + offset (unsigned)
```

### Jump Instructions
```assembly
JAL   rd, label       # rd = PC + 4; PC = PC + offset
JALR  rd, imm(rs1)    # rd = PC + 4; PC = (rs1 + imm) & ~1
```

### Upper Immediate
```assembly
LUI   rd, imm         # rd = imm << 12
AUIPC rd, imm         # rd = PC + (imm << 12)
```

### M-Extension (Multiply/Divide)
```assembly
MUL    rd, rs1, rs2   # rd = (rs1 * rs2)[31:0]
MULH   rd, rs1, rs2   # rd = (rs1 * rs2)[63:32]    (signed * signed)
MULHSU rd, rs1, rs2   # rd = (rs1 * rs2)[63:32]    (signed * unsigned)
MULHU  rd, rs1, rs2   # rd = (rs1 * rs2)[63:32]    (unsigned * unsigned)
DIV    rd, rs1, rs2   # rd = rs1 / rs2              (signed)
DIVU   rd, rs1, rs2   # rd = rs1 / rs2              (unsigned)
REM    rd, rs1, rs2   # rd = rs1 % rs2              (signed)
REMU   rd, rs1, rs2   # rd = rs1 % rs2              (unsigned)
```

### CSR Instructions
```assembly
CSRRW  rd, csr, rs1   # rd = CSR; CSR = rs1
CSRRS  rd, csr, rs1   # rd = CSR; CSR = CSR | rs1
CSRRC  rd, csr, rs1   # rd = CSR; CSR = CSR & ~rs1
CSRRWI rd, csr, uimm  # rd = CSR; CSR = uimm
CSRRSI rd, csr, uimm  # rd = CSR; CSR = CSR | uimm
CSRRCI rd, csr, uimm  # rd = CSR; CSR = CSR & ~uimm
```

**Note**: When rs1=x0 for CSRRS/CSRRC (or uimm=0 for immediate variants), the CSR is not written (read-only operation).

### System Instructions
```assembly
ECALL                 # Environment call - trap to handler
EBREAK                # Breakpoint - trap to handler
MRET                  # Return from machine-mode trap
```

---

## Pipeline Stages

### 1. IF (Instruction Fetch)
- Fetch instruction from memory at PC
- Calculate PC+4
- Pass instruction and PC+4 to IF/ID register

### 2. ID (Instruction Decode)
- Decode opcode, funct3, funct7, register addresses
- Read from register file (rs1, rs2)
- Generate control signals via control unit
- Detect system instructions (ECALL, EBREAK, MRET)
- Extract and sign-extend immediate values

### 3. EX (Execute)
- **Forwarding Logic**: Select forwarded values if needed
- **ALU Operation**: Perform arithmetic/logic/multiply/divide
- **Branch Calculation**: Compute branch target address
- **Address Calculation**: Calculate memory address for loads/stores
- **CSR Operation**: Compute new CSR value (read-modify-write)

### 4. MEM (Memory Access)
- **Load**: Read data from memory, format based on LB/LH/LW
- **Store**: Write data to memory with proper byte enables
- **Branch Resolution**: Evaluate condition and determine if taken
- **Trap Detection**: Detect ECALL/EBREAK/interrupts, save trap state
- **CSR Write**: Write computed value to CSR file

### 5. WB (Write Back)
- Select data source (ALU result, memory data, CSR data, PC+4)
- Write to register file destination register

### Pipeline Registers

- **IF/ID**: Instruction, PC+4
- **ID/EX**: Control signals, register data, immediate, addresses, CSR address, trap flags
- **EX/MEM**: ALU result, store data, control signals, branch info, CSR data, trap signals
- **MEM/WB**: Memory data, ALU result, CSR read data, control signals

---

## Modules

### CPU Core

| Module | Lines | Description |
|--------|-------|-------------|
| `cpu.v` | ~750 | Main 5-stage pipeline with forwarding and trap handling |
| `control_unit.v` | ~155 | Instruction decoder for RV32IM + system instructions |
| `ALU.v` | ~90 | 18 operations (10 base + 8 M-extension) |
| `csr_file.v` | ~210 | CSR registers, trap entry/return, performance counters |
| `reg_file.v` | ~30 | 32-entry register file (x0 hardwired to 0) |
| `definitions.v` | ~20 | ALU opcode definitions |

### SoC Peripherals

| Module | Lines | Description |
|--------|-------|-------------|
| `soc.v` | ~220 | Top-level SoC: RAM, MMIO routing, address decoding |
| `uart.v` | ~485 | NS16550-compatible UART with 16-byte TX/RX FIFOs, hardware TX/RX serializer |
| `clint.v` | ~140 | CLINT: mtime, mtimecmp, MSIP, timer interrupt generation |
| `plic.v` | ~245 | PLIC: priority, pending, enable, threshold, claim/complete |

---

## Getting Started

### Prerequisites

- Verilog simulator ([Icarus Verilog](http://iverilog.icarus.com/), ModelSim, or Verilator)
- Waveform viewer (GTKWave) for debugging
- RISC-V toolchain (optional, for compiling test programs)

### Project Structure

```
riscv_cpu/
├── src/
│   ├── cpu.v              # Main CPU pipeline
│   ├── control_unit.v     # Instruction decoder
│   ├── ALU.v              # Arithmetic Logic Unit
│   ├── csr_file.v         # CSR register file
│   ├── reg_file.v         # Register file
│   ├── definitions.v      # ALU opcode constants
│   ├── soc.v              # SoC top-level integration
│   ├── uart.v             # 16550 UART
│   ├── clint.v            # Timer unit
│   ├── plic.v             # Interrupt controller
│   └── csr_tb.v           # CSR testbench
├── sim/
│   ├── clint_tb.v         # CLINT unit tests
│   ├── plic_tb.v          # PLIC unit tests
│   └── m_ext_tb.v         # M-extension tests
├── constr/
│   └── board_arty_s7.xdc  # FPGA constraints (Arty S7-25)
└── README.md
```

### Building and Running

```bash
# Compile and run a testbench
cd sim
iverilog -o test_out -g2005 <testbench>.v
vvp test_out

# Example: run M-extension tests
iverilog -o m_ext_test -g2005 m_ext_tb.v
vvp m_ext_test

# View waveforms
gtkwave test.vcd
```

### Example Test Program

```assembly
# Test program with CSR and M-extension
.text
.global _start

_start:
    # Setup trap handler
    la t0, trap_handler
    csrw mtvec, t0

    # Test M-extension
    li x1, 7
    li x2, 6
    mul x3, x1, x2        # x3 = 42

    li x4, 20
    li x5, 6
    div x6, x4, x5        # x6 = 3
    rem x7, x4, x5        # x7 = 2

    # Read cycle counter
    csrr x8, cycle

    # Test trap
    ecall                 # Triggers trap

    # Should return here after MRET
    li x9, 0xDEAD         # Success marker
    j end

trap_handler:
    # Simple trap handler
    csrr t0, mcause       # Read cause
    csrr t1, mepc         # Read exception PC
    addi t1, t1, 4        # Skip past ECALL
    csrw mepc, t1         # Write back
    mret                  # Return

end:
    j end                 # Spin
```

---

## Design Decisions

### Why 5-Stage Pipeline?

The classic 5-stage pipeline provides a good balance between simplicity, performance, and educational value. It reduces CPI compared to single-cycle while remaining easy to understand and debug.

### SoC Integration

The SoC wraps the CPU with a unified memory bus. Address decoding routes requests to the appropriate peripheral based on address ranges. RAM uses combinational reads for single-cycle access and registered writes with byte enables. MMIO peripherals are memory-mapped with 32-bit word access.

### Interrupt Architecture

The interrupt system follows the RISC-V privileged specification:
- **CLINT** provides timer interrupts (MTIP) by comparing mtime against mtimecmp
- **PLIC** manages external interrupts with programmable priority and a claim/complete protocol
- Interrupts are detected in the MEM stage alongside trap handling
- `mstatus.MIE` provides a global interrupt enable; `mie` masks individual sources

### Data Forwarding Strategy

Data forwarding is implemented with:
- **Two forwarding sources**: EX/MEM and MEM/WB stages
- **Priority**: EX/MEM (newer data) takes precedence
- **Separate store forwarding**: rs2 for stores forwarded independently

### M-Extension Implementation

All multiply/divide operations complete in a single cycle. Uses explicitly typed `wire signed` declarations for proper signed arithmetic handling. RISC-V-specified edge cases (division by zero, signed overflow) return defined values.

**Key Learning**: Verilog's `$signed()` cast in conditional assignments doesn't propagate signedness. Use declared signed wires:
```verilog
wire signed [31:0] signed_in1 = alu_in1;
wire signed [31:0] signed_in2 = alu_in2;
wire signed [31:0] signed_div = signed_in1 / signed_in2;
```

---

## Testing

### Test Coverage

| Category | Status | Tests |
|----------|--------|-------|
| RV32I Base | Pass | All arithmetic, load/store, branch, jump, upper-immediate |
| RV32M Extension | Pass | MUL/MULH/MULHSU/MULHU, DIV/DIVU, REM/REMU, edge cases |
| CSR Operations | Pass | CSRRW/S/C, immediate variants, read-only behavior, counters |
| Trap Handling | Pass | ECALL, EBREAK, mepc/mcause save, MRET return |
| CLINT | Pass | mtime increment, mtimecmp comparison, interrupt generation |
| PLIC | Pass | Priority, pending, enable, claim/complete protocol |

### Test Results

**M-Extension Tests (29 cases):**
```
M-Extension ALU Testbench
Test Summary: 29 passed, 0 failed
ALL TESTS PASSED!
```

**CSR Tests:**
```
========================================
CSR Testbench Started
========================================
Test 1: CSRRW read mscratch to t2 - PASS
Test 2: CSRRS set bits in mstatus - PASS
Test 3: CSRRC cleared bit 3 - PASS
Test 4: CSRRWI wrote immediate - PASS
Test 7: Cycle counter is incrementing - PASS
Test 8: Instret counter is incrementing - PASS
Test 9: Machine info registers readable - PASS
Test 10: CSRRS with rs1=x0 read-only - PASS
========================================
```

**CLINT Tests (14 cases):**
```
CLINT Testbench
Test Summary: 14 passed, 0 failed
ALL TESTS PASSED!
```

### Debug Features

- **Cycle Counter**: 64-bit hardware cycle counter
- **Instruction Counter**: 64-bit retired instruction counter
- **Debug Registers**: Track instructions in each pipeline stage
- **Waveform Support**: All signals accessible via VCD dump for GTKWave debugging

### Known Limitations

- **No Branch Prediction**: Assumes not taken; 3-cycle penalty on taken branches
- **M-Mode Only**: No supervisor or user privilege levels yet
- **Single-Cycle Division**: May limit maximum clock frequency on FPGA
- **No Virtual Memory**: Physical addresses only
- **No Compressed Instructions**: Only 32-bit instructions supported

---

## Future Enhancements

- [ ] C extension (compressed 16-bit instructions)
- [ ] Sv32 virtual memory (MMU with TLB)
- [ ] A extension (atomic operations: LR/SC, AMO)
- [ ] Privilege levels (S-mode, U-mode)
- [ ] Multi-cycle divider for improved timing
- [ ] FPGA synthesis and bring-up
- [ ] Boot a real operating system

---

## Resources

### RISC-V Specifications
- [RISC-V ISA Manual Volume 1](https://riscv.org/technical/specifications/) - Unprivileged Specification
- [RISC-V ISA Manual Volume 2](https://riscv.org/technical/specifications/) - Privileged Specification

### Learning Resources
- [Computer Organization and Design: RISC-V Edition](https://www.elsevier.com/books/computer-organization-and-design-risc-v-edition/patterson/978-0-12-812275-4) by Patterson & Hennessy
- [The RISC-V Reader](http://www.riscvbook.com/) by Patterson & Waterman

### Related Projects
- [PicoRV32](https://github.com/YosysHQ/picorv32) - Small RISC-V core
- [SERV](https://github.com/olofk/serv) - Smallest RISC-V CPU
- [Rocket Chip](https://github.com/chipsalliance/rocket-chip) - Full-featured RISC-V core

---

## Statistics

| Metric | Value |
|--------|-------|
| Total Lines of Verilog | ~2,345 |
| Core CPU Modules | 6 |
| SoC Peripheral Modules | 4 |
| Instructions Implemented | 54 (RV32IM + CSR + System) |
| Pipeline Stages | 5 |
| ALU Operations | 18 (10 base + 8 M-extension) |
| CSRs Implemented | 15 |
| Interrupt Sources | 31 (PLIC) + timer + software |

---

## Contributing

This is a personal learning project, but suggestions and improvements are welcome!

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

---

## License

This project is open source and available under the MIT License.

---

## Acknowledgments

- RISC-V Foundation for the excellent ISA specification
- Patterson & Hennessy for their textbook which served as the primary reference
- The open-source RISC-V community

---

## Contact

For questions or suggestions about this project, please open an issue on GitHub.

---

*Project Status: Active Development*
