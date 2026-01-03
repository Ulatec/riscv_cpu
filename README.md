# RISC-V RV32IM CPU Implementation
## 5-Stage Pipelined Processor with Trap Handling and CSR Support

A hardware implementation of a RISC-V RV32IM processor in Verilog, featuring a classic 5-stage pipeline, data forwarding, multiply/divide operations, full CSR support, and trap handling.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Supported Instructions](#supported-instructions)
- [Pipeline Stages](#pipeline-stages)
- [Modules](#modules)
- [Getting Started](#getting-started)
- [Design Decisions](#design-decisions)
- [Testing](#testing)
- [Future Enhancements](#future-enhancements)
- [Resources](#resources)

---

## 🎯 Overview

This project implements a RISC-V RV32IM (32-bit integer base instruction set with multiply/divide extension) processor with a 5-stage pipeline architecture. The design emphasizes clarity and educational value while maintaining functional correctness. It's designed as a learning project to deeply understand CPU architecture, pipelining, and the RISC-V ISA.

### Key Highlights

- **Complete RV32IM ISA**: All 40 base integer instructions + 8 M-extension instructions
- **5-Stage Pipeline**: IF, ID, EX, MEM, WB stages with proper hazard handling
- **Data Forwarding**: EX-to-EX and MEM-to-EX forwarding eliminates most data hazards
- **M-Extension**: Full multiply/divide support with RISC-V spec-compliant edge cases
- **Complete CSR Support**: All 6 CSR instructions fully functional with atomic read-modify-write
- **Trap Handling**: ECALL, EBREAK, and MRET with proper mstatus management
- **Performance Counters**: 64-bit cycle and instruction-retired counters
- **Modular Design**: Separate control unit and CSR file for maintainability

---

## ✨ Features

### Instruction Support

#### ✅ Fully Implemented - RV32I Base (40 instructions)
- **R-Type**: ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU
- **I-Type Arithmetic**: ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU
- **Load Instructions**: LW, LH, LB, LHU, LBU
- **Store Instructions**: SW, SH, SB
- **Branch Instructions**: BEQ, BNE, BLT, BGE, BLTU, BGEU
- **Jump Instructions**: JAL, JALR
- **Upper Immediate**: LUI, AUIPC

#### ✅ Fully Implemented - RV32M Extension (8 instructions)
- **Multiply**: MUL, MULH, MULHSU, MULHU
- **Divide**: DIV, DIVU
- **Remainder**: REM, REMU

#### ✅ Fully Implemented - CSR Instructions (6 instructions)
- **Register-based**: CSRRW, CSRRS, CSRRC
- **Immediate-based**: CSRRWI, CSRRSI, CSRRCI

#### ✅ Fully Implemented - System Instructions
- **ECALL**: Environment call (triggers trap with mcause=11)
- **EBREAK**: Breakpoint (triggers trap with mcause=3)
- **MRET**: Return from machine-mode trap handler

### Pipeline Features

- **Data Forwarding**: EX-to-EX and MEM-to-EX forwarding paths
- **Separate Store Forwarding**: Independent forwarding for store data (rs2)
- **Branch Resolution**: All 6 branch conditions evaluated in MEM stage
- **Jump Support**: JAL and JALR with proper PC+4 link and target calculation
- **Pipeline Flushing**: Automatic flush on control flow changes (branches, jumps, traps)

### M-Extension Features

The M-extension provides hardware multiply and divide operations:

- **64-bit Multiplication**: Full 64-bit product computed for MULH variants
- **Signed/Unsigned Support**: Proper handling of signed × signed, unsigned × unsigned, and mixed operands
- **Division Edge Cases**: RISC-V spec-compliant handling of:
  - Division by zero: Returns defined values (-1 for DIV, 0xFFFFFFFF for DIVU)
  - Signed overflow (MIN_INT ÷ -1): Returns MIN_INT for DIV, 0 for REM
- **Single-Cycle Operation**: All M-extension operations complete in one cycle

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
| 0x301 | misa | ISA and extensions (reports RV32I) |
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

### Trap Handling

Complete trap mechanism with proper state management:

- **Trap Entry** (on ECALL/EBREAK):
  - Saves PC to `mepc`
  - Saves cause code to `mcause`
  - Saves MIE to MPIE, clears MIE (disables interrupts)
  - Sets MPP to current privilege mode (M-mode)
  - Jumps to `mtvec`

- **Trap Return** (on MRET):
  - Restores MIE from MPIE
  - Sets MPIE to 1
  - Returns to address in `mepc`

### Memory Interface

- **Harvard Architecture**: Separate instruction and data memory interfaces
- **Byte-Addressable**: Full support for byte, halfword, and word accesses
- **Byte Enables**: 4-bit write strobe with proper alignment for SB/SH/SW
- **Aligned Store Data**: Data properly replicated for sub-word stores

---

## 🏗️ Architecture

### Pipeline Overview

```
┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐
│   IF    │───▶│   ID    │───▶│   EX    │───▶│   MEM   │───▶│   WB    │
│ Fetch   │    │ Decode  │    │ Execute │    │ Memory  │    │  Write  │
└─────────┘    └─────────┘    └─────────┘    └─────────┘    └─────────┘
     │              │              │              │              │
     │              │              │              │              │
   PC Reg      Register File     ALU          Data Mem      Reg Write
               Control Unit   (incl. M-ext)   CSR File
                                             Trap Logic
```

### Data Forwarding Paths

```
                    ┌──────────────────────────┐
                    │   Forwarding Unit        │
                    └──────────────────────────┘
                              │
                ┌─────────────┼─────────────┐
                │             │             │
                ▼             ▼             ▼
            EX/MEM        MEM/WB       ID/EX
            Result        Result        Inputs
                │             │
                └─────────────┴───────▶ ALU Inputs
                                       Store Data
```

### Trap Handling Flow

```
ECALL/EBREAK detected (MEM stage)
         │
         ▼
┌─────────────────────────┐
│  Save mepc ← trap_pc    │
│  Save mcause ← cause    │
│  mstatus.MPIE ← MIE     │
│  mstatus.MIE ← 0        │
│  mstatus.MPP ← M-mode   │
└─────────────────────────┘
         │
         ▼
    next_pc ← mtvec
         │
         ▼
   [Trap Handler Code]
         │
         ▼
       MRET
         │
         ▼
┌─────────────────────────┐
│  mstatus.MIE ← MPIE     │
│  mstatus.MPIE ← 1       │
└─────────────────────────┘
         │
         ▼
    next_pc ← mepc
```

### Pipeline Registers

- **IF/ID**: Instruction, PC+4
- **ID/EX**: Control signals, register data, immediate, addresses, CSR address, trap flags
- **EX/MEM**: ALU result, store data, control signals, branch info, CSR data, trap signals
- **MEM/WB**: Memory data, ALU result, CSR read data, control signals

---

## 📝 Supported Instructions

### Instruction Format Summary

| Type | Format | Examples |
|------|--------|----------|
| R-Type | `op rd, rs1, rs2` | ADD, SUB, MUL, DIV |
| I-Type | `op rd, rs1, imm` | ADDI, LW, JALR |
| S-Type | `op rs2, imm(rs1)` | SW, SH, SB |
| B-Type | `op rs1, rs2, label` | BEQ, BNE, BLT |
| U-Type | `op rd, imm` | LUI, AUIPC |
| J-Type | `op rd, label` | JAL |

### Detailed Instruction List

#### R-Type (Register-Register Operations)
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

#### M-Extension (Multiply/Divide)
```assembly
MUL    rd, rs1, rs2   # rd = (rs1 × rs2)[31:0]     (lower 32 bits)
MULH   rd, rs1, rs2   # rd = (rs1 × rs2)[63:32]    (upper, signed × signed)
MULHSU rd, rs1, rs2   # rd = (rs1 × rs2)[63:32]    (upper, signed × unsigned)
MULHU  rd, rs1, rs2   # rd = (rs1 × rs2)[63:32]    (upper, unsigned × unsigned)
DIV    rd, rs1, rs2   # rd = rs1 ÷ rs2             (signed)
DIVU   rd, rs1, rs2   # rd = rs1 ÷ rs2             (unsigned)
REM    rd, rs1, rs2   # rd = rs1 % rs2             (signed)
REMU   rd, rs1, rs2   # rd = rs1 % rs2             (unsigned)
```

**M-Extension Edge Cases (RISC-V Specification):**
| Operation | Division by Zero | Overflow (MIN_INT ÷ -1) |
|-----------|-----------------|-------------------------|
| DIV | -1 | MIN_INT |
| DIVU | 2³²-1 | N/A |
| REM | dividend | 0 |
| REMU | dividend | N/A |

#### I-Type Arithmetic
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

#### Load Instructions
```assembly
LW   rd, imm(rs1)     # rd = MEM[rs1 + imm] (word)
LH   rd, imm(rs1)     # rd = MEM[rs1 + imm] (halfword, sign-extended)
LB   rd, imm(rs1)     # rd = MEM[rs1 + imm] (byte, sign-extended)
LHU  rd, imm(rs1)     # rd = MEM[rs1 + imm] (halfword, zero-extended)
LBU  rd, imm(rs1)     # rd = MEM[rs1 + imm] (byte, zero-extended)
```

#### Store Instructions
```assembly
SW   rs2, imm(rs1)    # MEM[rs1 + imm] = rs2 (word)
SH   rs2, imm(rs1)    # MEM[rs1 + imm] = rs2[15:0] (halfword)
SB   rs2, imm(rs1)    # MEM[rs1 + imm] = rs2[7:0] (byte)
```

#### Branch Instructions
```assembly
BEQ   rs1, rs2, label # if (rs1 == rs2) PC = PC + offset
BNE   rs1, rs2, label # if (rs1 != rs2) PC = PC + offset
BLT   rs1, rs2, label # if (rs1 < rs2) PC = PC + offset (signed)
BGE   rs1, rs2, label # if (rs1 >= rs2) PC = PC + offset (signed)
BLTU  rs1, rs2, label # if (rs1 < rs2) PC = PC + offset (unsigned)
BGEU  rs1, rs2, label # if (rs1 >= rs2) PC = PC + offset (unsigned)
```

#### Jump Instructions
```assembly
JAL   rd, label       # rd = PC + 4; PC = PC + offset
JALR  rd, imm(rs1)    # rd = PC + 4; PC = (rs1 + imm) & ~1
```

#### Upper Immediate
```assembly
LUI   rd, imm         # rd = imm << 12
AUIPC rd, imm         # rd = PC + (imm << 12)
```

#### CSR Instructions
```assembly
CSRRW  rd, csr, rs1   # rd = CSR; CSR = rs1
CSRRS  rd, csr, rs1   # rd = CSR; CSR = CSR | rs1
CSRRC  rd, csr, rs1   # rd = CSR; CSR = CSR & ~rs1
CSRRWI rd, csr, uimm  # rd = CSR; CSR = uimm
CSRRSI rd, csr, uimm  # rd = CSR; CSR = CSR | uimm
CSRRCI rd, csr, uimm  # rd = CSR; CSR = CSR & ~uimm
```

**Note**: When rs1=x0 for CSRRS/CSRRC (or uimm=0 for immediate variants), the CSR is not written (read-only operation).

#### System Instructions
```assembly
ECALL                 # Environment call - trap to handler
EBREAK                # Breakpoint - trap to handler
MRET                  # Return from machine-mode trap
```

---

## 🔄 Pipeline Stages

### 1. IF (Instruction Fetch)
- Fetch instruction from instruction memory at PC
- Calculate PC+4
- Pass instruction and PC+4 to IF/ID register

### 2. ID (Instruction Decode)
- Decode opcode, funct3, funct7, register addresses
- Read from register file (rs1, rs2)
- Generate control signals via control unit
- Detect M-extension instructions via `funct7 == 7'b0000001`
- Detect system instructions (ECALL, EBREAK, MRET)
- Extract and sign-extend immediate values
- Pass decoded information to ID/EX register

### 3. EX (Execute)
- **Forwarding Logic**: Select forwarded values if needed
- **ALU Operation**: Perform arithmetic/logic/multiply/divide operation
- **Branch Calculation**: Compute branch target address
- **Address Calculation**: Calculate memory address for loads/stores
- **CSR Operation**: Compute new CSR value (read-modify-write)
- Pass results to EX/MEM register

### 4. MEM (Memory Access)
- **Load**: Read data from memory, format based on LB/LH/LW
- **Store**: Write data to memory with proper byte enables
- **Branch Resolution**: Evaluate condition and determine if taken
- **Trap Detection**: Detect ECALL/EBREAK, save trap state
- **CSR Write**: Write computed value to CSR file
- Pass data to MEM/WB register

### 5. WB (Write Back)
- Select data source (ALU result, memory data, CSR data, PC+4)
- Write to register file destination register

---

## 🧩 Modules

### Core Modules

#### `cpu.v` (Main CPU Module)
- Top-level module integrating all components
- Pipeline registers and forwarding logic
- Next-PC selection (branches, jumps, traps)
- ~670 lines

#### `control_unit.v` (Control Unit)
- Generates all control signals from instruction fields
- Determines instruction type (R, I, S, B, U, J, CSR, M-extension)
- Detects M-extension via `funct7 == 7'b0000001`
- Detects system instructions (ECALL, EBREAK, MRET)
- Outputs control signals for all pipeline stages
- ~140 lines

#### `reg_file.v` (Register File)
- 32 general-purpose registers (x0-x31)
- 2 read ports, 1 write port
- x0 hardwired to zero

#### `ALU.v` (Arithmetic Logic Unit)
- Performs arithmetic and logic operations
- **Base operations**: ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU
- **M-extension operations**: MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU
- Generates comparison flags for branches (alu_lt, alu_ltu, zero_flag)
- Handles signed/unsigned multiplication with 64-bit intermediate results
- Implements RISC-V-specified division edge cases
- ~90 lines

#### `csr_file.v` (CSR Register File)
- Implements all machine-mode CSRs
- 64-bit cycle and instret counters
- Trap entry logic (saves mepc, mcause, updates mstatus)
- MRET logic (restores mstatus)
- Proper CSR write masking (only writable bits modified)
- ~170 lines

#### `definitions.v` (Constants)
- ALU operation codes (5-bit encoding for 18 operations)
- ~20 lines

---

## 🚀 Getting Started

### Prerequisites

- Verilog simulator (iverilog, ModelSim, Verilator, etc.)
- RISC-V toolchain (for compiling test programs)
- Waveform viewer (GTKWave, ModelSim, etc.)

### Project Structure

```
riscv_cpu/
├── src/
│   ├── cpu.v             # Main CPU module
│   ├── control_unit.v    # Control unit
│   ├── reg_file.v        # Register file
│   ├── ALU.v             # Arithmetic Logic Unit (with M-extension)
│   ├── csr_file.v        # CSR register file
│   └── definitions.v     # Constants and definitions
├── test/
│   ├── csr_tb.v          # CSR testbench
│   └── m_ext_tb.v        # M-extension testbench
└── README.md             # This file
```

### Building and Running

#### Using Icarus Verilog

```bash
# Run CSR tests
iverilog -o csr_test test/csr_tb.v
./csr_test

# Run M-extension tests
iverilog -o m_ext_test test/m_ext_tb.v
./m_ext_test

# View waveforms
gtkwave csr_test.vcd
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

## 🎨 Design Decisions

### Why 5-Stage Pipeline?

The classic 5-stage pipeline provides a good balance between:
- **Simplicity**: Easy to understand and implement
- **Performance**: Reduces CPI compared to single-cycle
- **Educational Value**: Industry-standard architecture

### Why Separate Instruction/Data Memory?

Harvard architecture was chosen for:
- **Simplicity**: No structural hazards on memory access
- **Common in embedded**: Many RISC-V implementations use this
- **Better for FPGA**: Easier to implement with block RAM

### Data Forwarding Strategy

Data forwarding is implemented with:
- **Two forwarding sources**: EX/MEM and MEM/WB stages
- **Priority**: EX/MEM (newer data) takes precedence
- **Separate store forwarding**: rs2 for stores forwarded independently

### M-Extension Implementation

The M-extension was implemented with the following considerations:

- **Single-Cycle Operation**: All multiply/divide operations complete in one cycle
- **Signed Arithmetic Handling**: Uses explicitly typed `wire signed` declarations
- **RISC-V Specification Compliance**: Division by zero and signed overflow return defined values

**Key Learning**: Verilog's `$signed()` cast in conditional assignments doesn't propagate signedness. Use declared signed wires:
```verilog
wire signed [31:0] signed_in1 = alu_in1;
wire signed [31:0] signed_in2 = alu_in2;
wire signed [31:0] signed_div = signed_in1 / signed_in2;
```

### Trap Handling Design

Trap handling follows RISC-V privileged specification:
- **Detection in MEM stage**: Allows precise exceptions
- **Atomic state save**: mepc, mcause, mstatus updated together
- **Priority over CSR writes**: Trap handling takes precedence

---

## 🧪 Testing

### Test Coverage

The CPU has been tested with:

#### RV32I Base
- ✅ All arithmetic operations (R-type and I-type)
- ✅ All load/store variants (LB, LH, LW, LBU, LHU, SB, SH, SW)
- ✅ All 6 branch conditions (BEQ, BNE, BLT, BGE, BLTU, BGEU)
- ✅ Jump instructions (JAL, JALR)
- ✅ Upper immediate instructions (LUI, AUIPC)
- ✅ Data forwarding scenarios (EX-EX, MEM-EX)

#### RV32M Extension
- ✅ Multiply operations (MUL, MULH, MULHSU, MULHU)
- ✅ Divide operations (DIV, DIVU)
- ✅ Remainder operations (REM, REMU)
- ✅ Edge cases (divide by zero, signed overflow)

#### CSR Operations
- ✅ CSRRW, CSRRS, CSRRC (register variants)
- ✅ CSRRWI, CSRRSI, CSRRCI (immediate variants)
- ✅ Read-only behavior (rs1=x0 doesn't write)
- ✅ Cycle counter reading
- ✅ Instret counter reading
- ✅ Machine info register reading

#### Trap Handling
- ✅ ECALL triggers trap
- ✅ EBREAK triggers trap
- ✅ mepc correctly saved
- ✅ mcause correctly set
- ✅ mstatus fields updated
- ✅ MRET returns correctly

### Test Results

**M-Extension Tests:**
```
========================================
M-Extension ALU Testbench
========================================
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

### Known Limitations

- **No Branch Prediction**: Assumes not taken; 3-cycle penalty on taken branches
- **No Load-Use Stalling**: Software must avoid load-use hazards or use NOPs
- **Single-Cycle Division**: May limit maximum clock frequency on FPGA
- **M-Mode Only**: No supervisor or user privilege levels
- **No Interrupts**: Timer and external interrupts not yet implemented

### Debug Features

- **Cycle Counter**: 64-bit hardware cycle counter
- **Instruction Counter**: 64-bit retired instruction counter
- **Debug Registers**: Track instructions in each pipeline stage
- **Waveform Support**: All signals accessible for debugging

---

## 🔮 Future Enhancements

### Short Term
- [ ] Pipeline stall logic for load-use hazards
- [ ] Branch prediction (static: backward taken, forward not taken)
- [ ] Timer interrupts (CLINT)

### Medium Term
- [x] ~~M extension (multiply/divide)~~ ✅ **Completed!**
- [x] ~~CSR instructions~~ ✅ **Completed!**
- [x] ~~Trap handling (ECALL/EBREAK/MRET)~~ ✅ **Completed!**
- [ ] C extension (compressed instructions)
- [ ] Privilege levels (S-mode, U-mode)
- [ ] External interrupts (PLIC)
- [ ] Multi-cycle divider (for improved timing)

### Long Term
- [ ] A extension (atomic operations)
- [ ] Virtual memory (Sv32 MMU with TLB)
- [ ] F extension (single-precision floating point)
- [ ] Boot Linux!

---

## 📚 Resources

### RISC-V Specifications
- [RISC-V ISA Manual Volume 1](https://riscv.org/technical/specifications/) - Unprivileged Specification
- [RISC-V ISA Manual Volume 2](https://riscv.org/technical/specifications/) - Privileged Specification
- [RISC-V Assembly Programmer's Manual](https://github.com/riscv/riscv-asm-manual)

### Learning Resources
- [Computer Organization and Design: RISC-V Edition](https://www.elsevier.com/books/computer-organization-and-design-risc-v-edition/patterson/978-0-12-812275-4) by Patterson & Hennessy
- [The RISC-V Reader](http://www.riscvbook.com/) by Patterson & Waterman

### Related Projects
- [PicoRV32](https://github.com/YosysHQ/picorv32) - Small RISC-V core
- [SERV](https://github.com/olofk/serv) - Smallest RISC-V CPU
- [Rocket Chip](https://github.com/chipsalliance/rocket-chip) - Full-featured RISC-V core

---

## 📊 Statistics

| Metric | Value |
|--------|-------|
| Total Lines of Code | ~1,100 lines (Verilog) |
| Core Modules | 6 |
| Instructions Implemented | 54 (RV32IM + CSR + System) |
| Pipeline Stages | 5 |
| Pipeline Registers | 4 sets |
| ALU Operations | 18 (10 base + 8 M-extension) |
| CSRs Implemented | 15 |
| Forwarding Paths | 3 (ALU in1, ALU in2, Store data) |

---

## 🤝 Contributing

This is a personal learning project, but suggestions and improvements are welcome! 

### How to Contribute
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

---

## 📝 License

This project is open source and available under the MIT License.

---

## 🙏 Acknowledgments

- RISC-V Foundation for the excellent ISA specification
- Patterson & Hennessy for their textbook which served as the primary reference
- The open-source RISC-V community

---

## 📧 Contact

For questions or suggestions about this project, please open an issue on GitHub.

---

*Last Updated: January 2026*  
*Project Status: Active Development - RV32IM with Trap Handling Complete*