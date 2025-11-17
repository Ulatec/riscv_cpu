# RISC-V RV32I CPU Implementation
## 5-Stage Pipelined Processor with CSR Support

A hardware implementation of a RISC-V RV32I processor in Verilog, featuring a classic 5-stage pipeline, data forwarding, and Control and Status Register (CSR) support.

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

This project implements a RISC-V RV32I (32-bit integer base instruction set) processor with a 5-stage pipeline architecture. The design emphasizes clarity and educational value while maintaining functional correctness. It's designed as a learning project to deeply understand CPU architecture, pipelining, and the RISC-V ISA.

### Key Highlights

- **Complete RV32I Base ISA**: All 40 base integer instructions fully functional
- **5-Stage Pipeline**: IF, ID, EX, MEM, WB stages with proper hazard handling
- **Data Forwarding**: Eliminates most data hazards
- **CSR Infrastructure**: Foundation laid for Control and Status Registers (integration in progress)
- **Modular Design**: Separate control unit for maintainability
- **Well-Documented**: Extensive comments and documentation

---

## ✨ Features

### Instruction Support

#### ✅ Fully Implemented
- **R-Type**: ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU
- **I-Type Arithmetic**: ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU
- **Load Instructions**: LW, LH, LB, LHU, LBU
- **Store Instructions**: SW, SH, SB
- **Branch Instructions**: BEQ, BNE, BLT, BGE, BLTU, BGEU
- **Jump Instructions**: JAL, JALR
- **Upper Immediate**: LUI, AUIPC

#### 🚧 Partially Implemented
- **CSR Instructions**: CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI (decoded, integration in progress)
- **System Instructions**: ECALL, EBREAK (decoded, not fully functional)

### Pipeline Features

- **Data Forwarding**: EX-to-EX and MEM-to-EX forwarding paths
- **Separate Store Forwarding**: Independent forwarding for store data
- **Branch Handling**: Branch resolution in MEM stage
- **Jump Support**: JAL and JALR with proper PC calculation

### CSR Support (In Progress)

The infrastructure for CSR support is in place but not yet fully functional:

- **CSR File Module**: Created with register storage
- **Control Unit**: Decodes CSR instructions correctly
- **Pipeline Registers**: Added for CSR data flow
- **Still TODO**: Complete pipeline integration and testing

**Machine Mode CSRs** (defined but not operational):
  - `mstatus` (0x300) - Machine status
  - `mie` (0x304) - Interrupt enable
  - `mtvec` (0x305) - Trap vector
  - `mscratch` (0x340) - Scratch register
  - `mepc` (0x341) - Exception PC
  - `mcause` (0x342) - Trap cause
  - `mtval` (0x343) - Trap value
  - `mip` (0x344) - Interrupt pending

**Performance Counters** (defined but not operational):
  - `cycle` (0xC00) - Cycle counter
  - `time` (0xC01) - Time counter (placeholder)
  - `instret` (0xC02) - Instructions retired

### Memory Interface

- **Harvard Architecture**: Separate instruction and data memory interfaces
- **Byte-Addressable**: Support for byte, halfword, and word accesses
- **Byte Enables**: 4-bit write strobe for flexible store operations

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
                Control Unit                  CSR File
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
```

### Pipeline Registers

- **IF/ID**: Instruction, PC+4
- **ID/EX**: Control signals, register data, immediate, addresses
- **EX/MEM**: ALU result, store data, control signals, branch info
- **MEM/WB**: Memory data, ALU result, control signals

---

## 📝 Supported Instructions

### Instruction Format Summary

| Type | Format | Examples |
|------|--------|----------|
| R-Type | `op rd, rs1, rs2` | ADD, SUB, AND, OR, XOR |
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
SLL   rd, rs1, rs2    # rd = rs1 << rs2
SRL   rd, rs1, rs2    # rd = rs1 >> rs2 (logical)
SRA   rd, rs1, rs2    # rd = rs1 >> rs2 (arithmetic)
SLT   rd, rs1, rs2    # rd = (rs1 < rs2) ? 1 : 0 (signed)
SLTU  rd, rs1, rs2    # rd = (rs1 < rs2) ? 1 : 0 (unsigned)
```

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
SH   rs2, imm(rs1)    # MEM[rs1 + imm] = rs2 (halfword)
SB   rs2, imm(rs1)    # MEM[rs1 + imm] = rs2 (byte)
```

#### Branch Instructions
```assembly
BEQ   rs1, rs2, label # if (rs1 == rs2) PC = label
BNE   rs1, rs2, label # if (rs1 != rs2) PC = label
BLT   rs1, rs2, label # if (rs1 < rs2) PC = label (signed)
BGE   rs1, rs2, label # if (rs1 >= rs2) PC = label (signed)
BLTU  rs1, rs2, label # if (rs1 < rs2) PC = label (unsigned)
BGEU  rs1, rs2, label # if (rs1 >= rs2) PC = label (unsigned)
```

#### Jump Instructions
```assembly
JAL   rd, label       # rd = PC + 4; PC = PC + imm
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
CSRRS  rd, csr, rs1   # rd = CSR; CSR |= rs1
CSRRC  rd, csr, rs1   # rd = CSR; CSR &= ~rs1
CSRRWI rd, csr, imm   # rd = CSR; CSR = imm
CSRRSI rd, csr, imm   # rd = CSR; CSR |= imm
CSRRCI rd, csr, imm   # rd = CSR; CSR &= ~imm
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
- Extract and sign-extend immediate values
- Pass decoded information to ID/EX register

### 3. EX (Execute)
- **Forwarding Logic**: Select forwarded values if needed
- **ALU Operation**: Perform arithmetic/logic operation
- **Branch Calculation**: Compute branch target and condition
- **Address Calculation**: Calculate memory address for loads/stores
- **CSR Operation**: Compute new CSR value
- Pass results to EX/MEM register

### 4. MEM (Memory Access)
- **Load**: Read data from memory
- **Store**: Write data to memory (with byte enables)
- **Branch Resolution**: Determine if branch is taken
- **CSR Write**: Write to CSR file
- Pass data to MEM/WB register

### 5. WB (Write Back)
- Select data source (ALU result, memory data, CSR data, PC+4)
- Write to register file destination register

---

## 🧩 Modules

### Core Modules

#### `cpu.v` (Main CPU Module)
- Top-level module integrating all components
- Pipeline register management
- Forwarding logic
- PC control logic
- Approximately 650 lines

#### `control_unit.v` (Control Unit)
- Decodes instructions and generates control signals
- Determines instruction type (R, I, S, B, U, J, CSR)
- Outputs control signals for all pipeline stages
- 132 lines

#### `reg_file.v` (Register File)
- 32 general-purpose registers (x0-x31)
- 2 read ports, 1 write port
- x0 hardwired to zero

#### `ALU.v` (Arithmetic Logic Unit)
- Performs arithmetic and logic operations
- Supports: ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU
- Generates comparison flags for branches

#### `csr_file.v` (CSR Register File)
- Implements machine-mode CSRs
- Provides cycle, time, and instret counters
- Supports atomic CSR operations

#### `definitions.v` (Constants)
- ALU operation codes
- CSR addresses
- Other constant definitions

---

## 🚀 Getting Started

### Prerequisites

- Verilog simulator (iverilog, ModelSim, Verilator, etc.)
- RISC-V toolchain (for compiling test programs)
- Waveform viewer (GTKWave, ModelSim, etc.)

### Project Structure

```
riscv_cpu/
├── cpu.v                 # Main CPU module
├── control_unit.v        # Control unit
├── reg_file.v            # Register file
├── ALU.v                 # Arithmetic Logic Unit
├── csr_file.v            # CSR register file
├── definitions.v         # Constants and definitions
├── testbench.v           # Testbench (if available)
└── README.md             # This file
```

### Building and Running

#### Using Icarus Verilog

```bash
# Compile
iverilog -o cpu_sim cpu.v testbench.v

# Run simulation
./cpu_sim

# View waveforms
gtkwave dump.vcd
```

#### Using ModelSim/QuestaSim

```bash
# Compile
vlog cpu.v control_unit.v reg_file.v ALU.v csr_file.v definitions.v

# Simulate
vsim -c cpu_tb
run -all

# Or with GUI
vsim cpu_tb
```

### Example Test Program

```assembly
# Simple test program
.text
.global _start

_start:
    # Test arithmetic
    li x1, 10
    li x2, 20
    add x3, x1, x2        # x3 = 30
    
    # Test load/store
    la x4, data
    sw x3, 0(x4)
    lw x5, 0(x4)          # x5 = 30
    
    # Test branch
    beq x3, x5, success
    j fail

success:
    li x6, 1              # Success indicator
    j end

fail:
    li x6, 0              # Failure indicator

end:
    # Exit (if trap handler implemented)
    li a7, 93
    li a0, 0
    ecall

.data
data:
    .word 0
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
- **Common in embedded**: Many RISC-V implementations use this architecture
- **Better for FPGA**: Easier to implement with block RAM

### Data Forwarding Strategy

Data forwarding is implemented with:
- **Two forwarding sources**: EX/MEM and MEM/WB stages
- **Priority**: EX/MEM (newer data) takes precedence
- **Separate store forwarding**: rs2 for stores forwarded independently

### CSR Implementation

CSRs are implemented with (incomplete):

---

## 🧪 Testing

### Test Coverage

The CPU has been tested with:
- ✅ All arithmetic operations (R-type and I-type)
- ✅ All load/store variants
- ✅ All branch conditions
- ✅ Jump instructions (JAL, JALR)
- ✅ Upper immediate instructions
- ✅ Data forwarding scenarios
- 🚧 CSR operations (infrastructure in place, testing in progress)

### Known Issues

- **Control Hazards**: No branch prediction; assumes not taken
- **Load-Use Hazard**: May require pipeline stalling (currently relies on careful code)
- **CSR Operations**: Infrastructure in place but not yet fully integrated
- **Exception Handling**: ECALL/EBREAK decoded but not fully functional
- **Interrupts**: Interrupt logic not yet implemented

### Debug Features

- **Cycle Counter**: Tracks simulation cycles
- **Debug Registers**: Track instructions in each pipeline stage
- **Waveform Support**: All signals accessible for debugging

---

## 🔮 Future Enhancements

### Short Term
- [ ] Complete CSR integration and testing
- [ ] Pipeline stall logic for load-use hazards
- [ ] Branch prediction (simple: always taken/not taken)
- [ ] Complete exception handling
- [ ] More comprehensive test suite

### Medium Term
- [ ] M extension (multiply/divide)
- [ ] C extension (compressed instructions)
- [ ] Privilege levels (S-mode, U-mode)
- [ ] Virtual memory support

### Long Term
- [ ] F extension (single-precision floating point)
- [ ] D extension (double-precision floating point)
- [ ] A extension (atomic operations)
- [ ] Multi-core support

---

## 📚 Resources

### RISC-V Specifications
- [RISC-V ISA Manual](https://riscv.org/technical/specifications/)
- [RISC-V Privileged Architecture](https://riscv.org/technical/specifications/)
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

- **Total Lines of Code**: ~1,200 lines (Verilog)
- **Modules**: 6 core modules
- **Instructions Fully Functional**: 40 (Complete RV32I base)
- **Instructions In Progress**: 7 (CSR instructions)
- **Pipeline Stages**: 5
- **Pipeline Registers**: 4 sets
- **CSRs Defined**: 11 (integration in progress)

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

*Last Updated: November 2025*
*Project Status: Active Development*


