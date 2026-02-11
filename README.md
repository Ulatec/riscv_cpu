# RISC-V RV32IMAC SoC
## A Verilog CPU that Boots Linux

A from-scratch RISC-V RV32IMAC system-on-chip in Verilog. 5-stage pipeline, Sv32 MMU, M/S/U privilege modes. Boots OpenSBI, Linux 6.1.25, and runs a BusyBox shell.

![ISA](https://img.shields.io/badge/ISA-RV32IMAC-blue)
![Linux](https://img.shields.io/badge/Boots-Linux%206.1.25-green)
![Verilog](https://img.shields.io/badge/HDL-Verilog-orange)

---

## What It Does

This CPU boots a full Linux kernel from reset:

```
OpenSBI v1.8.1 (M-mode, 0x80000000)
  -> Platform init, UART/CLINT/PLIC setup
  -> Jump to Linux kernel (S-mode, 0x80400000)
    -> Sv32 MMU enabled, virtual memory active
    -> Device init, earlycon on ns16550 UART
    -> Initramfs extraction
    -> /init -> BusyBox shell
```

The simulation runs on Verilator and reaches a working shell prompt where you can execute commands.

---

## Features

### ISA Support

| Extension | Instructions | Description |
|-----------|-------------|-------------|
| **RV32I** | 40 | Base integer (ALU, load/store, branch, jump) |
| **M** | 8 | Multiply/divide (32-cycle iterative divider) |
| **A** | 11 | Atomics (LR/SC, AMO swap/add/and/or/xor/min/max) |
| **C** | 30+ | Compressed 16-bit instructions (RVC decompressor in IF stage) |

### Privilege Modes

| Mode | Level | Usage |
|------|-------|-------|
| **M-mode** | 3 | OpenSBI firmware, full hardware access |
| **S-mode** | 1 | Linux kernel, MMU-translated memory |
| **U-mode** | 0 | User processes (BusyBox/init) |

### Memory Management (Sv32)

- Two-level page table: 10-bit VPN[1] + 10-bit VPN[0] + 12-bit offset
- 4KB pages and 4MB superpages
- Hardware page table walker with automatic A/D bit updates (Svadu)
- 16-entry fully-associative TLB with ASID tagging and LRU replacement
- SFENCE.VMA support for TLB invalidation
- CSR-to-MMU forwarding: SUM/MXR bits forwarded from WB-stage CSR writes
- MPRV support: M-mode data accesses use MPP privilege when MPRV=1

### Pipeline

- 5-stage: IF / ID / EX / MEM / WB
- Data forwarding (EX-to-EX, MEM-to-EX, store data)
- Load-use and CSR-use hazard detection with pipeline stalls
- RVC decompression in IF stage with spanning instruction support
- Branch resolution in MEM stage with full pipeline flush
- Trap/interrupt detection in MEM stage

### Peripherals

- **CLINT**: 64-bit mtime counter, mtimecmp comparator, timer interrupts
- **PLIC**: 31 external interrupt sources, priority/threshold, claim/complete
- **UART**: NS16550-compatible, 16-byte FIFOs, hardware TX/RX serializer for FPGA

---

## Architecture

```
                    +----------------------------------+
                    |         RISC-V CPU (RV32IMAC)    |
                    |  +----+----+----+-----+----+     |
                    |  | IF | ID | EX | MEM | WB |     |
                    |  |RVC |    |ALU |MMU  |    |     |
                    |  |dec.|    |DIV |trap |    |     |
                    |  +----+----+----+-----+----+     |
                    |  |   CSR File   | CLINT | PLIC | |
                    +--------+-------------------------+
                             |
                      Memory Bus (32-bit)
                             |
              +--------------+--------------+
              |              |              |
        +-----+----+  +-----+----+  +------+-----+
        |   RAM    |  |   UART   |  | PTW Memory |
        | 0x80000000|  | 0x10000000|  |   Port     |
        +----------+  +----------+  +------------+
```

### Memory Map

| Address Range | Region | Description |
|--------------|--------|-------------|
| 0x02000000 | CLINT | Timer (mtime @ 0x0200BFF8, mtimecmp @ 0x02004000) |
| 0x0C000000 | PLIC | External interrupt controller (31 sources, 2 contexts) |
| 0x10000000 | UART | 16550-compatible serial (mmio32, 4-byte register spacing) |
| 0x80000000 | RAM | Main memory (36MB for Linux boot, configurable) |

---

## Source Files

### CPU Core

| File | Description |
|------|-------------|
| `src/cpu.v` | Main 5-stage pipeline with forwarding, MMU integration, CSR forwarding |
| `src/control_unit.v` | Instruction decoder: RV32IMAC + system instructions |
| `src/ALU.v` | 18 ALU operations (base + M extension) |
| `src/div_unit.v` | 32-cycle iterative restoring divider |
| `src/csr_file.v` | CSR registers, M/S-mode trap handling, interrupt delegation |
| `src/decompressor.v` | RVC 16-bit to 32-bit instruction expansion |
| `src/reg_file.v` | 32-entry register file (x0 hardwired to 0) |
| `src/definitions.v` | ALU opcode definitions |

### MMU

| File | Description |
|------|-------------|
| `src/mmu.v` | Sv32 MMU coordinator: TLB lookup, PTW arbitration, fault handling |
| `src/tlb.v` | 16-entry fully-associative TLB with ASID and superpage support |
| `src/page_table_walker.v` | Hardware 2-level Sv32 page table walker with A/D bit management |

### SoC

| File | Description |
|------|-------------|
| `src/soc.v` | Top-level SoC: RAM, MMIO routing, dual-port memory |
| `src/uart.v` | 16550 UART with FIFOs and hardware TX/RX for FPGA |
| `src/clint.v` | CLINT timer unit |
| `src/plic.v` | PLIC interrupt controller |
| `src/atomic_unit.v` | LR/SC reservation tracking + AMO operations |

---

## Getting Started

### Prerequisites

- **Simulation**: [Icarus Verilog](http://iverilog.icarus.com/) or [Verilator](https://www.veripool.org/verilator/)
- **Waveforms**: [GTKWave](http://gtkwave.sourceforge.net/)
- **Linux build** (optional): Docker + RISC-V cross-compiler (see `Dockerfile`)

### Quick Test (Bare-Metal)

```bash
cd sim
iverilog -o test -g2005 -DSIMULATION soc_tb.v
vvp test
```

### Linux Boot (Verilator)

```bash
# Build the Verilator model
cd sim
make -f Makefile.verilator

# Run with interactive UART
./obj_dir/Vsoc -i
```

### Build Linux Kernel + Initramfs (Docker)

```bash
docker build -t riscv-toolchain .
docker run -it --rm \
  -v "$(pwd):/output" \
  -v riscv-build:/work \
  riscv-toolchain bash /output/scripts/build_initramfs.sh
```

This produces `dts/fw_payload_earlycon.hex` containing OpenSBI + Linux kernel + BusyBox initramfs.

---

## Linux Boot Details

### Boot Arguments

```
earlycon=ns16550,mmio32,0x10000000,115200n8 console=ttyS0,115200n8 loglevel=8 lpj=1000
```

### Kernel Virtual Address Mapping

- Physical 0x80000000 -> Virtual 0xC0000000 (offset 0x40000000)
- 4MB superpages for kernel linear mapping
- MMIO identity maps: UART (0x10000000), CLINT (0x02000000), PLIC (0x0C000000)

### Software Stack

| Component | Version | Mode | Description |
|-----------|---------|------|-------------|
| OpenSBI | v1.8.1 | M-mode | SBI firmware, hardware abstraction |
| Linux | 6.1.25 | S-mode | Kernel with earlycon, initramfs |
| BusyBox | - | U-mode | Minimal userspace with shell |

---

## FPGA

The design includes synthesis support for the **Digilent Arty S7-25** (Spartan-7 XC7S25):

- `ifdef SIMULATION` / `ifndef SYNTHESIS` guards on all simulation constructs
- Hardware UART TX/RX serializer (baud rate generator + shift registers)
- MMCM clock generation (100MHz -> 50MHz)
- 32-cycle iterative divider (replaces combinational div for timing closure)

See `src/fpga_soc_wrapper.v`, `constr/board_arty_s7.xdc`, and `fpga/build.tcl`.

---

## Design Highlights

### Compressed Instruction Handling

The RVC decompressor sits in the IF stage. When PC[1]=1, a 32-bit instruction can span two memory words. The pipeline tracks this with `saved_half` / `have_saved` registers and a `spanning_pc` to record the true instruction address.

### Iterative Divider

Division uses a 32-cycle restoring algorithm in `div_unit.v`. The divider stalls the pipeline via `div_stall` merged into the MMU stall signal. Special cases (divide-by-zero, signed overflow MIN_INT/-1) resolve in a single cycle.

### CSR-to-MMU Forwarding

When a CSR write to mstatus/sstatus is in the WB stage, the new SUM and MXR bits are forwarded to the MMU so the instruction in the MEM stage sees the updated permission bits immediately. Without this, the first user memory access after `csrs sstatus, SUM` would fault.

### Pipeline Stall Hierarchy

```
Reset -> Pipeline Flush -> MMU/Div Stall (frozen) -> Data Hazard Stall -> Normal
```

MMU stalls (page table walks, division) freeze the entire pipeline. Data hazard stalls (load-use, CSR-use) insert a bubble in ID/EX while letting later stages drain.

---

## Resources

- [RISC-V ISA Manual Vol 1](https://riscv.org/technical/specifications/) - Unprivileged Specification
- [RISC-V ISA Manual Vol 2](https://riscv.org/technical/specifications/) - Privileged Specification
- [Computer Organization and Design: RISC-V Edition](https://www.elsevier.com/books/computer-organization-and-design-risc-v-edition/patterson/978-0-12-812275-4) - Patterson & Hennessy

---

## License

This project is open source and available under the MIT License.
