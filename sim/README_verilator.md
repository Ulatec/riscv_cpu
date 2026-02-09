# Verilator Simulation for RISC-V SoC

Fast simulation using Verilator (compiles Verilog to optimized C++).
Expect ~10-100x speedup vs iverilog/vvp.

## Setup (WSL / Ubuntu)

```bash
# Install Verilator
sudo apt update
sudo apt install -y verilator build-essential

# Verify version (need 4.210+ for unpacked arrays)
verilator --version
```

If your distro has an older Verilator, build from source:
```bash
sudo apt install -y git perl python3 make autoconf g++ flex bison libfl2 libfl-dev
git clone https://github.com/verilator/verilator.git
cd verilator
git checkout v5.024  # or latest stable
autoconf && ./configure && make -j$(nproc) && sudo make install
```

## Build

```bash
cd sim
make -f Makefile.verilator
```

## Run

```bash
make -f Makefile.verilator run
```

Or directly:
```bash
cd sim/obj_dir
./Vsoc
```

The simulation outputs UART text to stdout and prints progress every 5M cycles.
It exits automatically after detecting a shell prompt (`# ` or `$ `) + 2M grace cycles.

## Clean

```bash
make -f Makefile.verilator clean
```

## Troubleshooting

**Signal access errors**: If Verilator's API for internal signals changes between versions,
you may need to update `sim_main.cpp` signal access paths. The `rootp->soc__DOT__...` naming
follows Verilator's flattened hierarchy convention.

**Missing hex files**: Ensure `dts/fw_payload_earlycon.hex` and `dts/riscv_soc_dtb_le.hex`
exist. The simulation runs from `sim/obj_dir/` so paths are relative from there (`../../dts/...`
is handled via `../dts/...` from `sim/`).

**Memory**: 36MB RAM array requires ~144MB of process memory. Ensure sufficient RAM available.
