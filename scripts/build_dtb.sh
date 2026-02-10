#!/bin/bash
# Compile the device tree blob for RV32IMAC SoC
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DTS_DIR="$SCRIPT_DIR/../dts"

cd "$DTS_DIR"

echo "=== Compiling Device Tree ==="
echo "Input:  riscv_soc.dts"
echo "Output: riscv_soc.dtb"

# Compile DTS to DTB
dtc -I dts -O dtb -o riscv_soc.dtb riscv_soc.dts

# Convert to hex for simulation (little-endian 32-bit words)
echo "=== Converting to hex for simulation ==="

# Create little-endian hex format
xxd -e -c 4 riscv_soc.dtb | awk '{print $2}' > riscv_soc_dtb_le.hex

echo "Generated files:"
ls -la riscv_soc.dtb riscv_soc_dtb_le.hex

echo ""
echo "DTB hex ready for simulation at:"
echo "  dts/riscv_soc_dtb_le.hex"
