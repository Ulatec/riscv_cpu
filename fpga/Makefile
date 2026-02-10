# =============================================================================
# FPGA Build Makefile for RISC-V SoC on Arty S7-25
# =============================================================================
# Prerequisites: Vivado in PATH, riscv32-unknown-elf toolchain for firmware
# =============================================================================

VIVADO ?= vivado
CROSS  ?= riscv32-unknown-elf

FW_DIR   = ../fw
SRC_DIR  = ../src
OUT_DIR  = ./output

.PHONY: all build firmware program flash clean

all: firmware build

# Build firmware hex file from assembly
firmware: $(FW_DIR)/firmware.hex

$(FW_DIR)/firmware.hex: $(FW_DIR)/fpga_firmware.S $(FW_DIR)/fpga_link.ld
	$(CROSS)-as -march=rv32imac -mabi=ilp32 -o $(FW_DIR)/firmware.o $(FW_DIR)/fpga_firmware.S
	$(CROSS)-ld -T $(FW_DIR)/fpga_link.ld -o $(FW_DIR)/firmware.elf $(FW_DIR)/firmware.o
	$(CROSS)-objcopy -O verilog $(FW_DIR)/firmware.elf $(FW_DIR)/firmware.hex
	$(CROSS)-objdump -d $(FW_DIR)/firmware.elf > $(FW_DIR)/firmware.dis
	@echo "Firmware built: $(FW_DIR)/firmware.hex"

# Run Vivado synthesis, place, route, bitstream
build:
	$(VIVADO) -mode batch -source build.tcl -log $(OUT_DIR)/vivado_build.log -journal $(OUT_DIR)/vivado_build.jou

# Program FPGA via JTAG
program:
	$(VIVADO) -mode batch -source program.tcl

# Program SPI flash for persistent boot
flash:
	$(VIVADO) -mode batch -source flash.tcl

clean:
	rm -rf $(OUT_DIR) .Xil vivado*.log vivado*.jou
	rm -f $(FW_DIR)/firmware.o $(FW_DIR)/firmware.elf $(FW_DIR)/firmware.hex $(FW_DIR)/firmware.dis
