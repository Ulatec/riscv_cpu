# =============================================================================
# Vivado JTAG Programming Script
# =============================================================================
# Usage: vivado -mode batch -source program.tcl
# =============================================================================

set bit_file "./output/riscv_soc.bit"

# Open hardware manager
open_hw_manager
connect_hw_server
open_hw_target

# Find the device
set device [lindex [get_hw_devices] 0]
current_hw_device $device

# Program
set_property PROGRAM.FILE $bit_file $device
program_hw_devices $device

puts "INFO: Device programmed with $bit_file"

close_hw_target
disconnect_hw_server
close_hw_manager
