# =============================================================================
# Vivado Non-Project Mode Build Script
# RISC-V RV32IMAC SoC for Arty S7-25 (XC7S25CSGA324-1)
# =============================================================================
# Usage: vivado -mode batch -source build.tcl
# =============================================================================

set project_name "riscv_soc"
set part "xc7s25csga324-1"
set top_module "fpga_soc_wrapper"

# Paths relative to fpga/ directory
set src_dir "../src"
set constr_dir "../constr"
set fw_dir "../fw"
set out_dir "./output"

# Create output directory
file mkdir $out_dir

# =============================================================================
# Read source files (no reliance on `include)
# =============================================================================
read_verilog -sv [list \
    $src_dir/definitions.v \
    $src_dir/reg_file.v \
    $src_dir/ALU.v \
    $src_dir/div_unit.v \
    $src_dir/decompressor.v \
    $src_dir/clint.v \
    $src_dir/plic.v \
    $src_dir/atomic_unit.v \
    $src_dir/tlb.v \
    $src_dir/page_table_walker.v \
    $src_dir/mmu.v \
    $src_dir/csr_file.v \
    $src_dir/control_unit.v \
    $src_dir/uart.v \
    $src_dir/cpu.v \
    $src_dir/soc.v \
    $src_dir/fpga_soc_wrapper.v \
]

# =============================================================================
# Read constraints
# =============================================================================
read_xdc $constr_dir/board_arty_s7.xdc

# =============================================================================
# Copy firmware hex file to output directory for BRAM init
# =============================================================================
if {[file exists $fw_dir/firmware.hex]} {
    file copy -force $fw_dir/firmware.hex $out_dir/firmware.hex
    puts "INFO: Copied firmware.hex to output directory"
} else {
    puts "WARNING: firmware.hex not found in $fw_dir - RAM will be uninitialized"
}

# =============================================================================
# Synthesis
# =============================================================================
puts "INFO: Starting synthesis..."
synth_design \
    -top $top_module \
    -part $part \
    -verilog_define SYNTHESIS=1 \
    -flatten_hierarchy rebuilt \
    -retiming

# Post-synthesis reports
report_utilization -file $out_dir/post_synth_utilization.rpt
report_timing_summary -file $out_dir/post_synth_timing.rpt
puts "INFO: Synthesis complete. Utilization report: $out_dir/post_synth_utilization.rpt"

# =============================================================================
# Optimization
# =============================================================================
puts "INFO: Running optimization..."
opt_design

# =============================================================================
# Placement
# =============================================================================
puts "INFO: Running placement..."
place_design
report_utilization -file $out_dir/post_place_utilization.rpt
report_timing_summary -file $out_dir/post_place_timing.rpt

# =============================================================================
# Physical Optimization (optional, helps timing)
# =============================================================================
phys_opt_design

# =============================================================================
# Routing
# =============================================================================
puts "INFO: Running routing..."
route_design

# =============================================================================
# Post-Route Reports
# =============================================================================
report_utilization -file $out_dir/post_route_utilization.rpt
report_timing_summary -file $out_dir/post_route_timing.rpt
report_timing -max_paths 10 -file $out_dir/post_route_timing_detail.rpt
report_drc -file $out_dir/post_route_drc.rpt
report_power -file $out_dir/post_route_power.rpt

puts "INFO: Routing complete. Check timing: $out_dir/post_route_timing.rpt"

# Check for timing violations
set timing_slack [get_property SLACK [get_timing_paths -max_paths 1]]
if {$timing_slack < 0} {
    puts "WARNING: Timing violation detected! Worst slack: $timing_slack ns"
} else {
    puts "INFO: Timing met. Worst slack: $timing_slack ns"
}

# =============================================================================
# Generate Bitstream
# =============================================================================
puts "INFO: Generating bitstream..."
write_bitstream -force $out_dir/${project_name}.bit

# =============================================================================
# Generate flash programming file (MCS)
# =============================================================================
write_cfgmem -force -format mcs -interface SPIx4 -size 4 \
    -loadbit "up 0x0 $out_dir/${project_name}.bit" \
    $out_dir/${project_name}.mcs

puts "============================================="
puts "  Build complete!"
puts "  Bitstream: $out_dir/${project_name}.bit"
puts "  Flash MCS: $out_dir/${project_name}.mcs"
puts "============================================="
