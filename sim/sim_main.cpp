// =============================================================================
// Verilator C++ Testbench for RISC-V SoC
// Replaces linux_shell_tb.v for ~10-100x faster simulation
//
// Usage:
//   ./Vsoc              # Test mode: boot to shell, print results, exit
//   ./Vsoc -i           # Interactive mode: boot to shell, then accept input
//   ./Vsoc --interactive # Same as -i
// =============================================================================

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include "Vsoc.h"
#include "Vsoc___024root.h"
#include "verilated.h"

// ============================================================================
// Configuration
// ============================================================================
static const uint32_t RAM_SIZE     = 0x2400000;   // 36MB
static const uint32_t RAM_WORDS    = RAM_SIZE / 4;
static const uint32_t RAM_BASE     = 0x80000000;
static const uint32_t DTB_ADDR     = 0x82200000;
static const uint64_t MAX_CYCLES   = 160000000ULL; // 160M - full boot
static const int      MAX_UART     = 1000000;

// ============================================================================
// RAM accessor — abstracts Verilator's unpacked array representation
// In Verilator, reg [31:0] ram [0:N-1] becomes VlUnpacked<uint32_t, N>
// ============================================================================
#define RAM(i) (top->rootp->soc__DOT__ram[(i)])

// ============================================================================
// Terminal raw mode for interactive input
// ============================================================================
static struct termios orig_termios;
static bool raw_mode_active = false;

static void disable_raw_mode() {
    if (raw_mode_active) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
        raw_mode_active = false;
    }
}

static void enable_raw_mode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    raw_mode_active = true;
    atexit(disable_raw_mode);

    struct termios raw = orig_termios;
    raw.c_iflag &= ~(ICRNL | IXON | IXOFF | INLCR | IGNCR);
    raw.c_lflag &= ~(ECHO | ICANON | ISIG | IEXTEN);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

// Signal handler for clean exit
static volatile bool quit_requested = false;
static void signal_handler(int) { quit_requested = true; }

// ============================================================================
// Hex file loader (Verilog $readmemh format)
// ============================================================================
static bool load_hex(const char *filename, Vsoc *top, uint32_t offset = 0) {
    FILE *f = fopen(filename, "r");
    if (!f) return false;

    char line[256];
    uint32_t addr = offset;

    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '/' || *p == '\n' || *p == '\r' || *p == '\0') continue;

        if (*p == '@') {
            addr = (uint32_t)strtoul(p + 1, NULL, 16);
            continue;
        }

        uint32_t val = (uint32_t)strtoul(p, NULL, 16);
        if (addr < RAM_WORDS) {
            RAM(addr) = val;
        }
        addr++;
    }

    fclose(f);
    printf("  Loaded %s (last addr: 0x%x)\n", filename, addr - 1);
    return true;
}

// ============================================================================
// Ground truth comparison: load decompressed initramfs for byte-by-byte check
// ============================================================================
static uint8_t *gt_data = nullptr;
static uint32_t gt_size = 0;

static void load_ground_truth() {
    const char *paths[] = {"initramfs_ground_truth.bin",
                           "../sim/initramfs_ground_truth.bin", NULL};
    for (int p = 0; paths[p]; p++) {
        FILE *f = fopen(paths[p], "rb");
        if (!f) continue;
        fseek(f, 0, SEEK_END);
        gt_size = (uint32_t)ftell(f);
        fseek(f, 0, SEEK_SET);
        gt_data = new uint8_t[gt_size];
        size_t nr = fread(gt_data, 1, gt_size, f);
        fclose(f);
        if (nr == gt_size) {
            fprintf(stderr, "Ground truth loaded: %s (%u bytes)\n", paths[p], gt_size);
            return;
        }
        delete[] gt_data; gt_data = nullptr; gt_size = 0;
    }
    fprintf(stderr, "WARNING: ground truth file not found (optional)\n");
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);

    // Parse arguments
    bool interactive = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--interactive") == 0)
            interactive = true;
    }

    Vsoc *top = new Vsoc;

    printf("=== RISC-V SoC Linux Boot (Verilator) ===\n");
    printf("Mode: %s\n", interactive ? "INTERACTIVE" : "TEST");
    printf("RAM: %u MB  Max cycles: %lluM\n",
           RAM_SIZE / (1024 * 1024), (unsigned long long)MAX_CYCLES / 1000000);

    // ====================================================================
    // Reset sequence — MUST run before loading hex files, because
    // Verilator executes Verilog 'initial' blocks during the first
    // eval(), which zeros the RAM array in soc.v.
    // ====================================================================
    top->rst = 1;
    top->uart_rx = 1;
    top->clk = 0;

    for (int i = 0; i < 100; i++) {
        top->clk = 0; top->eval();
        top->clk = 1; top->eval();
    }

    top->rst = 0;

    // ====================================================================
    // Load firmware and DTB (after reset so initial block doesn't wipe RAM)
    // ====================================================================
    const char *fw_paths[]  = {"../../dts/fw_payload_earlycon.hex",
                               "../dts/fw_payload_earlycon.hex", NULL};
    const char *dtb_paths[] = {"../../dts/riscv_soc_dtb_le.hex",
                               "../dts/riscv_soc_dtb_le.hex", NULL};

    bool fw_ok = false;
    for (int p = 0; fw_paths[p] && !fw_ok; p++)
        fw_ok = load_hex(fw_paths[p], top, 0);
    if (!fw_ok) {
        fprintf(stderr, "ERROR: Cannot load firmware hex file\n");
        delete top; return 1;
    }

    bool dtb_ok = false;
    for (int p = 0; dtb_paths[p] && !dtb_ok; p++)
        dtb_ok = load_hex(dtb_paths[p], top, (DTB_ADDR - RAM_BASE) / 4);
    if (!dtb_ok) {
        fprintf(stderr, "ERROR: Cannot load DTB hex file\n");
        delete top; return 1;
    }

    // ====================================================================
    // Auto-detect and apply relocation table
    // ====================================================================
    uint32_t rela_start = 0;
    bool found_rela = false;

    for (uint32_t scan = 0x8000; scan < 0x20000 && !found_rela; scan++) {
        if ((RAM(scan + 1) & 0xFF) == 0x03 &&
            (RAM(scan + 4) & 0xFF) == 0x03 &&
            (RAM(scan + 7) & 0xFF) == 0x03 &&
            RAM(scan)     < RAM_SIZE &&
            RAM(scan + 3) < RAM_SIZE) {
            rela_start = scan;
            found_rela = true;
        }
    }

    if (found_rela) {
        uint32_t rela_end = rela_start;
        for (uint32_t scan = rela_start; scan < rela_start + 0x3000; scan += 3) {
            if ((RAM(scan + 1) & 0xFF) == 0x03 && RAM(scan) < RAM_SIZE)
                rela_end = scan + 3;
            else
                break;
        }

        int rela_applied = 0;
        for (uint32_t idx = rela_start; idx < rela_end; idx += 3) {
            uint32_t r_offset = RAM(idx);
            uint32_t r_info   = RAM(idx + 1);
            uint32_t r_addend = RAM(idx + 2);
            if ((r_info & 0xFF) == 0x03) {
                RAM(r_offset >> 2) = RAM_BASE + r_addend;
                rela_applied++;
            }
        }
        printf("Relocation: word 0x%x, applied %d entries\n",
               rela_start, rela_applied);
    } else {
        printf("WARNING: No relocation table found\n");
    }

    // Verify initramfs data loaded correctly
    // __initramfs_start = VA 0xC080A214, PA = 0x80C0A214, RAM word = 0x302885
    {
        uint32_t initramfs_word = 0x302885;
        printf("Initramfs header (PA 0x80C0A214, word 0x%x):", initramfs_word);
        for (int i = 0; i < 8; i++)
            printf(" %08x", RAM(initramfs_word + i));
        printf("\n");
        // First 2 bytes should be 0x1f 0x8b (gzip) or "070701" (cpio)
        uint8_t b0 = RAM(initramfs_word) & 0xFF;
        uint8_t b1 = (RAM(initramfs_word) >> 8) & 0xFF;
        if (b0 == 0x1f && b1 == 0x8b)
            printf("  -> gzip compressed initramfs detected\n");
        else if (b0 == '0' && b1 == '7')
            printf("  -> cpio newc format detected\n");
        else
            printf("  -> WARNING: unexpected header bytes: 0x%02x 0x%02x\n", b0, b1);
        // __initramfs_size at VA 0xC08C4500, PA = 0x80CC4500, word = 0x331140
        uint32_t size_word = 0x331140;
        uint32_t initramfs_sz = RAM(size_word);
        printf("  __initramfs_size (word 0x%x) = 0x%08x (%u bytes)\n",
               size_word, initramfs_sz, initramfs_sz);
    }

    // Compute initramfs checksum for integrity verification
    uint32_t initramfs_cksum = 0;
    const uint32_t INITRAMFS_WORD = 0x302885;
    const uint32_t INITRAMFS_WORDS = 762600 / 4; // ~190650 words
    for (uint32_t i = 0; i < INITRAMFS_WORDS; i++)
        initramfs_cksum ^= RAM(INITRAMFS_WORD + i);
    printf("  Initramfs checksum: 0x%08x (%u words)\n", initramfs_cksum, INITRAMFS_WORDS);

    // Verify hex loading by spot-checking words
    printf("  Verify: RAM[0]=0x%08x RAM[1]=0x%08x RAM[2]=0x%08x (expect 00050433 000584b3 00060933)\n",
           RAM(0), RAM(1), RAM(2));
    printf("  Verify: RAM[0x100000]=0x%08x RAM[0x200000]=0x%08x RAM[0x300000]=0x%08x\n",
           RAM(0x100000), RAM(0x200000), RAM(0x300000));

    // Dump printk_rb_static structure (VA 0xc140bbac, PA 0x8180bbac, word 0x602EEB)
    // desc_ring: count_bits[0] descs[1] infos[2] head_id[3] tail_id[4]
    // text_data_ring: count_bits[5] data[6] head_lpos[7] tail_lpos[8]
    printf("  printk_rb_static (word 0x602EEB, VA 0xc140bbac):\n");
    for (int i = 0; i < 12; i++)
        printf("    [%d] 0x%08x\n", i, RAM(0x602EEB + i));
    // desc_ring.count_bits should be 12 (0x0c), descs=0xc1463c40, infos=0xc140bc40

    // Full RAM checksum for comparison with iverilog
    {
        uint32_t cksum = 0;
        for (uint32_t i = 0; i < 0x900000; i++)
            cksum ^= RAM(i);
        printf("  Full RAM checksum (0x900000 words): 0x%08x\n", cksum);
    }

    // Verify initcall table: VA 0xC0809E14, PA 0x80C09E14, word 0x302785
    {
        uint32_t ic_word = 0x302785;
        printf("  __initcall_start (word 0x%x, VA 0xc0809e14):\n", ic_word);
        for (int i = 0; i < 20; i++)
            printf("    [%d] 0x%08x\n", i, RAM(ic_word + i));
        // Count NULL entries
        int nulls = 0;
        // initcall table spans 0xC0809E14 to 0xC080A200 = 0x1EC bytes = 123 entries
        for (int i = 0; i < 123; i++)
            if (RAM(ic_word + i) == 0) nulls++;
        printf("  Initcall table: %d NULL entries out of 123\n", nulls);
    }

    // Load ground truth for decompression comparison
    load_ground_truth();

    // Set boot parameters: a0=hartid(0), a1=DTB address
    top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile[10] = 0;
    top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile[11] = DTB_ADDR;

    // ====================================================================
    // Simulation state
    // ====================================================================
    uint32_t prev_satp = 0;
    bool entered_smode = false;
    bool saw_kernel_virt = false;
    bool saw_shell_prompt = false;
    uint64_t shell_cycle = 0;
    int uart_count = 0;
    int page_faults = 0;
    uint8_t prev_char = 0;
    bool rx_injected = false;  // track if we injected a char last cycle

    printf("Starting simulation...\n");
    if (interactive)
        printf("  Boot takes ~5 minutes. Shell will become interactive after.\n"
               "  Ctrl+C to abort boot. Ctrl+] to quit after shell is ready.\n");
    fflush(stdout);

    // ====================================================================
    // Main simulation loop
    // ====================================================================
    uint64_t cycle;
    for (cycle = 0; cycle < MAX_CYCLES && !quit_requested; cycle++) {

        // ---- UART RX injection (before clock edge) ----
        if (rx_injected) {
            top->rootp->soc__DOT__uart_sim_rx_valid = 0;
            rx_injected = false;
        }

        // Interactive mode: read from stdin
        if (interactive && saw_shell_prompt && (cycle % 512 == 0)) {
            char ch;
            ssize_t n = read(STDIN_FILENO, &ch, 1);
            if (n == 1) {
                if (ch == 0x1d) {  // Ctrl+]
                    printf("\n[Ctrl+] pressed — exiting]\n");
                    break;
                }
                // Convert Enter (\r) to newline (\n) for the shell
                if (ch == '\r') ch = '\n';
                top->rootp->soc__DOT__uart_sim_rx_data = (uint8_t)ch;
                top->rootp->soc__DOT__uart_sim_rx_valid = 1;
                rx_injected = true;
            }
        }

        // ---- Clock cycle ----
        top->clk = 0;
        top->eval();

        // Capture bus signals BEFORE posedge (these drive the RAM write)
        uint32_t pre_bus_addr  = top->rootp->soc__DOT__mem_addr;
        uint8_t  pre_bus_wstrb = top->rootp->soc__DOT__mem_wstrb;
        uint32_t pre_bus_wdata = top->rootp->soc__DOT__mem_wdata;
        uint32_t pre_em_pc     = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_pc;
        uint8_t  pre_ptw_req   = top->rootp->soc__DOT__cpu_inst__DOT__ptw_mem_req;
        uint8_t  pre_ptw_wr    = top->rootp->soc__DOT__cpu_inst__DOT__ptw_mem_write;

        // Capture trap/pipeline state BEFORE posedge — this is the state that
        // the always @(posedge clk) block will act on. Reading these signals
        // after the posedge gives the NEXT cycle's combinational state, not
        // what was actually processed this cycle.
        uint8_t  pre_trap_taken = top->rootp->soc__DOT__cpu_inst__DOT__take_trap_effective;
        uint32_t pre_trap_cause = top->rootp->soc__DOT__cpu_inst__DOT__trap_cause_final;
        uint32_t pre_id_ex_pc   = top->rootp->soc__DOT__cpu_inst__DOT__id_ex_pc;
        uint32_t pre_if_id_pc   = top->rootp->soc__DOT__cpu_inst__DOT__if_id_pc;
        uint32_t pre_pc_reg     = top->rootp->soc__DOT__cpu_inst__DOT__pc_reg;
        uint8_t  pre_priv       = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__priv_level;
        uint8_t  pre_data_stall = top->rootp->soc__DOT__cpu_inst__DOT__data_hazard_stall;
        uint8_t  pre_mmu_stall  = top->rootp->soc__DOT__cpu_inst__DOT__mmu_stall;
        uint8_t  pre_trap_to_s  = top->rootp->soc__DOT__cpu_inst__DOT__trap_to_s_mode;
        uint8_t  pre_exception  = top->rootp->soc__DOT__cpu_inst__DOT__exception_taken;
        uint8_t  pre_pflush     = top->rootp->soc__DOT__cpu_inst__DOT__pipeline_flush;
        uint32_t pre_ex_mem_mem_write = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_mem_write;
        uint8_t  pre_ex_mem_csr_write = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_csr_write;
        uint32_t pre_ex_mem_csr_addr = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_csr_addr;
        uint32_t pre_ex_mem_csr_wdata = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_csr_wdata;

        // Pre-posedge flush cause captures
        uint8_t pre_em_branch = top->rootp->soc__DOT__cpu_inst__DOT__take_branch_condition;
        uint8_t pre_em_jal    = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_isJAL;
        uint8_t pre_em_jalr   = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_isJALR;
        uint8_t pre_em_mret   = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_is_mret;
        uint8_t pre_em_sret   = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_is_sret;
        uint8_t pre_em_sfence = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_is_sfence_vma;
        uint8_t pre_em_btype  = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_isBtype;
        uint8_t pre_take_trap = top->rootp->soc__DOT__cpu_inst__DOT__take_trap;
        uint8_t pre_can_int   = top->rootp->soc__DOT__cpu_inst__DOT__can_take_interrupt;
        uint32_t pre_em_pcpls = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_pcplus4;
        uint32_t pre_em_btgt  = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_branch_target;
        uint32_t pre_next_pc  = top->rootp->soc__DOT__cpu_inst__DOT__next_pc;

        // Additional pre-posedge captures for U-mode trace
        uint8_t  pre_ex_mem_mr  = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_mem_read;
        uint32_t pre_alu_result  = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_alu_result;
        uint32_t pre_data_paddr_v = top->rootp->soc__DOT__cpu_inst__DOT__data_paddr;
        uint32_t pre_rdata_v     = top->rootp->soc__DOT__mem_rdata;
        uint8_t  pre_ex_mem_rd_v = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_rd_addr;
        uint8_t  pre_ex_mem_rw_v = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_reg_write;
        uint32_t pre_wb_data_v   = top->rootp->soc__DOT__cpu_inst__DOT__write_data_to_reg;
        uint8_t  pre_wb_rd_v     = top->rootp->soc__DOT__cpu_inst__DOT__mem_wb_rd_addr;
        uint8_t  pre_wb_rw_v     = top->rootp->soc__DOT__cpu_inst__DOT__mem_wb_reg_write;

        // Check for pipeline anomaly: id_ex_pc == em_pc should never happen
        // in normal operation (same instruction in both EX and MEM stages)
        if (pre_id_ex_pc != 0 && pre_id_ex_pc == pre_em_pc && cycle > 100) {
            static int dup_count = 0;
            dup_count++;
            if (dup_count <= 30) {
                uint8_t pflush = top->rootp->soc__DOT__cpu_inst__DOT__pipeline_flush;
                uint8_t ex_mem_read = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_mem_read;
                uint8_t id_ex_read = top->rootp->soc__DOT__cpu_inst__DOT__id_ex_mem_read;
                uint8_t div_stall = top->rootp->soc__DOT__cpu_inst__DOT__div_stall;
                fprintf(stderr, "[%llu] DUP_PC #%d id_ex=em=0x%08x if_id=0x%08x pc=0x%08x "
                       "stall=%d mmu=%d pflush=%d div=%d trap=%d "
                       "em_rd=%d ex_rd=%d priv=%d\n",
                       (unsigned long long)cycle, dup_count,
                       pre_id_ex_pc, pre_if_id_pc, pre_pc_reg,
                       pre_data_stall, pre_mmu_stall, pflush, div_stall, pre_trap_taken,
                       ex_mem_read, id_ex_read, pre_priv);
            }
            if (dup_count == 30)
                fprintf(stderr, "  (suppressing further DUP_PC logs)\n");
        }

        top->clk = 1;
        top->eval();

        // ---- OOB RAM access detection ----
        // RAM has RAM_WORDS entries but address width covers more.
        // With the padded array this no longer corrupts memory, but we log it.
        {
            uint32_t bus_addr = top->rootp->soc__DOT__mem_addr;
            bool is_ram_range = ((bus_addr >> 28) == 0x8);
            bool beyond_ram = is_ram_range && ((bus_addr & 0x0FFFFFFF) >= RAM_SIZE);
            uint8_t wstrb = top->rootp->soc__DOT__mem_wstrb;
            if (beyond_ram && wstrb) {
                static uint64_t oob_write_count = 0;
                oob_write_count++;
                if (oob_write_count <= 20 || (oob_write_count % 10000 == 0)) {
                    fprintf(stderr, "[%llu] OOB_WRITE #%llu addr=0x%08x ws=0x%x wd=0x%08x pc=0x%08x ptw=%d\n",
                           (unsigned long long)cycle, (unsigned long long)oob_write_count,
                           bus_addr, wstrb, top->rootp->soc__DOT__mem_wdata,
                           pre_em_pc, pre_ptw_req);
                }
            }
            if (beyond_ram) {
                static uint64_t oob_read_count = 0;
                oob_read_count++;
                if (oob_read_count == 1 || oob_read_count == 10 || oob_read_count == 100 ||
                    oob_read_count == 1000 || oob_read_count % 100000 == 0) {
                    fprintf(stderr, "[%llu] OOB_ACCESS #%llu addr=0x%08x pc=0x%08x ptw=%d\n",
                           (unsigned long long)cycle, (unsigned long long)oob_read_count,
                           bus_addr, pre_em_pc, pre_ptw_req);
                }
            }
        }

        // ---- Stray write detection: writes when MEM stage has bubble ----
        // If mem_wstrb != 0 and the MEM stage has no valid instruction (bubble),
        // AND it's not a PTW write, this is a spurious RAM write.
        if (pre_bus_wstrb != 0 && !pre_ptw_req && pre_em_pc == 0 && cycle > 1000) {
            static int stray_count = 0;
            stray_count++;
            if (stray_count <= 50) {
                fprintf(stderr, "[%llu] STRAY_WRITE #%d addr=0x%08x wstrb=0x%x wdata=0x%08x "
                       "em_pc=0x%08x id_ex=0x%08x if_id=0x%08x pc=0x%08x "
                       "priv=%d stall=%d mmu=%d trap=%d\n",
                       (unsigned long long)cycle, stray_count,
                       pre_bus_addr, pre_bus_wstrb, pre_bus_wdata,
                       pre_em_pc, pre_id_ex_pc, pre_if_id_pc, pre_pc_reg,
                       pre_priv, pre_data_stall, pre_mmu_stall, pre_trap_taken);
            }
        }

        // ---- Interrupt PC validation ----
        // When an interrupt is taken, verify sepc/mepc matches the expected value
        if (pre_trap_taken && (pre_trap_cause & 0x80000000)) {
            // This is an interrupt. Check that sepc/mepc (post-posedge) matches
            // the interrupt_pc computed from pipeline state
            uint32_t expected_pc =
                (pre_id_ex_pc != 0) ? pre_id_ex_pc :
                (pre_if_id_pc != 0) ? pre_if_id_pc :
                pre_pc_reg;

            // Read the actual sepc/mepc AFTER posedge
            uint32_t actual_sepc = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__sepc;
            uint32_t actual_mepc = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mepc;

            // Use pre-posedge trap_to_s_mode to determine which epc was written
            uint32_t actual_epc = pre_trap_to_s ? actual_sepc : actual_mepc;

            static int irq_check_count = 0;
            irq_check_count++;
            if (actual_epc != expected_pc) {
                fprintf(stderr, "[%llu] IRQ_PC_MISMATCH #%d! cause=0x%08x "
                       "expected=0x%08x actual_%cepc=0x%08x "
                       "id_ex=0x%08x if_id=0x%08x pc_reg=0x%08x em_pc=0x%08x "
                       "priv=%d stall=%d mmu=%d\n",
                       (unsigned long long)cycle, irq_check_count,
                       pre_trap_cause, expected_pc,
                       pre_trap_to_s ? 's' : 'm', actual_epc,
                       pre_id_ex_pc, pre_if_id_pc, pre_pc_reg, pre_em_pc,
                       pre_priv, pre_data_stall, pre_mmu_stall);
            } else if (irq_check_count <= 10) {
                fprintf(stderr, "[%llu] IRQ_PC_OK #%d cause=0x%08x epc=0x%08x "
                       "id_ex=0x%08x if_id=0x%08x pc_reg=0x%08x em_pc=0x%08x priv=%d\n",
                       (unsigned long long)cycle, irq_check_count,
                       pre_trap_cause, actual_epc,
                       pre_id_ex_pc, pre_if_id_pc, pre_pc_reg, pre_em_pc, pre_priv);
            }

            // CRITICAL: Check if MEM stage has a CSR write that will flow to WB
            // and potentially overwrite trap entry CSR modifications
            if (pre_ex_mem_csr_write) {
                static int csr_irq_conflict = 0;
                csr_irq_conflict++;
                fprintf(stderr, "[%llu] CSR_IRQ_CONFLICT #%d! IRQ cause=0x%08x "
                       "MEM csr_addr=0x%03x csr_wdata=0x%08x em_pc=0x%08x priv=%d\n",
                       (unsigned long long)cycle, csr_irq_conflict,
                       pre_trap_cause, pre_ex_mem_csr_addr, pre_ex_mem_csr_wdata,
                       pre_em_pc, pre_priv);
            }
        }

        // ---- Post-interrupt sstatus SIE check ----
        // After an S-mode trap entry, sstatus.SIE should be 0 (interrupts disabled)
        // If it's 1, something overwrote sstatus after the trap entry
        {
            static bool prev_was_s_irq = false;
            static uint32_t s_irq_cycle = 0;
            if (prev_was_s_irq) {
                uint32_t sstatus_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mstatus;
                uint8_t sie_bit = (sstatus_val >> 1) & 1;  // SIE is bit 1 of mstatus
                uint8_t wb_csr_write = top->rootp->soc__DOT__cpu_inst__DOT__mem_wb_csr_write;
                uint32_t wb_csr_addr = top->rootp->soc__DOT__cpu_inst__DOT__mem_wb_csr_addr;
                if (sie_bit) {
                    static int sie_violations = 0;
                    sie_violations++;
                    if (sie_violations <= 30) {
                        fprintf(stderr, "[%llu] SIE_VIOLATION #%d! sstatus.SIE=1 after S-mode interrupt "
                               "(irq at cycle %u) mstatus=0x%08x wb_csr_wr=%d wb_csr_addr=0x%03x\n",
                               (unsigned long long)cycle, sie_violations,
                               s_irq_cycle, sstatus_val, wb_csr_write, wb_csr_addr);
                    }
                }
                prev_was_s_irq = false;
            }
            if (pre_trap_taken && (pre_trap_cause & 0x80000000) && pre_trap_to_s) {
                prev_was_s_irq = true;
                s_irq_cycle = (uint32_t)cycle;
            }
        }

        // ---- Sentinel RAM check ----
        // First word of OpenSBI should never change after initialization
        {
            static uint32_t sentinel_value = 0;
            uint32_t current = RAM(0);
            if (cycle == 1000) sentinel_value = current;
            if (sentinel_value != 0 && current != sentinel_value && cycle > 1000) {
                fprintf(stderr, "[%llu] SENTINEL_CORRUPT! RAM[0] was 0x%08x now 0x%08x\n",
                       (unsigned long long)cycle, sentinel_value, current);
                sentinel_value = current; // only report once
            }
        }

        // ---- Read internal signals ----
        uint32_t pc   = top->rootp->soc__DOT__cpu_inst__DOT__pc_reg;
        uint8_t  priv = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__priv_level;
        uint32_t satp = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__satp;

        uint8_t  trap_taken = top->rootp->soc__DOT__cpu_inst__DOT__take_trap_effective;
        uint32_t trap_cause = top->rootp->soc__DOT__cpu_inst__DOT__trap_cause_final;

        uint8_t  uart_valid = top->rootp->soc__DOT__uart_inst__DOT__sim_tx_valid;
        uint8_t  uart_data  = top->rootp->soc__DOT__uart_inst__DOT__sim_tx_data;

        // (progress report is at the end of the loop)

        // ==== FULL U-MODE TRACE ====
        // Log every non-stall cycle in U-mode. U-mode runs ~23K cycles total,
        // so this is lightweight. Shows MEM-stage operation (about to go to WB)
        // and the WB commit happening THIS posedge (writing to register file).
        // Register values are POST-posedge (reflect this cycle's WB commit).
        if (pre_priv == 0 && !pre_mmu_stall && cycle > 100) {
            auto& rf_ut = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile;
            const char *op = pre_ex_mem_mr ? "LD" : (pre_ex_mem_mem_write ? "ST" : "--");
            fprintf(stderr, "UT[%llu] MEM:0x%08x %s x%d VA=0x%08x PA=0x%08x rd=0x%08x "
                   "| WB:x%d%s0x%08x "
                   "| x12=%08x x14=%08x x16=%08x sp=%08x s0=%08x trap=%d\n",
                   (unsigned long long)cycle, pre_em_pc, op, pre_ex_mem_rd_v,
                   pre_alu_result, pre_data_paddr_v, pre_rdata_v,
                   pre_wb_rd_v, pre_wb_rw_v ? "<-" : "  ", pre_wb_data_v,
                   rf_ut[12], rf_ut[14], rf_ut[16], rf_ut[2], rf_ut[8],
                   pre_trap_taken);
        }
        // Also log stalled U-mode cycles (marks them, but less verbose)
        if (pre_priv == 0 && pre_mmu_stall && cycle > 100) {
            static int umode_stall_count = 0;
            umode_stall_count++;
            if (umode_stall_count <= 200) {
                fprintf(stderr, "UT[%llu] STALL em=0x%08x bus=0x%08x PA=0x%08x ptw=%d\n",
                       (unsigned long long)cycle, pre_em_pc, pre_bus_addr,
                       pre_data_paddr_v, pre_ptw_req);
            }
        }

        // ==== U-MODE FLUSH CAUSE LOGGER ====
        if (pre_priv == 0 && pre_pflush && cycle > 100) {
            fprintf(stderr, "UFLUSH[%llu] em_pc=0x%08x trap=%d take_trap=%d can_int=%d exc=%d "
                   "br=%d btype=%d jal=%d jalr=%d mret=%d sret=%d sfence=%d "
                   "mmu=%d next_pc=0x%08x tgt=0x%08x pcpls=0x%08x\n",
                   (unsigned long long)cycle, pre_em_pc,
                   pre_trap_taken, pre_take_trap, pre_can_int, pre_exception,
                   pre_em_branch, pre_em_btype, pre_em_jal, pre_em_jalr,
                   pre_em_mret, pre_em_sret, pre_em_sfence,
                   pre_mmu_stall, pre_next_pc, pre_em_btgt, pre_em_pcpls);
        }

        // ==== CANARY VALUE WATCHPOINTS ====
        // Track when canary 0x8474df3e is stored in U-mode
        if (pre_priv == 0 && pre_bus_wstrb != 0 && !pre_ptw_req &&
            pre_bus_wdata == 0x8474df3e) {
            fprintf(stderr, "[%llu] *** CANARY_STORE PA=0x%08x pc=0x%08x ws=0x%x ***\n",
                   (unsigned long long)cycle, pre_bus_addr, pre_em_pc, pre_bus_wstrb);
        }
        // Track when x12 becomes 0x95b7a000 (the wrong canary value)
        {
            auto& rf_cw = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile;
            static uint32_t prev_x12_cw = 0;
            if (rf_cw[12] != prev_x12_cw) {
                if (rf_cw[12] == 0x95b7a000 && pre_priv == 0) {
                    fprintf(stderr, "[%llu] *** X12_WRONG_CANARY! 0x%08x -> 0x95b7a000 "
                           "wb:x%d%s0x%08x em_pc=0x%08x ***\n",
                           (unsigned long long)cycle, prev_x12_cw,
                           pre_wb_rd_v, pre_wb_rw_v ? "<-" : "  ",
                           pre_wb_data_v, pre_em_pc);
                }
                prev_x12_cw = rf_cw[12];
            }
        }

        // ---- PTE injection for identity-mapped MMIO ----
        if (satp != prev_satp && (satp & 0x80000000)) {
            uint32_t pt_base_word = (((satp & 0x003FFFFF) << 12) - RAM_BASE) >> 2;

            fprintf(stderr, "[%llu] SATP=0x%08x pt_base_word=0x%x — MMIO PTEs injected\n",
                    (unsigned long long)cycle, satp, pt_base_word);

            // MMIO identity mapping only (matching iverilog testbench)
            RAM(pt_base_word + 0x040) = 0x040000C7;  // UART
            RAM(pt_base_word + 0x008) = 0x008000C7;  // CLINT
            RAM(pt_base_word + 0x030) = 0x030000C7;  // PLIC

            // Validate kernel linear mapping PTEs
            // VA 0xC0000000+ maps to PA 0x80400000+ (PPN[1] = VPN[1] - 0xFF)
            // Kernel starts at PA 0x80400000 (skips OpenSBI at 0x80000000-0x803FFFFF)
            for (int vpn1 = 0x300; vpn1 <= 0x309; vpn1++) {
                uint32_t pte = RAM(pt_base_word + vpn1);
                if (!(pte & 1)) continue;  // skip invalid PTEs
                uint32_t actual_ppn1 = pte >> 20;
                uint32_t expected_ppn1 = vpn1 - 0xFF;
                uint32_t flags = pte & 0x3FF;
                if ((flags & 0xE) != 0 && actual_ppn1 != expected_ppn1) {
                    fprintf(stderr, "  *** PTE ERROR: L1[0x%x] = 0x%08x PPN[1]=0x%x expected=0x%x ***\n",
                           vpn1, pte, actual_ppn1, expected_ppn1);
                }
            }
            prev_satp = satp;
        } else if (satp != prev_satp) {
            prev_satp = satp;
        }

        // ---- S-mode entry ----
        if (priv == 1 && !entered_smode) {
            entered_smode = true;
            uint32_t mie_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mie;
            fprintf(stderr, "[%llu] ENTERED S-MODE pc=0x%08x mie=0x%x\n",
                   (unsigned long long)cycle, pc, mie_val);
        }

        // ---- Kernel virtual address space ----
        if (!saw_kernel_virt && pc >= 0xC0000000) {
            saw_kernel_virt = true;
            fprintf(stderr, "[%llu] KERNEL VIRTUAL pc=0x%08x (MMU active)\n",
                   (unsigned long long)cycle, pc);
        }

        // ---- Kernel function tracing ----
        if (priv == 1 && saw_kernel_virt) {
            // Key function entry tracing (one-shot)
            struct FnEntry { uint32_t addr; const char* name; int count; int max; };
            static FnEntry fn_trace[] = {
                {0xC040070A, "start_kernel", 0, 1},
                {0xC029066E, "__warn", 0, 3},
                {0xC02903EE, "panic", 0, 5},
                // 8250 serial driver
                {0xC01F5D78, "serial8250_config_port", 0, 3},
                {0xC01F7C66, "serial8250_startup", 0, 3},
                {0xC01F55A0, "wait_for_lsr", 0, 5},
                {0xC01F5608, "wait_for_xmitr", 0, 5},
                {0xC01F4A1A, "serial8250_probe", 0, 3},
                {0xC01F3448, "uart_add_one_port", 0, 3},
                // udelay — trace first 10 entries
                {0xC028FB52, "udelay", 0, 10},
                // initramfs
                {0xC04028DE, "do_populate_rootfs", 0, 2},
                {0xC0402914, "do_populate_rootfs_panic", 0, 2},  // +0x36: call to panic
                {0xC0402144, "unpack_to_rootfs", 0, 3},
                {0xC041B736, "gunzip", 0, 3},
                {0xC041B4A8, "__gunzip", 0, 3},
                {0xC0401A04, "initramfs_error", 0, 5},
                {0xC04019AE, "initrd_load", 0, 2},
                // Timer setup
                {0xC02644D0, "riscv_timer_starting_cpu", 0, 3},
                {0xC026454E, "riscv_clock_next_event", 0, 5},
                {0xC0005698, "sbi_set_timer", 0, 5},
                {0xC000515A, "__sbi_set_timer_v01", 0, 5},
                {0xC0005176, "__sbi_set_timer_v02", 0, 5},
                {0xC0418544, "riscv_timer_init_dt", 0, 2},
                {0xC0264488, "riscv_timer_interrupt", 0, 5},
            };
            for (auto& fn : fn_trace) {
                if (pc == fn.addr && fn.count < fn.max) {
                    fn.count++;
                    auto& rf = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile;
                    fprintf(stderr, "[%llu] FUNC %s (#%d) ra=0x%08x sp=0x%08x a0=0x%08x a1=0x%08x s0=0x%08x s1=0x%08x\n",
                           (unsigned long long)cycle, fn.name, fn.count,
                           rf[1], rf[2], rf[10], rf[11], rf[8], rf[9]);
                    // For panic, dump full register state + memory at string pointers
                    if (fn.addr == 0xC02903EE) {
                        fprintf(stderr, "  a2=0x%08x a3=0x%08x a4=0x%08x a5=0x%08x a6=0x%08x a7=0x%08x\n",
                               rf[12], rf[13], rf[14], rf[15], rf[16], rf[17]);
                        fprintf(stderr, "  s2=0x%08x s3=0x%08x s4=0x%08x s5=0x%08x s6=0x%08x s7=0x%08x\n",
                               rf[18], rf[19], rf[20], rf[21], rf[22], rf[23]);
                        // Dump stack (saved RA/FP)
                        uint32_t sp = rf[2];
                        if (sp >= RAM_BASE && sp < RAM_BASE + RAM_SIZE) {
                            uint32_t sp_word = (sp - RAM_BASE) >> 2;
                            fprintf(stderr, "  Stack:");
                            for (int j = 0; j < 16; j++)
                                fprintf(stderr, " %08x", RAM(sp_word + j));
                            fprintf(stderr, "\n");
                        }
                        // Dump memory at a0 (format string) and a1 (first arg / error string)
                        // Kernel VA 0xC0xxxxxx -> PA 0x80xxxxxx (linear mapping, offset 0x40000000)
                        for (int ai = 0; ai <= 1; ai++) {
                            uint32_t va = rf[10 + ai];  // a0, a1
                            fprintf(stderr, "  a%d string dump (VA=0x%08x): ", ai, va);
                            if (va >= 0xC0000000 && va < 0xC2400000) {
                                uint32_t pa = va - 0x3FC00000;  // kernel: VA 0xC0000000 → PA 0x80400000
                                uint32_t byte_off = pa - RAM_BASE;
                                fprintf(stderr, "PA=0x%08x bytes: ", pa);
                                for (int b = 0; b < 64; b++) {
                                    uint32_t word_idx = (byte_off + b) >> 2;
                                    uint32_t byte_pos = (byte_off + b) & 3;
                                    if (word_idx < RAM_WORDS) {
                                        uint8_t byte_val = (RAM(word_idx) >> (byte_pos * 8)) & 0xFF;
                                        fprintf(stderr, "%02x ", byte_val);
                                    }
                                }
                                fprintf(stderr, "\n  a%d as text: \"", ai);
                                for (int b = 0; b < 64; b++) {
                                    uint32_t word_idx = (byte_off + b) >> 2;
                                    uint32_t byte_pos = (byte_off + b) & 3;
                                    if (word_idx < RAM_WORDS) {
                                        uint8_t byte_val = (RAM(word_idx) >> (byte_pos * 8)) & 0xFF;
                                        if (byte_val == 0) break;
                                        fprintf(stderr, "%c", (byte_val >= 32 && byte_val < 127) ? byte_val : '.');
                                    }
                                }
                                fprintf(stderr, "\"\n");
                            } else {
                                fprintf(stderr, "(not in kernel linear map)\n");
                            }
                        }
                    }
                    // For do_populate_rootfs_panic, dump error string from a1
                    if (fn.addr == 0xC0402914) {
                        fprintf(stderr, "  s2=0x%08x s3=0x%08x s4=0x%08x s5=0x%08x\n",
                               rf[18], rf[19], rf[20], rf[21]);
                        for (int ai = 0; ai <= 1; ai++) {
                            uint32_t va = rf[10 + ai];
                            fprintf(stderr, "  a%d string (VA=0x%08x): ", ai, va);
                            if (va >= 0xC0000000 && va < 0xC2400000) {
                                uint32_t pa = va - 0x3FC00000;  // kernel: VA 0xC0000000 → PA 0x80400000
                                uint32_t byte_off = pa - RAM_BASE;
                                for (int b = 0; b < 80; b++) {
                                    uint32_t word_idx = (byte_off + b) >> 2;
                                    uint32_t byte_pos = (byte_off + b) & 3;
                                    if (word_idx < RAM_WORDS) {
                                        uint8_t bv = (RAM(word_idx) >> (byte_pos * 8)) & 0xFF;
                                        if (bv == 0) break;
                                        fprintf(stderr, "%c", (bv >= 32 && bv < 127) ? bv : '.');
                                    }
                                }
                                fprintf(stderr, "\n");
                            } else {
                                fprintf(stderr, "(not in kernel linear map)\n");
                            }
                        }
                    }
                    // For udelay, also dump stack to find caller's caller
                    if (fn.addr == 0xC028FB52) {
                        uint32_t sp = rf[2];
                        if (sp >= RAM_BASE && sp < RAM_BASE + RAM_SIZE) {
                            uint32_t sp_word = (sp - RAM_BASE) >> 2;
                            fprintf(stderr, "  Stack:");
                            for (int j = 0; j < 8; j++)
                                fprintf(stderr, " %08x", RAM(sp_word + j));
                            fprintf(stderr, "\n");
                        }
                    }
                }
            }
        }

        // ---- UART register access trace (for 8250 driver debugging) ----
        // Track reads/writes to UART registers at PA 0x10000000-0x1000001F
        {
            uint32_t bus_addr = top->rootp->soc__DOT__mem_addr;
            uint8_t  bus_wstrb = top->rootp->soc__DOT__mem_wstrb;
            uint8_t  mem_read = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_mem_read;
            static int uart_access_count = 0;
            if (bus_addr >= 0x10000000 && bus_addr < 0x10000020 && (mem_read || bus_wstrb) && priv == 1) {
                uart_access_count++;
                // Log first 10 S-mode UART accesses, then every 5000th
                if (uart_access_count <= 10 || uart_access_count % 5000 == 0) {
                    uint32_t rdata = top->rootp->soc__DOT__mem_rdata;
                    uint32_t ex_mem_pc_val = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_pc;
                    uint8_t reg_off = (bus_addr - 0x10000000) >> 2;  // register index
                    const char* regnames[] = {"THR/RBR","IER","IIR/FCR","LCR","MCR","LSR","MSR","SCR"};
                    const char* rname = (reg_off < 8) ? regnames[reg_off] : "???";
                    fprintf(stderr, "[%llu] UART_%s %s[%d] %s=0x%02x pc=0x%08x (#%d)\n",
                           (unsigned long long)cycle, bus_wstrb ? "WR" : "RD",
                           rname, reg_off,
                           bus_wstrb ? "wdata" : "rdata",
                           bus_wstrb ? (top->rootp->soc__DOT__mem_wdata & 0xFF) : (rdata & 0xFF),
                           ex_mem_pc_val, uart_access_count);
                }
            }
        }

        // ---- Interrupt & trap tracking ----
        {
            static uint32_t prev_pc = 0;
            static uint32_t prev_scause = 0, prev_mcause = 0;
            static uint32_t prev_sepc = 0, prev_mepc = 0;
            static int strap_count = 0, mtrap_count = 0;
            static int s_irq_count = 0;

            uint32_t sepc = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__sepc;
            uint32_t scause = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__scause;
            uint32_t stval = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__stval;
            uint32_t mepc = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mepc;
            uint32_t mcause = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mcause;
            uint32_t mtval = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mtval;
            uint32_t stvec_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__stvec;

            // Detailed interrupt tracking using PRE-POSEDGE state (captures
            // the actual state the always @(posedge clk) block processed)
            if (pre_trap_taken) {
                uint32_t trap_cause_val = pre_trap_cause;
                bool is_irq = (trap_cause_val & 0x80000000) != 0;

                // Log S-mode interrupts (first 10 and every 500th)
                if (is_irq && pre_priv == 1) {
                    s_irq_count++;
                    if (s_irq_count <= 10 || s_irq_count % 500 == 0) {
                        fprintf(stderr, "[%llu] S_IRQ #%d cause=0x%08x sepc=0x%08x "
                               "id_ex_pc=0x%08x if_id_pc=0x%08x pc_reg=0x%08x em_pc=0x%08x "
                               "stall=%d mmu=%d\n",
                               (unsigned long long)cycle, s_irq_count,
                               trap_cause_val, sepc,
                               pre_id_ex_pc, pre_if_id_pc, pre_pc_reg, pre_em_pc,
                               pre_data_stall, pre_mmu_stall);
                    }
                }

                // Log ALL M-mode interrupts with pipeline state
                if (is_irq && pre_priv != 1) {
                    static int m_irq_count = 0;
                    m_irq_count++;
                    if (m_irq_count <= 5) {
                        fprintf(stderr, "[%llu] M_IRQ #%d cause=0x%08x mepc=0x%08x "
                               "id_ex_pc=0x%08x if_id_pc=0x%08x pc_reg=0x%08x em_pc=0x%08x "
                               "priv=%d stall=%d mmu=%d\n",
                               (unsigned long long)cycle, m_irq_count,
                               trap_cause_val, mepc,
                               pre_id_ex_pc, pre_if_id_pc, pre_pc_reg, pre_em_pc,
                               pre_priv, pre_data_stall, pre_mmu_stall);
                    }
                }

                // Log ALL traps near potential crash (page faults)
                if (!is_irq && (trap_cause_val == 13 || trap_cause_val == 15)) {
                    uint32_t fault_addr = top->rootp->soc__DOT__cpu_inst__DOT__mmu_fault_vaddr;
                    fprintf(stderr, "[%llu] TRAP cause=%d pc=0x%08x priv=%d "
                           "fault_va=0x%08x em_pc=0x%08x id_ex=0x%08x if_id=0x%08x\n",
                           (unsigned long long)cycle, trap_cause_val, pre_pc_reg, pre_priv,
                           fault_addr, pre_em_pc, pre_id_ex_pc, pre_if_id_pc);
                    if (fault_addr == 0x200 || fault_addr < 0x1000) {
                        auto& rf = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile;
                        fprintf(stderr, "  *** LOW ADDR FAULT! Dumping regs ***\n");
                        for (int r = 0; r < 32; r += 4)
                            fprintf(stderr, "  x%d=0x%08x x%d=0x%08x x%d=0x%08x x%d=0x%08x\n",
                                   r, rf[r], r+1, rf[r+1], r+2, rf[r+2], r+3, rf[r+3]);
                    }
                }
            }

            // Detect S-mode trap: scause changes
            if (scause != prev_scause) {
                strap_count++;
                if (strap_count <= 20) {
                    fprintf(stderr, "[%llu] SCAUSE_CHG #%d: 0x%08x -> 0x%08x sepc=0x%08x stval=0x%08x stvec=0x%08x\n",
                           (unsigned long long)cycle, strap_count,
                           prev_scause, scause, sepc, stval, stvec_val);
                }
                prev_scause = scause;
                prev_sepc = sepc;
            } else if (sepc != prev_sepc) {
                prev_sepc = sepc;
            }

            // Detect M-mode trap: mcause changes
            if (mcause != prev_mcause) {
                mtrap_count++;
                if (mtrap_count <= 20) {
                    fprintf(stderr, "[%llu] MCAUSE_CHG #%d: 0x%08x -> 0x%08x mepc=0x%08x mtval=0x%08x\n",
                           (unsigned long long)cycle, mtrap_count,
                           prev_mcause, mcause, mepc, mtval);
                }
                prev_mcause = mcause;
                prev_mepc = mepc;
            }
            prev_pc = pc;
        }

        // ---- Trace desc_read_finalized_seq function execution ----
        // Function: VA 0xC00352F8 - 0xC00353E4 (0xEC bytes)
        // Also trace loads from any VA < 0x1000 (clearly invalid)
        {
            uint32_t ex_mem_pc_val = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_pc;
            uint32_t bus_addr_val = top->rootp->soc__DOT__mem_addr;
            uint8_t bus_wstrb_val = top->rootp->soc__DOT__mem_wstrb;
            uint8_t mem_read_val = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_mem_read;
            uint32_t rdata_val = top->rootp->soc__DOT__mem_rdata;
            uint32_t data_paddr = top->rootp->soc__DOT__cpu_inst__DOT__data_paddr;
            uint8_t mmu_stall = top->rootp->soc__DOT__cpu_inst__DOT__mmu_stall;
            uint8_t pgf_data = top->rootp->soc__DOT__cpu_inst__DOT__page_fault_data;

            // Log when MEM-stage PC is in desc_read_finalized_seq — only on fault/low addr
            static int desc_trace_count = 0;
            if (ex_mem_pc_val >= 0xC00352F8 && ex_mem_pc_val <= 0xC00353E4 && priv == 1) {
                desc_trace_count++;
                // Only log if page fault or accessing low addresses (< 0x10000)
                if (desc_trace_count <= 5 || (pgf_data && desc_trace_count <= 20) ||
                    (bus_addr_val < 0x10000 && bus_addr_val > 0)) {
                    fprintf(stderr, "[%llu] DESC_FN em_pc=0x%08x bus=0x%08x %s%s paddr=0x%08x rdata=0x%08x pgf=%d stall=%d\n",
                           (unsigned long long)cycle, ex_mem_pc_val, bus_addr_val,
                           mem_read_val ? "RD" : "", bus_wstrb_val ? "WR" : "",
                           data_paddr, rdata_val, pgf_data, mmu_stall);
                }
            }

            // Log ANY load from low VA (< 0x10000) — should always fault
            if (mem_read_val && bus_addr_val < 0x10000 && bus_addr_val > 0 && priv == 1) {
                fprintf(stderr, "[%llu] LOW_ADDR_LOAD bus=0x%08x paddr=0x%08x rdata=0x%08x em_pc=0x%08x pgf=%d stall=%d\n",
                       (unsigned long long)cycle, bus_addr_val, data_paddr, rdata_val, ex_mem_pc_val, pgf_data, mmu_stall);
            }

            // Deferred U-mode fault report — read CSR values one cycle after trap
            // (NBA semantics: CSRs updated at next cycle)
            {
                static bool pending_umode_fault = false;
                static uint64_t fault_cycle = 0;
                static uint32_t fault_pc = 0;
                static uint32_t fault_cause_saved = 0;

                if (pending_umode_fault) {
                    pending_umode_fault = false;
                    uint32_t new_sepc = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__sepc;
                    uint32_t new_stval = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__stval;
                    uint32_t new_scause = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__scause;
                    fprintf(stderr, "[%llu+1] U-MODE FAULT (deferred) cause=%u sepc=0x%08x stval=0x%08x scause=0x%08x\n",
                           (unsigned long long)fault_cycle, fault_cause_saved, new_sepc, new_stval, new_scause);
                    // Try to read the faulting instruction by walking page table
                    {
                        uint32_t satp_now = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__satp;
                        uint32_t pt_base_pa = (satp_now & 0x003FFFFF) << 12;
                        uint32_t vpn1 = (new_sepc >> 22) & 0x3FF;
                        uint32_t vpn0 = (new_sepc >> 12) & 0x3FF;
                        uint32_t offset = new_sepc & 0xFFF;
                        // Walk L1
                        uint32_t l1_word = (pt_base_pa + vpn1 * 4 - RAM_BASE) >> 2;
                        uint32_t pte1 = (l1_word < RAM_WORDS) ? RAM(l1_word) : 0;
                        uint32_t pa = 0;
                        bool found = false;
                        if (pte1 & 1) {
                            if (pte1 & 0xE) { // Superpage
                                pa = ((pte1 >> 10) << 12) | (vpn0 << 12) | offset;
                                found = true;
                            } else { // L2 page table
                                uint32_t l2_base_pa = (pte1 >> 10) << 12;
                                uint32_t l2_word = (l2_base_pa + vpn0 * 4 - RAM_BASE) >> 2;
                                uint32_t pte0 = (l2_word < RAM_WORDS) ? RAM(l2_word) : 0;
                                if (pte0 & 1) {
                                    pa = ((pte0 >> 10) << 12) | offset;
                                    found = true;
                                }
                            }
                        }
                        if (found && pa >= RAM_BASE && pa < RAM_BASE + RAM_SIZE) {
                            uint32_t byte_off = pa - RAM_BASE;
                            uint32_t w0 = RAM(byte_off >> 2);
                            uint32_t w1 = RAM((byte_off >> 2) + 1);
                            uint32_t shift = (byte_off & 3) * 8;
                            uint32_t insn = (shift == 0) ? w0 : (w0 >> shift) | (w1 << (32 - shift));
                            fprintf(stderr, "  insn at sepc VA 0x%08x -> PA 0x%08x: 0x%08x (halfwords: %04x %04x)\n",
                                   new_sepc, pa, insn, insn & 0xFFFF, (insn >> 16) & 0xFFFF);
                            // Try to decode basic store instructions
                            uint16_t lo = insn & 0xFFFF;
                            if ((lo & 0x3) == 0x3) {
                                // 32-bit instruction
                                uint32_t opcode = insn & 0x7F;
                                uint32_t rs1 = (insn >> 15) & 0x1F;
                                uint32_t rs2 = (insn >> 20) & 0x1F;
                                int32_t imm_s = ((int32_t)(insn & 0xFE000000) >> 20) | ((insn >> 7) & 0x1F);
                                if (opcode == 0x23) { // STORE
                                    uint32_t funct3 = (insn >> 12) & 0x7;
                                    const char* snames[] = {"sb","sh","sw","??","??","??","??","??"};
                                    auto& rf2 = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile;
                                    fprintf(stderr, "  DECODED: %s x%d, %d(x%d)  base=0x%08x+%d=0x%08x\n",
                                           snames[funct3], rs2, imm_s, rs1,
                                           rf2[rs1], imm_s, rf2[rs1] + imm_s);
                                }
                            } else {
                                // 16-bit compressed instruction
                                uint32_t c_op = lo & 0x3;
                                uint32_t c_funct3 = (lo >> 13) & 0x7;
                                if (c_op == 0 && (c_funct3 == 5 || c_funct3 == 6 || c_funct3 == 7)) {
                                    fprintf(stderr, "  DECODED: C.SW/C.FSW/C.SD variant\n");
                                } else if (c_op == 2 && (c_funct3 == 5 || c_funct3 == 6 || c_funct3 == 7)) {
                                    fprintf(stderr, "  DECODED: C.SWSP/C.FSWSP/C.SDSP variant\n");
                                }
                            }
                        } else {
                            fprintf(stderr, "  insn at sepc: VA 0x%08x not translatable (pte1=0x%08x, pa=0x%08x)\n",
                                   new_sepc, pte1, pa);
                        }
                    }
                    auto& rf = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile;
                    fprintf(stderr, "  regs: ra=0x%08x sp=0x%08x gp=0x%08x tp=0x%08x\n",
                           rf[1], rf[2], rf[3], rf[4]);
                    fprintf(stderr, "  regs: t0=0x%08x t1=0x%08x t2=0x%08x s0=0x%08x\n",
                           rf[5], rf[6], rf[7], rf[8]);
                    fprintf(stderr, "  regs: s1=0x%08x a0=0x%08x a1=0x%08x a2=0x%08x\n",
                           rf[9], rf[10], rf[11], rf[12]);
                    fprintf(stderr, "  regs: a3=0x%08x a4=0x%08x a5=0x%08x a6=0x%08x a7=0x%08x\n",
                           rf[13], rf[14], rf[15], rf[16], rf[17]);

                    // ---- Direct RAM canary comparison ----
                    // If the crash is at sepc=0x95b21568 (sb x0,0(x0)), check canary
                    // values directly in RAM to distinguish memory corruption from load bug.
                    // Walk page tables for the two canary addresses:
                    //   VA 0x95b7a000 = __stack_chk_guard
                    //   VA 0x95b776f8 = saved canary
                    {
                        uint32_t canary_vas[2] = { 0x95b7a000, 0x95b776f8 };
                        uint32_t canary_pas[2] = { 0, 0 };
                        const char* canary_labels[2] = { "__stack_chk_guard", "saved_canary" };

                        uint32_t satp2 = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__satp;
                        uint32_t pt2_base = (satp2 & 0x003FFFFF) << 12;

                        fprintf(stderr, "  === DIRECT RAM CANARY CHECK ===\n");
                        for (int ci = 0; ci < 2; ci++) {
                            uint32_t va = canary_vas[ci];
                            uint32_t v1 = (va >> 22) & 0x3FF;
                            uint32_t v0 = (va >> 12) & 0x3FF;
                            uint32_t pg_off = va & 0xFFF;
                            uint32_t l1w = (pt2_base + v1 * 4 - RAM_BASE) >> 2;
                            uint32_t p1 = (l1w < RAM_WORDS) ? RAM(l1w) : 0;
                            uint32_t cpa = 0;
                            bool cfound = false;
                            if (p1 & 1) {
                                if (p1 & 0xE) { // superpage
                                    cpa = ((p1 >> 20) << 22) | (v0 << 12) | pg_off;
                                    cfound = true;
                                } else {
                                    uint32_t l2b = (p1 >> 10) << 12;
                                    uint32_t l2w = (l2b + v0 * 4 - RAM_BASE) >> 2;
                                    uint32_t p0 = (l2w < RAM_WORDS) ? RAM(l2w) : 0;
                                    if (p0 & 1) {
                                        cpa = ((p0 >> 10) << 12) | pg_off;
                                        cfound = true;
                                    }
                                }
                            }
                            canary_pas[ci] = cpa;
                            if (cfound && cpa >= RAM_BASE && cpa < RAM_BASE + RAM_SIZE) {
                                uint32_t cw0 = RAM((cpa - RAM_BASE) >> 2);
                                uint32_t cw1 = RAM(((cpa - RAM_BASE) >> 2) + 1);
                                fprintf(stderr, "  %s VA 0x%08x -> PA 0x%08x: "
                                       "word[0]=0x%08x word[1]=0x%08x\n",
                                       canary_labels[ci], va, cpa, cw0, cw1);
                            } else {
                                fprintf(stderr, "  %s VA 0x%08x -> NOT MAPPED (pte1=0x%08x)\n",
                                       canary_labels[ci], va, p1);
                            }
                        }
                        // Compare the two canaries
                        if (canary_pas[0] && canary_pas[1]) {
                            uint32_t g0 = RAM((canary_pas[0] - RAM_BASE) >> 2);
                            uint32_t g1 = RAM(((canary_pas[0] - RAM_BASE) >> 2) + 1);
                            uint32_t s0v = RAM((canary_pas[1] - RAM_BASE) >> 2);
                            uint32_t s1v = RAM(((canary_pas[1] - RAM_BASE) >> 2) + 1);
                            if (g0 == s0v && g1 == s1v) {
                                fprintf(stderr, "  MATCH: both copies identical in RAM!\n"
                                       "  -> This means the LOAD returned wrong data (CPU bug)\n");
                            } else {
                                fprintf(stderr, "  MISMATCH: guard={0x%08x,0x%08x} saved={0x%08x,0x%08x}\n",
                                       g0, g1, s0v, s1v);
                                if (g0 == s0v)
                                    fprintf(stderr, "  -> Low words match, HIGH WORDS DIFFER (memory corruption)\n");
                                else
                                    fprintf(stderr, "  -> Low words also differ! (significant corruption)\n");
                            }
                        }
                    }
                }

                // ---- U-mode instruction trace buffer ----
                // Track WB-stage commits in user mode to diagnose crashes
                {
                    struct UTraceEntry {
                        uint64_t cycle;
                        uint32_t wb_pc;     // MEM/WB PC (the committing instruction)
                        uint32_t wb_rd;     // destination register
                        uint32_t wb_data;   // data written to rd
                        uint32_t if_pc;     // current fetch PC
                        uint32_t regs[8];   // s0,s1,a0,a1,a2,a3,a4,a5
                    };
                    static UTraceEntry utrace[128];
                    static int utrace_idx = 0;
                    static bool utrace_active = false;
                    static bool utrace_dumped = false;

                    // Start tracing when we enter U-mode
                    if (priv == 0 && !trap_taken) utrace_active = true;
                    if (priv != 0) utrace_active = false;

                    if (utrace_active && !trap_taken) {
                        // Record the WB-stage instruction (it's committing this cycle)
                        auto& rf = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile;
                        uint32_t wb_pc = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_pc; // MEM-stage PC (close to commit)
                        uint32_t wb_rd = top->rootp->soc__DOT__cpu_inst__DOT__mem_wb_rd_addr;
                        uint32_t wb_rw = top->rootp->soc__DOT__cpu_inst__DOT__mem_wb_reg_write;
                        UTraceEntry& e = utrace[utrace_idx & 127];
                        e.cycle = cycle;
                        e.wb_pc = wb_pc;
                        e.wb_rd = wb_rw ? wb_rd : 0xFF;  // 0xFF = no write
                        e.wb_data = wb_rw ? (wb_rd == 0 ? 0 : rf[wb_rd]) : 0;
                        e.if_pc = pc;
                        e.regs[0] = rf[8];  // s0
                        e.regs[1] = rf[9];  // s1
                        e.regs[2] = rf[10]; // a0
                        e.regs[3] = rf[11]; // a1
                        e.regs[4] = rf[12]; // a2
                        e.regs[5] = rf[13]; // a3
                        e.regs[6] = rf[14]; // a4
                        e.regs[7] = rf[15]; // a5
                        utrace_idx++;

                        // Log U-mode stores (MEM stage has a store in progress)
                        // pre_bus_wstrb != 0 indicates a write, check it's not PTW
                        if (pre_bus_wstrb != 0 && !pre_ptw_req && !pre_mmu_stall) {
                            static int ustore_count = 0;
                            ustore_count++;
                            uint32_t store_va = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_alu_result;
                            uint32_t store_pa = top->rootp->soc__DOT__cpu_inst__DOT__data_paddr;
                            uint32_t store_data = pre_bus_wdata;
                            // Log all stores (limited since only ~23K U-mode cycles)
                            if (ustore_count <= 5000) {
                                fprintf(stderr, "[%llu] USTORE #%d pc=0x%08x VA=0x%08x PA=0x%08x data=0x%08x ws=0x%x\n",
                                       (unsigned long long)cycle, ustore_count,
                                       pre_em_pc, store_va, store_pa, store_data, pre_bus_wstrb);
                            }
                        }
                    }

                    // Dump trace on NULL deref U-mode page fault
                    if (trap_taken && priv == 0 && !utrace_dumped) {
                        uint32_t em_alu = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_alu_result;
                        uint32_t tc_eff = trap_cause & 0x1F;
                        bool is_irq = (trap_cause & 0x80000000) != 0;
                        // Dump on data page fault to VA 0 (NULL deref)
                        if (!is_irq && (tc_eff == 13 || tc_eff == 15) && em_alu == 0) {
                            utrace_dumped = true;
                            int total = (utrace_idx < 128) ? utrace_idx : 128;
                            int start = (utrace_idx < 128) ? 0 : (utrace_idx & 127);
                            fprintf(stderr, "\n=== U-MODE TRACE BUFFER (last %d instructions before NULL deref) ===\n", total);
                            for (int j = 0; j < total; j++) {
                                UTraceEntry& e = utrace[(start + j) & 127];
                                fprintf(stderr, "[%llu] wb_pc=0x%08x if_pc=0x%08x",
                                       (unsigned long long)e.cycle, e.wb_pc, e.if_pc);
                                if (e.wb_rd != 0xFF)
                                    fprintf(stderr, " x%d<-0x%08x", e.wb_rd, e.wb_data);
                                fprintf(stderr, " s0=%08x s1=%08x a0=%08x a5=%08x\n",
                                       e.regs[0], e.regs[1], e.regs[2], e.regs[7]);
                            }
                            fprintf(stderr, "=== END TRACE ===\n\n");

                            // Dump crash page to see what code is actually there
                            const uint32_t PAGE_BASE = 0x895000;
                            fprintf(stderr, "=== CRASH PAGE AT NULL DEREF (PA 0x82254000) ===\n");
                            fprintf(stderr, "  Function entry (offset 0x550-0x5A0):\n");
                            for (uint32_t off = 0x550; off <= 0x5A0; off += 4) {
                                uint32_t w = RAM(PAGE_BASE + off/4);
                                fprintf(stderr, "    [off=0x%03x] 0x%08x (%04x %04x)%s\n",
                                       off, w, w & 0xFFFF, (w >> 16) & 0xFFFF,
                                       (off == 0x568) ? " <<<CRASH" : "");
                            }
                            fprintf(stderr, "  Branch area (offset 0x5A0-0x620):\n");
                            for (uint32_t off = 0x5A0; off <= 0x620; off += 4) {
                                uint32_t w = RAM(PAGE_BASE + off/4);
                                fprintf(stderr, "    [off=0x%03x] 0x%08x (%04x %04x)%s\n",
                                       off, w, w & 0xFFFF, (w >> 16) & 0xFFFF,
                                       (off == 0x5D8) ? " <<<BRANCH" : "");
                            }
                            // Decode branch instructions in 0x5D0-0x5E0 range
                            fprintf(stderr, "  Branch decode (halfwords at 0x5D0-0x5E0):\n");
                            for (uint32_t ha = 0x5D0; ha <= 0x5E0; ha += 2) {
                                uint32_t word = RAM(PAGE_BASE + (ha & ~3u)/4);
                                uint16_t hw = (ha & 2) ? (word >> 16) : (word & 0xFFFF);
                                if ((hw & 3) == 3) {
                                    // 32-bit instruction: get next halfword
                                    uint32_t word2 = RAM(PAGE_BASE + ((ha+2) & ~3u)/4);
                                    uint16_t hw2 = ((ha+2) & 2) ? (word2 >> 16) : (word2 & 0xFFFF);
                                    uint32_t insn = hw | ((uint32_t)hw2 << 16);
                                    uint32_t opcode = insn & 0x7F;
                                    if (opcode == 0x63) { // BRANCH
                                        uint32_t funct3 = (insn >> 12) & 7;
                                        int32_t imm = (((int32_t)(insn & 0x80000000)) >> 19)
                                                    | ((insn & 0x80) << 4)
                                                    | ((insn >> 20) & 0x7E0)
                                                    | ((insn >> 7) & 0x1E);
                                        uint32_t rs1 = (insn >> 15) & 0x1F;
                                        uint32_t rs2 = (insn >> 20) & 0x1F;
                                        const char *bnames[] = {"beq","bne","??","??","blt","bge","bltu","bgeu"};
                                        fprintf(stderr, "      [0x%03x] 32b 0x%08x: %s x%d, x%d, %+d (target=0x%03x)\n",
                                               ha, insn, bnames[funct3], rs1, rs2, imm, ha + imm);
                                    } else if (opcode == 0x6F) { // JAL
                                        int32_t imm = (((int32_t)(insn & 0x80000000)) >> 11)
                                                    | (insn & 0xFF000)
                                                    | ((insn >> 9) & 0x800)
                                                    | ((insn >> 20) & 0x7FE);
                                        uint32_t rd = (insn >> 7) & 0x1F;
                                        fprintf(stderr, "      [0x%03x] 32b 0x%08x: jal x%d, %+d (target=0x%03x)\n",
                                               ha, insn, rd, imm, ha + imm);
                                    } else {
                                        fprintf(stderr, "      [0x%03x] 32b 0x%08x: opcode=0x%02x\n", ha, insn, opcode);
                                    }
                                    ha += 2; // skip second halfword (loop will add another 2)
                                } else {
                                    // 16-bit instruction
                                    uint8_t op = hw & 3;
                                    uint8_t f3 = (hw >> 13) & 7;
                                    if (op == 1 && f3 == 6) { // C.BEQZ
                                        uint8_t rs1p = (hw >> 7) & 7;
                                        int32_t off8 = ((hw & 0x1000) ? (-256) : 0)
                                                     | ((hw >> 4) & 0x60)
                                                     | ((hw << 1) & 0xC0)  // bits 6-5 from bits 5-4? wrong
                                                     | ((hw >> 7) & 0x18)
                                                     | ((hw << 3) & 0x4); // bit 2 from ... hmm complex
                                        // Easier: just extract using the known encoding
                                        // imm[8|4:3|7:6|2:1|5] from bits [12|11:10|6:5|4:3|2]
                                        int32_t boff = 0;
                                        boff |= ((hw >> 2) & 1) << 5;  // bit 2 -> imm[5]
                                        boff |= ((hw >> 3) & 3) << 1;  // bits 4:3 -> imm[2:1]
                                        boff |= ((hw >> 5) & 3) << 6;  // bits 6:5 -> imm[7:6]
                                        boff |= ((hw >> 10) & 3) << 3; // bits 11:10 -> imm[4:3]
                                        boff |= ((hw >> 12) & 1) << 8; // bit 12 -> imm[8]
                                        if (boff & 0x100) boff |= 0xFFFFFE00; // sign extend
                                        fprintf(stderr, "      [0x%03x] C.BEQZ x%d(x%d), %+d (target=0x%03x)\n",
                                               ha, 8+rs1p, rs1p, boff, ha + boff);
                                    } else if (op == 1 && f3 == 7) { // C.BNEZ
                                        uint8_t rs1p = (hw >> 7) & 7;
                                        int32_t boff = 0;
                                        boff |= ((hw >> 2) & 1) << 5;
                                        boff |= ((hw >> 3) & 3) << 1;
                                        boff |= ((hw >> 5) & 3) << 6;
                                        boff |= ((hw >> 10) & 3) << 3;
                                        boff |= ((hw >> 12) & 1) << 8;
                                        if (boff & 0x100) boff |= 0xFFFFFE00;
                                        fprintf(stderr, "      [0x%03x] C.BNEZ x%d(x%d), %+d (target=0x%03x)\n",
                                               ha, 8+rs1p, rs1p, boff, ha + boff);
                                    } else {
                                        fprintf(stderr, "      [0x%03x] 16b 0x%04x: op=%d funct3=%d\n", ha, hw, op, f3);
                                    }
                                }
                            }
                            // Count non-zero words in full page
                            int nonzero = 0, zero_words = 0;
                            for (int i = 0; i < 1024; i++) {
                                if (RAM(PAGE_BASE + i) != 0) nonzero++;
                                else zero_words++;
                            }
                            fprintf(stderr, "  Page stats: %d nonzero words, %d zero words (out of 1024)\n", nonzero, zero_words);
                        }

                        // === CANARY PAGE SEARCH ===
                        // Walk page table for VPN containing canary stack area,
                        // then search entire physical page for canary value 0x8474df3e
                        {
                            auto& rf3 = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile;
                            uint32_t satp_now = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__satp;
                            uint32_t pt_base_pa = (satp_now & 0x003FFFFF) << 12;

                            // Dump all registers at crash time
                            fprintf(stderr, "\n=== CRASH REGISTER DUMP ===\n");
                            for (int r = 0; r < 32; r += 4)
                                fprintf(stderr, "  x%d=%08x x%d=%08x x%d=%08x x%d=%08x\n",
                                       r, rf3[r], r+1, rf3[r+1], r+2, rf3[r+2], r+3, rf3[r+3]);

                            // Walk page table for multiple VPNs of interest
                            // VPN 0x95b77 was the canary page in previous run
                            uint32_t vpns_to_check[] = { 0x95b77, 0x95b7a,
                                (rf3[2] >> 12) & 0xFFFFF,    // sp page
                                (rf3[8] >> 12) & 0xFFFFF };  // s0 page
                            const char *vpn_names[] = { "canary_page", "wrong_val_page", "sp_page", "s0_page" };

                            fprintf(stderr, "\n=== CANARY PAGE SEARCH (looking for 0x8474df3e) ===\n");
                            fprintf(stderr, "  SATP=0x%08x pt_base=0x%08x\n", satp_now, pt_base_pa);

                            for (int vi = 0; vi < 4; vi++) {
                                uint32_t vpn = vpns_to_check[vi];
                                uint32_t vpn1 = (vpn >> 10) & 0x3FF;
                                uint32_t vpn0 = vpn & 0x3FF;
                                uint32_t l1_word = (pt_base_pa + vpn1 * 4 - RAM_BASE) >> 2;
                                uint32_t pte1 = (l1_word < RAM_WORDS) ? RAM(l1_word) : 0;
                                uint32_t page_pa = 0;
                                bool mapped = false;
                                if (pte1 & 1) {
                                    if (pte1 & 0xE) { // Superpage
                                        uint32_t ppn1 = (pte1 >> 20) & 0xFFF;
                                        page_pa = (ppn1 << 22) | (vpn0 << 12);
                                        mapped = true;
                                    } else {
                                        uint32_t l2_base = (pte1 >> 10) << 12;
                                        uint32_t l2_word = (l2_base + vpn0 * 4 - RAM_BASE) >> 2;
                                        uint32_t pte0 = (l2_word < RAM_WORDS) ? RAM(l2_word) : 0;
                                        if (pte0 & 1) {
                                            page_pa = ((pte0 >> 10) << 12);
                                            mapped = true;
                                        }
                                    }
                                }
                                fprintf(stderr, "  %s VPN=0x%05x -> %s PA=0x%08x\n",
                                       vpn_names[vi], vpn,
                                       mapped ? "MAPPED" : "UNMAPPED", page_pa);
                                if (mapped && page_pa >= RAM_BASE && page_pa < RAM_BASE + RAM_SIZE) {
                                    uint32_t base_word = (page_pa - RAM_BASE) >> 2;
                                    // Search the 4KB page (1024 words) for canary value
                                    int canary_hits = 0;
                                    for (int w = 0; w < 1024; w++) {
                                        if (RAM(base_word + w) == 0x8474df3e) {
                                            fprintf(stderr, "    FOUND canary at PA 0x%08x (offset 0x%03x, word 0x%x)\n",
                                                   page_pa + w * 4, w * 4, base_word + w);
                                            canary_hits++;
                                        }
                                        if (RAM(base_word + w) == 0x95b7a000) {
                                            fprintf(stderr, "    FOUND wrong_val 0x95b7a000 at PA 0x%08x (offset 0x%03x)\n",
                                                   page_pa + w * 4, w * 4);
                                        }
                                    }
                                    if (canary_hits == 0)
                                        fprintf(stderr, "    canary 0x8474df3e NOT found in this page\n");
                                }
                            }

                            // Also dump stack area (sp ± 256 bytes) to see actual stack contents
                            uint32_t sp_val = rf3[2];
                            fprintf(stderr, "\n=== STACK DUMP (sp=0x%08x) ===\n", sp_val);
                            // Walk page table for sp
                            uint32_t sp_vpn1 = (sp_val >> 22) & 0x3FF;
                            uint32_t sp_vpn0 = (sp_val >> 12) & 0x3FF;
                            uint32_t sp_l1_word = (pt_base_pa + sp_vpn1 * 4 - RAM_BASE) >> 2;
                            uint32_t sp_pte1 = (sp_l1_word < RAM_WORDS) ? RAM(sp_l1_word) : 0;
                            uint32_t sp_pa = 0;
                            bool sp_mapped = false;
                            if (sp_pte1 & 1) {
                                if (sp_pte1 & 0xE) {
                                    sp_pa = (((sp_pte1 >> 20) & 0xFFF) << 22) | (sp_vpn0 << 12) | (sp_val & 0xFFF);
                                    sp_mapped = true;
                                } else {
                                    uint32_t l2_base = (sp_pte1 >> 10) << 12;
                                    uint32_t l2_word = (l2_base + sp_vpn0 * 4 - RAM_BASE) >> 2;
                                    uint32_t pte0 = (l2_word < RAM_WORDS) ? RAM(l2_word) : 0;
                                    if (pte0 & 1) {
                                        sp_pa = ((pte0 >> 10) << 12) | (sp_val & 0xFFF);
                                        sp_mapped = true;
                                    }
                                }
                            }
                            if (sp_mapped && sp_pa >= RAM_BASE && sp_pa < RAM_BASE + RAM_SIZE) {
                                uint32_t sp_word = (sp_pa - RAM_BASE) >> 2;
                                // Dump 64 words (256 bytes) above sp
                                for (int si = 0; si < 64 && sp_word + si < RAM_WORDS; si++) {
                                    if (si % 4 == 0)
                                        fprintf(stderr, "  [sp+0x%03x PA 0x%08x]", si*4, sp_pa + si*4);
                                    fprintf(stderr, " %08x", RAM(sp_word + si));
                                    if (si % 4 == 3) fprintf(stderr, "\n");
                                }
                            }
                        }

                        // === TLB DUMP at crash time ===
                        {
                            fprintf(stderr, "\n=== TLB DUMP (all 16 entries) ===\n");
                            for (int ti = 0; ti < 16; ti++) {
                                uint8_t v = (top->rootp->soc__DOT__cpu_inst__DOT__mmu_inst__DOT__tlb_inst__DOT__entry_valid >> ti) & 1;
                                uint8_t s = (top->rootp->soc__DOT__cpu_inst__DOT__mmu_inst__DOT__tlb_inst__DOT__entry_superpage >> ti) & 1;
                                uint8_t g = (top->rootp->soc__DOT__cpu_inst__DOT__mmu_inst__DOT__tlb_inst__DOT__entry_global >> ti) & 1;
                                uint32_t vpn_val = top->rootp->soc__DOT__cpu_inst__DOT__mmu_inst__DOT__tlb_inst__DOT__entry_vpn[ti];
                                uint32_t asid_val = top->rootp->soc__DOT__cpu_inst__DOT__mmu_inst__DOT__tlb_inst__DOT__entry_asid[ti];
                                uint32_t ppn_val = top->rootp->soc__DOT__cpu_inst__DOT__mmu_inst__DOT__tlb_inst__DOT__entry_ppn[ti];
                                uint32_t flags_val = top->rootp->soc__DOT__cpu_inst__DOT__mmu_inst__DOT__tlb_inst__DOT__entry_flags[ti];
                                uint32_t lru_val = top->rootp->soc__DOT__cpu_inst__DOT__mmu_inst__DOT__tlb_inst__DOT__lru_counter[ti];
                                if (v) {
                                    uint32_t va_base = s ? (vpn_val >> 10) << 22 : vpn_val << 12;
                                    uint32_t pa_base = s ? (ppn_val >> 10) << 22 : ppn_val << 12;
                                    fprintf(stderr, "  [%2d] V=%d S=%d G=%d ASID=%03x VPN=0x%05x PPN=0x%06x flags=%c%c%c%c%c%c%c%c LRU=%d VA=0x%08x->PA=0x%08x\n",
                                           ti, v, s, g, asid_val, vpn_val, ppn_val,
                                           (flags_val & 0x80) ? 'D' : '-',
                                           (flags_val & 0x40) ? 'A' : '-',
                                           (flags_val & 0x20) ? 'G' : '-',
                                           (flags_val & 0x10) ? 'U' : '-',
                                           (flags_val & 0x08) ? 'X' : '-',
                                           (flags_val & 0x04) ? 'W' : '-',
                                           (flags_val & 0x02) ? 'R' : '-',
                                           (flags_val & 0x01) ? 'V' : '-',
                                           lru_val, va_base, pa_base);
                                } else {
                                    fprintf(stderr, "  [%2d] (invalid)\n", ti);
                                }
                            }
                            fprintf(stderr, "=== END TLB DUMP ===\n\n");
                        }
                    }
                }

                // Detect U-mode page faults — defer CSR reading to next cycle
                if (trap_taken && priv == 0 && cycle > 6800000) {
                    uint32_t tc_eff = trap_cause & 0x1F;
                    bool is_irq_eff = (trap_cause & 0x80000000) != 0;
                    if (!is_irq_eff && (tc_eff == 12 || tc_eff == 13 || tc_eff == 15)) {
                        static int umode_pgf_count = 0;
                        umode_pgf_count++;
                        // Log the trap-time info (registers at fault point)
                        if (umode_pgf_count <= 30 || umode_pgf_count >= 40) {
                            uint32_t em_pc = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_pc;
                            uint32_t fault_vaddr = top->rootp->soc__DOT__cpu_inst__DOT__mmu_fault_vaddr;
                            uint32_t data_va = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_alu_result;
                            fprintf(stderr, "[%llu] U-MODE PGF #%d cause=%u pc=0x%08x em_pc=0x%08x fault_vaddr=0x%08x alu_result=0x%08x\n",
                                   (unsigned long long)cycle, umode_pgf_count, trap_cause, pc, em_pc, fault_vaddr, data_va);
                        }
                        pending_umode_fault = true;
                        fault_cycle = cycle;
                        fault_pc = pc;
                        fault_cause_saved = trap_cause;
                    }
                }
            }
        }

        // ---- Memory watchpoints (corruption detection) ----
        // desc_read_finalized_seq crash: descs[44].state_var at word 0x618F94
        // VA 0xC1463E50, PA 0x81863E50
        // Also watch descs pointer at printk_rb_static+4 (word 0x602EEC)
        {
            // Watch a range of descriptor words around the crash site
            static uint32_t prev_desc44[3] = {0, 0, 0}; // state_var, begin, next
            const uint32_t DESC44_WORD = 0x618F94;
            for (int w = 0; w < 3; w++) {
                uint32_t cur = RAM(DESC44_WORD + w);
                if (cur != prev_desc44[w]) {
                    uint32_t bus_addr_val = top->rootp->soc__DOT__mem_addr;
                    uint8_t bus_wstrb_val = top->rootp->soc__DOT__mem_wstrb;
                    uint32_t bus_wdata_val = top->rootp->soc__DOT__mem_wdata;
                    uint32_t ex_mem_pc_val = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_pc;
                    uint8_t ptw_req = top->rootp->soc__DOT__cpu_inst__DOT__ptw_mem_req;
                    fprintf(stderr, "[%llu] WATCH desc44[%d] word 0x%x: 0x%08x -> 0x%08x "
                           "bus=0x%08x ws=0x%x wd=0x%08x pc=0x%08x priv=%d ptw=%d\n",
                           (unsigned long long)cycle, w, DESC44_WORD + w,
                           prev_desc44[w], cur, bus_addr_val, bus_wstrb_val,
                           bus_wdata_val, ex_mem_pc_val, priv, ptw_req);
                    prev_desc44[w] = cur;
                }
            }
            // Watch descs pointer itself (printk_rb_static.desc_ring.descs)
            static uint32_t prev_descs_ptr = 0;
            const uint32_t DESCS_PTR_WORD = 0x602EEC;  // printk_rb_static+4
            uint32_t cur_ptr = RAM(DESCS_PTR_WORD);
            if (cur_ptr != prev_descs_ptr) {
                fprintf(stderr, "[%llu] WATCH descs_ptr word 0x%x: 0x%08x -> 0x%08x pc=0x%08x\n",
                       (unsigned long long)cycle, DESCS_PTR_WORD, prev_descs_ptr, cur_ptr, pc);
                prev_descs_ptr = cur_ptr;
            }
        }

        // ---- Watch PTE word: L1 PTE[0x306] at PA 0x81892C18 (word 0x624B06) ----
        // This PTE maps VA 0xC18xxxxx; correct PPN[1] should be 0x206
        {
            static uint32_t prev_pte306 = 0;
            const uint32_t PTE306_WORD = 0x624B06;
            uint32_t cur_pte = RAM(PTE306_WORD);
            if (cur_pte != prev_pte306) {
                fprintf(stderr, "[%llu] PTE_WATCH L1[0x306] word 0x%x: 0x%08x -> 0x%08x "
                       "PRE_bus=0x%08x ws=0x%x wd=0x%08x pc=0x%08x priv=%d ptw=%d/%d\n",
                       (unsigned long long)cycle, PTE306_WORD,
                       prev_pte306, cur_pte, pre_bus_addr, pre_bus_wstrb,
                       pre_bus_wdata, pre_em_pc, priv, pre_ptw_req, pre_ptw_wr);
                uint32_t ppn1 = cur_pte >> 20;
                if ((cur_pte & 1) && (cur_pte & 0xE) && ppn1 != 0x206) {
                    fprintf(stderr, "  *** PTE CORRUPTED! PPN[1]=0x%x should be 0x206 ***\n", ppn1);
                    // Dump surrounding PTEs for context
                    for (int i = -2; i <= 2; i++) {
                        uint32_t adj = RAM(PTE306_WORD + i);
                        fprintf(stderr, "  PTE[0x%x] = 0x%08x PPN[1]=0x%x\n",
                               0x306 + i, adj, adj >> 20);
                    }
                }
                prev_pte306 = cur_pte;
            }
        }

        // ---- Watch SOURCE word: PA 0x81CEDBC0 (RAM word 0x73B6F0) ----
        // This is the decompress output that gets memcpy'd to crash page offset 0x568
        {
            static uint32_t prev_src_word = 0xDEADBEEF;
            const uint32_t SRC_WORD = 0x73B6F0;  // PA 0x81CEDBC0
            uint32_t cur_src = RAM(SRC_WORD);
            if (cur_src != prev_src_word) {
                fprintf(stderr, "[%llu] SRC_WATCH 0x%x: 0x%08x -> 0x%08x "
                       "PRE_bus=0x%08x ws=0x%x wd=0x%08x pc=0x%08x priv=%d ptw=%d/%d\n",
                       (unsigned long long)cycle, SRC_WORD,
                       prev_src_word, cur_src, pre_bus_addr, pre_bus_wstrb,
                       pre_bus_wdata, pre_em_pc, priv, pre_ptw_req, pre_ptw_wr);
                // If this is THE corrupted write (value == 0x00000023), detailed dump
                if (cur_src == 0x00000023) {
                    fprintf(stderr, "  *** CORRUPTION DETECTED! Value 0x00000023 written ***\n");
                    fprintf(stderr, "  Pipeline: IF_pc=0x%08x em_pc=0x%08x\n",
                           pc, pre_em_pc);
                    // Dump all registers
                    fprintf(stderr, "  REGS: ");
                    for (int r = 0; r < 32; r++)
                        fprintf(stderr, "x%d=%08x ", r,
                               top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile[r]);
                    fprintf(stderr, "\n");
                    // Check bus: is this a regular store or PTW write?
                    uint32_t ptw_addr = (uint32_t)(top->rootp->soc__DOT__cpu_inst__DOT__ptw_mem_addr);
                    uint32_t ptw_wdata = top->rootp->soc__DOT__cpu_inst__DOT__ptw_mem_wdata;
                    fprintf(stderr, "  PTW: req=%d wr=%d addr=0x%08x wdata=0x%08x\n",
                           pre_ptw_req, pre_ptw_wr, ptw_addr, ptw_wdata);
                    // Check data_paddr — what address the MMU resolved
                    uint32_t dpaddr = top->rootp->soc__DOT__cpu_inst__DOT__data_paddr;
                    fprintf(stderr, "  data_paddr=0x%08x ex_mem_alu_result=0x%08x\n",
                           dpaddr,
                           (uint32_t)(top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_alu_result));
                    // Is there an mmu_stall?
                    uint8_t stall_val = top->rootp->soc__DOT__cpu_inst__DOT__mmu_stall;
                    fprintf(stderr, "  mmu_stall=%d\n", stall_val);
                }
                // If written to non-zero first time, dump context
                if (cur_src != 0 && prev_src_word == 0) {
                    fprintf(stderr, "  Context: RAM around 0x73B6F0:\n");
                    for (int i = -4; i <= 4; i++)
                        fprintf(stderr, "    [0x%x] 0x%08x%s\n",
                               SRC_WORD + i, RAM(SRC_WORD + i),
                               (i == 0) ? " <<< SRC" : "");
                }
                prev_src_word = cur_src;
            }
        }

        // ---- Watch crash page: PA 0x82254000 (4KB page containing crash address) ----
        // Word at offset 0x568 (PA 0x82254568, RAM word 0x89515A) — crash instruction
        // Word at offset 0x8C8 (PA 0x822548C8, RAM word 0x895232) — valid code
        {
            static uint32_t prev_crash_word = 0xDEADBEEF;
            static uint32_t prev_valid_word = 0xDEADBEEF;
            const uint32_t CRASH_WORD = 0x89515A;  // offset 0x568
            const uint32_t VALID_WORD = 0x895232;  // offset 0x8C8
            const uint32_t PAGE_BASE  = 0x895000;  // PA 0x82254000
            uint32_t cur_crash = RAM(CRASH_WORD);
            uint32_t cur_valid = RAM(VALID_WORD);
            if (cur_crash != prev_crash_word) {
                fprintf(stderr, "[%llu] CRASH_PAGE[0x568] 0x%x: 0x%08x -> 0x%08x "
                       "PRE_bus=0x%08x ws=0x%x wd=0x%08x pc=0x%08x priv=%d ptw=%d/%d\n",
                       (unsigned long long)cycle, CRASH_WORD,
                       prev_crash_word, cur_crash, pre_bus_addr, pre_bus_wstrb,
                       pre_bus_wdata, pre_em_pc, priv, pre_ptw_req, pre_ptw_wr);
                // Dump CPU registers when the word changes (to find memcpy source)
                if (cur_crash != 0) {
                    fprintf(stderr, "  REGS at write: ");
                    for (int r = 0; r < 32; r++) {
                        fprintf(stderr, "x%d=%08x ", r,
                               top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile[r]);
                    }
                    fprintf(stderr, "\n");
                    // The store is sw a4, 40(t6) at PC 0xc028fcd4.
                    // a4 was loaded by c.lw a4, 40(a1) BEFORE addi a1,a1,64
                    // So source VA = (a1_current - 64) + 40 = a1 - 24
                    uint32_t a1_val = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile[11];
                    uint32_t src_va = a1_val - 24;  // a1 - 64 + 40
                    fprintf(stderr, "  Source VA for a4: 0x%08x (a1=0x%08x, a1_orig=0x%08x + 40)\n",
                           src_va, a1_val, a1_val - 64);

                    // Walk the page table to find the ACTUAL physical address
                    uint32_t satp_now = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__satp;
                    uint32_t pt_base_pa = (satp_now & 0x003FFFFF) << 12;
                    uint32_t vpn1 = (src_va >> 22) & 0x3FF;
                    uint32_t vpn0 = (src_va >> 12) & 0x3FF;
                    uint32_t pg_offset = src_va & 0xFFF;
                    uint32_t l1_word = (pt_base_pa + vpn1 * 4 - RAM_BASE) >> 2;
                    uint32_t pte1 = (l1_word < RAM_WORDS) ? RAM(l1_word) : 0;
                    fprintf(stderr, "  PTW: satp=0x%08x pt_base=0x%08x vpn1=0x%x vpn0=0x%x\n",
                           satp_now, pt_base_pa, vpn1, vpn0);
                    fprintf(stderr, "  PTW: L1 PTE[%d] at word 0x%x = 0x%08x\n", vpn1, l1_word, pte1);
                    uint32_t actual_pa = 0;
                    bool pa_found = false;
                    if (pte1 & 1) {
                        if (pte1 & 0xE) { // Superpage (R/W/X set)
                            actual_pa = ((pte1 >> 10) << 12) | (vpn0 << 12) | pg_offset;
                            // For 4MB superpage: PA = PPN[1]<<22 | VPN[0]<<12 | offset
                            uint32_t ppn1 = (pte1 >> 20) & 0xFFF;
                            actual_pa = (ppn1 << 22) | (vpn0 << 12) | pg_offset;
                            fprintf(stderr, "  PTW: SUPERPAGE ppn1=0x%x -> PA=0x%08x\n", ppn1, actual_pa);
                            pa_found = true;
                        } else { // L2 page table
                            uint32_t l2_base_pa = (pte1 >> 10) << 12;
                            uint32_t l2_word = (l2_base_pa + vpn0 * 4 - RAM_BASE) >> 2;
                            uint32_t pte0 = (l2_word < RAM_WORDS) ? RAM(l2_word) : 0;
                            fprintf(stderr, "  PTW: L2 base=0x%08x PTE[%d] at word 0x%x = 0x%08x\n",
                                   l2_base_pa, vpn0, l2_word, pte0);
                            if (pte0 & 1) {
                                actual_pa = ((pte0 >> 10) << 12) | pg_offset;
                                fprintf(stderr, "  PTW: PA=0x%08x\n", actual_pa);
                                pa_found = true;
                            }
                        }
                    }
                    if (pa_found && actual_pa >= RAM_BASE && actual_pa < RAM_BASE + RAM_SIZE) {
                        uint32_t src_word_idx = (actual_pa - RAM_BASE) >> 2;
                        fprintf(stderr, "  SOURCE at PA 0x%08x (word 0x%x) = 0x%08x %s\n",
                               actual_pa, src_word_idx, RAM(src_word_idx),
                               (RAM(src_word_idx) == cur_crash) ? "(MATCHES store data)" : "(DIFFERS from store data!)");
                        // Dump surrounding words for context
                        fprintf(stderr, "  SOURCE area (PA 0x%08x ± 8 words):\n", actual_pa);
                        for (int i = -4; i <= 4; i++) {
                            uint32_t idx = src_word_idx + i;
                            if (idx < RAM_WORDS)
                                fprintf(stderr, "    [PA 0x%08x word 0x%x] 0x%08x%s\n",
                                       actual_pa + i * 4, idx, RAM(idx),
                                       (i == 0) ? " <<< a4 source" : "");
                        }
                    }
                    // Also check if simple linear map gives same result
                    {
                        uint32_t linear_pa = src_va - 0x40000000;
                        if (linear_pa >= RAM_BASE && linear_pa < RAM_BASE + RAM_SIZE) {
                            uint32_t lin_word = (linear_pa - RAM_BASE) >> 2;
                            fprintf(stderr, "  LINEAR MAP: VA 0x%08x -> PA 0x%08x = 0x%08x\n",
                                   src_va, linear_pa, RAM(lin_word));
                        }
                    }
                }
                prev_crash_word = cur_crash;
            }
            if (cur_valid != prev_valid_word) {
                fprintf(stderr, "[%llu] CRASH_PAGE[0x8C8] 0x%x: 0x%08x -> 0x%08x "
                       "PRE_bus=0x%08x ws=0x%x wd=0x%08x pc=0x%08x priv=%d ptw=%d/%d\n",
                       (unsigned long long)cycle, VALID_WORD,
                       prev_valid_word, cur_valid, pre_bus_addr, pre_bus_wstrb,
                       pre_bus_wdata, pre_em_pc, priv, pre_ptw_req, pre_ptw_wr);
                prev_valid_word = cur_valid;
            }
            // ---- Targeted trace: operations around key cycles ----
            // Window 1: Source corruption at cycle ~52798063
            // Window 2: Crash page store at cycle ~53144483
            if ((cycle >= 52798020 && cycle <= 52798080) ||
                (cycle >= 53144470 && cycle <= 53144490)) {
                uint8_t mem_read_val = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_mem_read;
                uint8_t mem_write_val = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_mem_write;
                uint32_t bus_addr_val = top->rootp->soc__DOT__mem_addr;
                uint32_t rdata_val = top->rootp->soc__DOT__mem_rdata;
                uint32_t wdata_val = top->rootp->soc__DOT__mem_wdata;
                uint8_t wstrb_val = top->rootp->soc__DOT__mem_wstrb;
                uint32_t data_paddr_val = top->rootp->soc__DOT__cpu_inst__DOT__data_paddr;
                uint8_t stall_val = top->rootp->soc__DOT__cpu_inst__DOT__mmu_stall;
                uint32_t ex_mem_rd = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_rd_addr;
                uint8_t ptw_r = top->rootp->soc__DOT__cpu_inst__DOT__ptw_mem_req;
                uint32_t alu_res = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_alu_result;
                uint32_t id_ex_pc_val = top->rootp->soc__DOT__cpu_inst__DOT__id_ex_pc;
                uint8_t ex_mem_rw = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_reg_write;
                uint8_t flush = top->rootp->soc__DOT__cpu_inst__DOT__pipeline_flush;
                auto& rf = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile;
                // WB stage info
                uint32_t wb_rd = top->rootp->soc__DOT__cpu_inst__DOT__mem_wb_rd_addr;
                uint8_t wb_rw = top->rootp->soc__DOT__cpu_inst__DOT__mem_wb_reg_write;
                uint32_t wb_data = top->rootp->soc__DOT__cpu_inst__DOT__wb_data;
                fprintf(stderr, "[%llu] TRACE em_pc=0x%08x id_pc=0x%08x %s%s bus=0x%08x paddr=0x%08x "
                       "alu=0x%08x rdata=0x%08x wdata=0x%08x ws=0x%x rd=x%d rw=%d "
                       "stall=%d ptw=%d flush=%d "
                       "wb:x%d%s=0x%08x "
                       "a0=0x%08x s3=0x%08x a1=0x%08x a5=0x%08x\n",
                       (unsigned long long)cycle, pre_em_pc, id_ex_pc_val,
                       mem_read_val ? "RD" : "", (pre_bus_wstrb != 0) ? "WR" : "",
                       bus_addr_val, data_paddr_val, alu_res,
                       rdata_val, wdata_val, wstrb_val,
                       ex_mem_rd, ex_mem_rw, stall_val, ptw_r, flush,
                       wb_rd, wb_rw ? "<-" : "  ", wb_data,
                       rf[10], rf[19], rf[11], rf[15]);
            }

            // Count total page writes between 25M-150M (initramfs extraction window)
            static int page_write_count = 0;
            static bool page_write_counted = false;
            if (cycle >= 25000000 && cycle <= 150000000 && !page_write_counted) {
                if (pre_bus_wstrb != 0 && pre_bus_addr >= 0x82254000 && pre_bus_addr < 0x82255000) {
                    page_write_count++;
                    // Log first 10 and every 100th write to the page
                    if (page_write_count <= 10 || page_write_count % 100 == 0) {
                        fprintf(stderr, "[%llu] PAGE_WRITE #%d PA=0x%08x ws=0x%x wd=0x%08x pc=0x%08x ptw=%d\n",
                               (unsigned long long)cycle, page_write_count,
                               pre_bus_addr, pre_bus_wstrb, pre_bus_wdata, pre_em_pc, pre_ptw_req);
                    }
                }
            }
        }

        // ---- Dynamic canary pair tracker ----
        // When we see a U-mode store of the canary value 0x8474df3e, record the PA.
        // The first two DISTINCT PAs give us __stack_chk_guard and the saved copy.
        // Then watch the next word at each PA for divergence.
        {
            static uint32_t canary_pa[2] = {0, 0};  // PAs of the two canary copies
            static int canary_count = 0;
            static bool canary_ever_diverged = false;

            // Detect canary stores (U-mode stores of 0x8474df3e)
            if (pre_priv == 0 && pre_bus_wstrb != 0 && !pre_ptw_req &&
                pre_bus_wdata == 0x8474df3e && pre_bus_addr >= RAM_BASE &&
                pre_bus_addr < RAM_BASE + RAM_SIZE) {
                uint32_t pa = pre_bus_addr;
                bool is_new = true;
                for (int i = 0; i < canary_count && i < 2; i++) {
                    // Same page = same copy (guard or saved)
                    if ((pa & 0xFFFFF000) == (canary_pa[i] & 0xFFFFF000))
                        is_new = false;
                }
                if (is_new && canary_count < 2) {
                    canary_pa[canary_count] = pa;
                    canary_count++;
                    fprintf(stderr, "[%llu] CANARY_PAIR #%d PA=0x%08x (word 0x%x) pc=0x%08x\n",
                           (unsigned long long)cycle, canary_count, pa,
                           (pa - RAM_BASE) >> 2, pre_em_pc);
                }
            }

            // Once we have both canary locations, watch for changes and divergence
            if (canary_count == 2) {
                uint32_t w0_guard = (canary_pa[0] - RAM_BASE) >> 2;
                uint32_t w0_saved = (canary_pa[1] - RAM_BASE) >> 2;
                // Check the NEXT word (offset +1) at each location
                uint32_t g_hi = RAM(w0_guard + 1);
                uint32_t s_hi = RAM(w0_saved + 1);

                // Track changes to high word at guard location
                static uint32_t prev_g_hi = 0xDEADBEEF;
                if (g_hi != prev_g_hi) {
                    fprintf(stderr, "[%llu] CANARY_GUARD_HI word 0x%x: 0x%08x -> 0x%08x "
                           "bus=0x%08x ws=0x%x wd=0x%08x pc=0x%08x priv=%d ptw=%d\n",
                           (unsigned long long)cycle, w0_guard + 1,
                           prev_g_hi, g_hi, pre_bus_addr, pre_bus_wstrb,
                           pre_bus_wdata, pre_em_pc, priv, pre_ptw_req);
                    prev_g_hi = g_hi;
                }
                // Track changes to high word at saved location
                static uint32_t prev_s_hi = 0xDEADBEEF;
                if (s_hi != prev_s_hi) {
                    fprintf(stderr, "[%llu] CANARY_SAVED_HI word 0x%x: 0x%08x -> 0x%08x "
                           "bus=0x%08x ws=0x%x wd=0x%08x pc=0x%08x priv=%d ptw=%d\n",
                           (unsigned long long)cycle, w0_saved + 1,
                           prev_s_hi, s_hi, pre_bus_addr, pre_bus_wstrb,
                           pre_bus_wdata, pre_em_pc, priv, pre_ptw_req);
                    prev_s_hi = s_hi;
                }
                // Divergence check
                if (!canary_ever_diverged && prev_g_hi != 0xDEADBEEF && prev_s_hi != 0xDEADBEEF) {
                    if (g_hi != s_hi) {
                        canary_ever_diverged = true;
                        fprintf(stderr, "[%llu] *** CANARY_DIVERGE! guard_hi=0x%08x saved_hi=0x%08x "
                               "guard_lo=0x%08x saved_lo=0x%08x pc=0x%08x priv=%d ***\n",
                               (unsigned long long)cycle, g_hi, s_hi,
                               RAM(w0_guard), RAM(w0_saved), pc, priv);
                        // Dump the bus state and pipeline state
                        fprintf(stderr, "  bus=0x%08x ws=0x%x wd=0x%08x em_pc=0x%08x\n",
                               pre_bus_addr, pre_bus_wstrb, pre_bus_wdata, pre_em_pc);
                        // Dump both full 8-byte canaries with surrounding context
                        fprintf(stderr, "  guard area (PA 0x%08x):", canary_pa[0]);
                        for (int j = -1; j <= 4; j++)
                            fprintf(stderr, " [%+d]=0x%08x", j, RAM(w0_guard + j));
                        fprintf(stderr, "\n  saved area (PA 0x%08x):", canary_pa[1]);
                        for (int j = -1; j <= 4; j++)
                            fprintf(stderr, " [%+d]=0x%08x", j, RAM(w0_saved + j));
                        fprintf(stderr, "\n");
                    }
                }
            }
        }

        // ---- Ground truth cpio comparison at cycle 53M ----
        // By this cycle inflate_fast has written the wrong word (cycle 52798063)
        // Compare decompressed cpio in RAM against ground truth to find FIRST error
        if (cycle == 53000000 && gt_data && gt_size > 0) {
            fprintf(stderr, "\n=== CPIO GROUND TRUTH COMPARISON at cycle 53M ===\n");
            // Ground truth first 4 words (LE): 0x37303730 0x30303130 0x32413231 0x30303644
            uint32_t gt_w0 = *(uint32_t*)&gt_data[0];
            uint32_t gt_w1 = *(uint32_t*)&gt_data[4];
            uint32_t gt_w2 = *(uint32_t*)&gt_data[8];
            uint32_t gt_w3 = *(uint32_t*)&gt_data[12];
            fprintf(stderr, "  GT pattern: %08x %08x %08x %08x\n", gt_w0, gt_w1, gt_w2, gt_w3);

            // Search RAM for cpio header (scan PA 0x81000000+ = word 0x400000+)
            uint32_t cpio_base = 0;
            bool found_cpio = false;
            for (uint32_t w = 0x400000; w < RAM_WORDS - 4; w++) {
                if (RAM(w) == gt_w0 && RAM(w+1) == gt_w1 &&
                    RAM(w+2) == gt_w2 && RAM(w+3) == gt_w3) {
                    cpio_base = w;
                    found_cpio = true;
                    fprintf(stderr, "  Found cpio at word 0x%x (PA 0x%08x)\n",
                           w, RAM_BASE + w * 4);
                    break;
                }
            }

            if (found_cpio) {
                int total_mismatches = 0;
                int first_mismatch = -1;
                uint32_t gt_bytes = gt_size;
                // Make sure we don't read past RAM
                uint32_t max_words = cpio_base + (gt_bytes + 3) / 4;
                if (max_words > RAM_WORDS) {
                    gt_bytes = (RAM_WORDS - cpio_base) * 4;
                    fprintf(stderr, "  WARNING: cpio extends beyond RAM, comparing %u bytes\n", gt_bytes);
                }
                for (uint32_t i = 0; i < gt_bytes; i++) {
                    uint32_t word_idx = cpio_base + i / 4;
                    uint32_t byte_pos = i % 4;
                    uint8_t ram_byte = (RAM(word_idx) >> (byte_pos * 8)) & 0xFF;
                    if (ram_byte != gt_data[i]) {
                        total_mismatches++;
                        if (first_mismatch < 0) first_mismatch = (int)i;
                        if (total_mismatches <= 30) {
                            fprintf(stderr, "  MISMATCH[%d] offset=0x%06x RAM=0x%02x GT=0x%02x "
                                   "(word 0x%x PA 0x%08x)\n",
                                   total_mismatches, i, ram_byte, gt_data[i],
                                   word_idx, RAM_BASE + word_idx * 4);
                        }
                    }
                }
                fprintf(stderr, "  Total: %d mismatches in %u bytes", total_mismatches, gt_bytes);
                if (first_mismatch >= 0)
                    fprintf(stderr, " (first at offset 0x%x)", first_mismatch);
                fprintf(stderr, "\n");

                // If there's a first mismatch, dump context
                if (first_mismatch >= 0) {
                    int fm_start = (first_mismatch / 4) * 4;
                    fprintf(stderr, "  Context around first mismatch (offset 0x%x):\n", first_mismatch);
                    for (int j = -8; j <= 8; j++) {
                        int off = fm_start + j * 4;
                        if (off < 0 || (uint32_t)off + 4 > gt_bytes) continue;
                        uint32_t w_idx = cpio_base + off / 4;
                        uint32_t ram_w = RAM(w_idx);
                        uint32_t gt_w = *(uint32_t*)&gt_data[off];
                        fprintf(stderr, "    [off=0x%06x word=0x%x] RAM=0x%08x GT=0x%08x%s\n",
                               off, w_idx, ram_w, gt_w,
                               (ram_w != gt_w) ? " ***DIFF***" : "");
                    }

                    // Compute the wrong word's offset in cpio
                    uint32_t src_word = 0x73B6F0;  // PA 0x81CEDBC0
                    if (src_word >= cpio_base && src_word < cpio_base + gt_bytes / 4) {
                        uint32_t cpio_off = (src_word - cpio_base) * 4;
                        fprintf(stderr, "  Known bad word 0x73B6F0 is at cpio offset 0x%x\n", cpio_off);
                        fprintf(stderr, "    RAM: 0x%08x  GT: 0x%08x\n",
                               RAM(src_word), *(uint32_t*)&gt_data[cpio_off]);
                    }
                }
            } else {
                fprintf(stderr, "  Cpio header not found in RAM!\n");
                // Try a broader search with just first 2 words
                for (uint32_t w = 0x100000; w < RAM_WORDS - 2; w++) {
                    if (RAM(w) == gt_w0 && RAM(w+1) == gt_w1) {
                        fprintf(stderr, "  Partial match (2 words) at word 0x%x (PA 0x%08x)\n",
                               w, RAM_BASE + w * 4);
                        fprintf(stderr, "    w[2]=0x%08x (expect 0x%08x) w[3]=0x%08x (expect 0x%08x)\n",
                               RAM(w+2), gt_w2, RAM(w+3), gt_w3);
                        break;
                    }
                }
            }
        }

        // ---- Page table validation at cycle 10M ----
        if (cycle == 10000000 && (satp & 0x80000000)) {
            uint32_t pt_base_word = (((satp & 0x003FFFFF) << 12) - RAM_BASE) >> 2;
            fprintf(stderr, "=== PAGE TABLE DUMP at cycle 10M (satp=0x%08x, base_word=0x%x) ===\n",
                   satp, pt_base_word);
            int errors = 0;
            for (int vpn1 = 0x300; vpn1 <= 0x309; vpn1++) {
                uint32_t pte = RAM(pt_base_word + vpn1);
                uint32_t ppn1 = pte >> 20;
                uint32_t expected = vpn1 - 0xFF;
                bool is_super = (pte & 1) && (pte & 0xE);
                const char *status = "";
                if (is_super && ppn1 != expected) { status = " *** WRONG ***"; errors++; }
                fprintf(stderr, "  PTE[0x%x] = 0x%08x PPN[1]=0x%03x (expect 0x%03x) flags=%s%s%s%s%s%s%s%s%s\n",
                       vpn1, pte, ppn1, expected,
                       (pte & 1) ? "V" : "", (pte & 2) ? "R" : "", (pte & 4) ? "W" : "",
                       (pte & 8) ? "X" : "", (pte & 0x10) ? "U" : "", (pte & 0x20) ? "G" : "",
                       (pte & 0x40) ? "A" : "", (pte & 0x80) ? "D" : "", status);
            }
            if (errors) fprintf(stderr, "  *** %d PTE ERROR(S) FOUND ***\n", errors);
            else fprintf(stderr, "  All linear mapping PTEs correct\n");
        }

        // ---- Dump crash page at cycle 143M (just before init enters U-mode) ----
        if (cycle == 143000000) {
            const uint32_t PAGE_BASE = 0x895000;
            fprintf(stderr, "=== CRASH PAGE DUMP at cycle 143M (PA 0x82254000-0x82254FFF) ===\n");
            // Dump entire crash function area: 0x550-0x600
            fprintf(stderr, "  Crash function (offset 0x550-0x600):\n");
            for (uint32_t off = 0x550; off <= 0x600; off += 4) {
                uint32_t w = RAM(PAGE_BASE + off/4);
                fprintf(stderr, "    [PA 0x%08x off=0x%03x] 0x%08x (%04x %04x)%s%s\n",
                       0x82254000 + off, off, w, w & 0xFFFF, (w >> 16) & 0xFFFF,
                       (off == 0x568) ? " <<<CRASH" : "",
                       (off == 0x5D8) ? " <<<BRANCH" : "");
            }
        }

        // ---- PTW A/D bit write monitor ----
        // Track ALL PTW writes to detect misaddressed A/D updates
        {
            uint8_t ptw_req = top->rootp->soc__DOT__cpu_inst__DOT__ptw_mem_req;
            uint8_t ptw_wr = top->rootp->soc__DOT__cpu_inst__DOT__ptw_mem_write;
            if (ptw_req && ptw_wr) {
                uint32_t ptw_addr = (uint32_t)(top->rootp->soc__DOT__cpu_inst__DOT__ptw_mem_addr); // low 32 bits
                uint32_t ptw_wdata = top->rootp->soc__DOT__cpu_inst__DOT__ptw_mem_wdata;
                static int ptw_write_count = 0;
                ptw_write_count++;
                // Log first 5 PTW writes, then every 500th
                if (ptw_write_count <= 5 || ptw_write_count % 500 == 0) {
                    fprintf(stderr, "[%llu] PTW_WRITE PA=0x%08x wdata=0x%08x pc=0x%08x priv=%d (#%d)\n",
                           (unsigned long long)cycle, ptw_addr, ptw_wdata, pc, priv, ptw_write_count);
                }
                // Alert if PTW writes to descriptor region (0x81863C40 - 0x8186CC40 = 49KB)
                if (ptw_addr >= 0x81863C40 && ptw_addr < 0x8186CC40) {
                    fprintf(stderr, "[%llu] *** PTW WRITE TO DESCS REGION! PA=0x%08x wdata=0x%08x ***\n",
                           (unsigned long long)cycle, ptw_addr, ptw_wdata);
                }
            }
        }

        // ---- Traps ----
        if (trap_taken) {
            if (trap_cause == 12 || trap_cause == 13 || trap_cause == 15) {
                page_faults++;
            }
            // Log first few traps of each type
            bool is_irq = (trap_cause & 0x80000000) != 0;
            uint32_t tc = trap_cause & 0x1F;
            if (is_irq) {
                static int irq_counts[32] = {};
                if (irq_counts[tc]++ < 3)
                    fprintf(stderr, "[%llu] IRQ cause=0x%08x pc=0x%08x priv=%d\n",
                           (unsigned long long)cycle, trap_cause, pc, priv);
            } else {
                static int trap_counts[32] = {};
                trap_counts[tc]++;
                if (trap_counts[tc] <= 10)
                    fprintf(stderr, "[%llu] TRAP cause=%u pc=0x%08x priv=%d\n",
                           (unsigned long long)cycle, trap_cause, pc, priv);
                // For cause=2 (illegal instruction), dump the faulting instruction
                if (tc == 2 && trap_counts[tc] <= 10) {
                    uint32_t ex_mem_pc_val = top->rootp->soc__DOT__cpu_inst__DOT__ex_mem_pc;
                    if (ex_mem_pc_val >= 0x80000000 && ex_mem_pc_val < 0x80000000 + RAM_SIZE) {
                        uint32_t byte_off = ex_mem_pc_val - 0x80000000;
                        uint32_t w0 = RAM(byte_off >> 2);
                        uint32_t w1 = RAM((byte_off >> 2) + 1);
                        uint32_t shift = (byte_off & 3) * 8;
                        uint32_t insn = (shift == 0) ? w0 : (w0 >> shift) | (w1 << (32 - shift));
                        uint32_t csr_num = (insn >> 20) & 0xFFF;
                        fprintf(stderr, "  illegal_csr: ex_mem_pc=0x%08x insn=0x%08x csr=0x%03x\n",
                               ex_mem_pc_val, insn, csr_num);
                    }
                }
            }
            // ecall from S-mode (cause=9) — log first few SBI calls
            if (trap_cause == 9) {
                static int ecall_count = 0;
                if (ecall_count++ < 10) {
                    uint32_t a7 = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile[17];
                    uint32_t a6 = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile[16];
                    fprintf(stderr, "  SBI ext=0x%x func=0x%x\n", a7, a6);
                }
            }
        }

        // ---- UART output ----
        if (uart_valid) {
            uart_count++;
            putchar(uart_data);
            if (interactive)
                fflush(stdout);  // flush every char for responsive echo
            else if (uart_data == '\n' || uart_count % 64 == 0)
                fflush(stdout);

            // Detect shell prompt "# " or "$ "
            if ((prev_char == '#' || prev_char == '$') && uart_data == ' ') {
                if (!saw_shell_prompt) {
                    saw_shell_prompt = true;
                    shell_cycle = cycle;
                    fflush(stdout);
                    fprintf(stderr, "\n*** SHELL PROMPT DETECTED at cycle %llu ***\n",
                           (unsigned long long)cycle);
                    if (interactive) {
                        fprintf(stderr, "Interactive mode active. Type commands. Ctrl+] to quit.\n");
                        fflush(stderr);
                        enable_raw_mode();
                        signal(SIGINT, signal_handler);
                        signal(SIGTERM, signal_handler);
                    }
                    fflush(stderr);
                }
            }
            prev_char = uart_data;

            if (uart_count >= MAX_UART) {
                fprintf(stderr, "\n[%llu] UART limit reached (%d chars)\n",
                       (unsigned long long)cycle, uart_count);
                break;
            }
        }

        // ---- Auto-exit in test mode after shell prompt + 2M grace cycles ----
        if (!interactive && saw_shell_prompt && (cycle - shell_cycle) > 2000000) {
            fprintf(stderr, "\n[%llu] Shell detected, stopping after grace period\n",
                   (unsigned long long)cycle);
            break;
        }

        // ---- Progress report every 5M cycles (suppressed after shell in interactive mode) ----
        if (cycle % 5000000 == 0 && cycle > 0 &&
            !(interactive && saw_shell_prompt)) {
            uint64_t mtime_val = top->rootp->soc__DOT__cpu_inst__DOT__clint_inst__DOT__mtime;
            uint8_t stip_sw = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mip_stip_sw;
            uint8_t sie_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mstatus_sie;
            uint32_t mie_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mie;
            uint32_t a0 = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile[10];
            uint32_t ra = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile[1];
            uint32_t mcounteren_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mcounteren;
            uint32_t mtimecmp_lo = (uint32_t)(top->rootp->soc__DOT__cpu_inst__DOT__clint_inst__DOT__mtimecmp);
            uint32_t mip_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mip;
            uint32_t sip_val = mip_val & 0x222;
            uint8_t int_pending = top->rootp->soc__DOT__cpu_inst__DOT__interrupt_pending;
            // Skip initramfs integrity check in progress (too expensive every 5M cycles)
            const char *ck_status = "SKIP";

            fprintf(stderr, "[%lluM] pc=0x%08x priv=%d uart=%d pgf=%d "
                   "mtime=0x%llx mtimecmp=0x%x "
                   "mip=0x%x sip=0x%x int_pend=%d stip_sw=%d "
                   "sie=%d mie=0x%x "
                   "a0=0x%x ra=0x%08x mcnten=0x%x satp=0x%08x initramfs=%s\n",
                   (unsigned long long)cycle / 1000000, pc, priv,
                   uart_count, page_faults,
                   (unsigned long long)mtime_val, mtimecmp_lo,
                   mip_val, sip_val, int_pending, stip_sw,
                   sie_val, mie_val,
                   a0, ra, mcounteren_val, satp,
                   ck_status);

            // (initramfs integrity check skipped for performance)
        }

        // ---- Register checksum (matching iverilog linux_shell_tb.v CKSUM format) ----
        // Skip regfile[0] to avoid X-propagation difference
        if (cycle > 0 && cycle % 100000 == 0 && cycle <= 10000000) {
            auto& rf = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile;
            uint32_t rf_cksum = 0;
            for (int r = 1; r < 32; r++)
                rf_cksum ^= rf[r];
            uint32_t mstatus_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mstatus;
            uint32_t mepc_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mepc;
            uint32_t mcause_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mcause;
            uint64_t mtime_val = top->rootp->soc__DOT__cpu_inst__DOT__clint_inst__DOT__mtime;
            fprintf(stderr, "CKSUM[%llu] pc=0x%08x rf=0x%08x mstatus=0x%08x mepc=0x%08x mcause=0x%08x satp=0x%08x priv=%d mtime=0x%08x%08x\n",
                   (unsigned long long)cycle, pc, rf_cksum, mstatus_val, mepc_val, mcause_val, satp, priv,
                   (uint32_t)(mtime_val >> 32), (uint32_t)(mtime_val & 0xFFFFFFFF));
        }

        // ---- Divergence hunt: mtime + PC + registers at key checkpoints ----
        if (cycle == 100000 || cycle == 500000 || cycle == 1000000 ||
            cycle == 1200000 || cycle == 1400000 || cycle == 1500000 ||
            cycle == 1600000 || cycle == 1700000 || cycle == 1750000 ||
            cycle == 1800000 || cycle == 1820000 || cycle == 1840000 ||
            cycle == 1850000 || cycle == 1860000 || cycle == 1870000 ||
            cycle == 1875000 || cycle == 1880000 || cycle == 1882000 ||
            cycle == 1884000 || cycle == 1885000 || cycle == 1886000 ||
            cycle == 1887000 || cycle == 1887200 || cycle == 1887400 ||
            cycle == 1887600 || cycle == 1887800 || cycle == 1887900 ||
            cycle == 1888000 || cycle == 1890000) {
            uint32_t ck1 = 0;
            for (uint32_t i = 0; i < 0x40000; i++)
                ck1 ^= RAM(i);
            uint32_t mtvec_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mtvec;
            uint32_t mepc_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mepc;
            uint32_t mcause_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mcause;
            uint32_t mstatus_val = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__mstatus;
            uint64_t mtime_val = top->rootp->soc__DOT__cpu_inst__DOT__clint_inst__DOT__mtime;
            uint64_t mtimecmp_val = top->rootp->soc__DOT__cpu_inst__DOT__clint_inst__DOT__mtimecmp;
            auto& rf = top->rootp->soc__DOT__cpu_inst__DOT__reg_file_inst__DOT__regfile;
            uint8_t stall_v = top->rootp->soc__DOT__cpu_inst__DOT__pipeline_stall;
            uint8_t flush_v = top->rootp->soc__DOT__cpu_inst__DOT__pipeline_flush;
            uint32_t ifid_i = top->rootp->soc__DOT__cpu_inst__DOT__if_id_instruction;
            fprintf(stderr, "[%llu] CKPT pc=0x%08x mtime=0x%08x%08x mtimecmp=0x%08x%08x priv=%d mtvec=0x%08x\n",
                   (unsigned long long)cycle, pc,
                   (uint32_t)(mtime_val >> 32), (uint32_t)(mtime_val & 0xFFFFFFFF),
                   (uint32_t)(mtimecmp_val >> 32), (uint32_t)(mtimecmp_val & 0xFFFFFFFF),
                   priv, mtvec_val);
            fprintf(stderr, "  regs: ra=0x%08x sp=0x%08x a0=0x%08x a1=0x%08x a2=0x%08x a3=0x%08x\n",
                   rf[1], rf[2], rf[10], rf[11], rf[12], rf[13]);
            fprintf(stderr, "  regs: a4=0x%08x a5=0x%08x t0=0x%08x t1=0x%08x s0=0x%08x s1=0x%08x\n",
                   rf[14], rf[15], rf[5], rf[6], rf[8], rf[9]);
            fprintf(stderr, "  csr: mepc=0x%08x mcause=0x%08x mstatus=0x%08x\n",
                   mepc_val, mcause_val, mstatus_val);
            fprintf(stderr, "  cksum1mb=0x%08x stall=%d flush=%d instr=0x%08x\n",
                   ck1, stall_v, flush_v, ifid_i);
        }
    }

    // ====================================================================
    // Cleanup
    // ====================================================================
    disable_raw_mode();

    fprintf(stderr, "\n=== RESULTS ===\n");
    fprintf(stderr, "Total cycles:    %llu\n", (unsigned long long)cycle);
    fprintf(stderr, "UART chars:      %d\n", uart_count);
    fprintf(stderr, "Page faults:     %d\n", page_faults);
    fprintf(stderr, "Entered S-mode:  %s\n", entered_smode ? "YES" : "NO");
    fprintf(stderr, "Kernel virtual:  %s\n", saw_kernel_virt ? "YES" : "NO");
    fprintf(stderr, "Shell prompt:    %s\n", saw_shell_prompt ? "YES" : "NO");

    top->final();
    delete top;
    delete[] gt_data;
    return saw_shell_prompt ? 0 : 1;
}
