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
static const uint64_t MAX_CYCLES   = 600000000ULL; // 600M for interactive
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
        top->clk = 1;
        top->eval();

        // ---- Read internal signals ----
        uint32_t pc   = top->rootp->soc__DOT__cpu_inst__DOT__pc_reg;
        uint8_t  priv = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__priv_level;
        uint32_t satp = top->rootp->soc__DOT__cpu_inst__DOT__csr_file_inst__DOT__satp;

        uint8_t  trap_taken = top->rootp->soc__DOT__cpu_inst__DOT__take_trap_effective;
        uint32_t trap_cause = top->rootp->soc__DOT__cpu_inst__DOT__trap_cause_final;

        uint8_t  uart_valid = top->rootp->soc__DOT__uart_inst__DOT__sim_tx_valid;
        uint8_t  uart_data  = top->rootp->soc__DOT__uart_inst__DOT__sim_tx_data;

        // ---- PTE injection for identity-mapped MMIO ----
        if (satp != prev_satp && (satp & 0x80000000)) {
            uint32_t pt_base_word = (((satp & 0x003FFFFF) << 12) - RAM_BASE) >> 2;
            // UART: 0x10000000 -> PTE index 0x040
            RAM(pt_base_word + 0x040) = 0x040000C7;
            // CLINT: 0x02000000 -> PTE index 0x008
            RAM(pt_base_word + 0x008) = 0x008000C7;
            // PLIC: 0x0C000000 -> PTE index 0x030
            RAM(pt_base_word + 0x030) = 0x030000C7;
            prev_satp = satp;
        } else if (satp != prev_satp) {
            prev_satp = satp;
        }

        // ---- S-mode entry ----
        if (priv == 1 && !entered_smode) {
            entered_smode = true;
            fprintf(stderr, "[%llu] ENTERED S-MODE pc=0x%08x\n",
                   (unsigned long long)cycle, pc);
        }

        // ---- Kernel virtual address space ----
        if (!saw_kernel_virt && pc >= 0xC0000000) {
            saw_kernel_virt = true;
            fprintf(stderr, "[%llu] KERNEL VIRTUAL pc=0x%08x (MMU active)\n",
                   (unsigned long long)cycle, pc);
        }

        // ---- Page faults ----
        if (trap_taken && (trap_cause == 12 || trap_cause == 13 || trap_cause == 15)) {
            page_faults++;
            if (page_faults <= 3) {
                uint32_t tpc = top->rootp->soc__DOT__cpu_inst__DOT__trap_pc_final;
                uint32_t tval = top->rootp->soc__DOT__cpu_inst__DOT__trap_val;
                fprintf(stderr, "[%llu] PGF #%d cause=%u pc=0x%08x stval=0x%08x\n",
                       (unsigned long long)cycle, page_faults, trap_cause,
                       tpc, tval);
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
            fprintf(stderr, "[%lluM] pc=0x%08x priv=%d uart=%d pgf=%d\n",
                   (unsigned long long)cycle / 1000000, pc, priv,
                   uart_count, page_faults);
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
    return saw_shell_prompt ? 0 : 1;
}
