# =============================================================================
# Bare-metal UART echo firmware for RISC-V RV32IMAC SoC
# =============================================================================
# Runs in M-mode. Initializes UART at 115200 baud (20MHz clock, divisor=11),
# prints a banner, then enters an echo loop with simple line editing.
#
# Memory map:
#   0x80000000 - 0x80000FFF : 4KB RAM (stack at top)
#   0x10000000 - 0x1000001F : UART (16550, mmio32 4-byte spacing)
# =============================================================================

.section .text
.globl _start

# UART register offsets (mmio32 = 4-byte spacing)
.equ UART_BASE, 0x10000000
.equ UART_THR,  0x00    # Transmit Holding Register (write) / RBR (read)
.equ UART_IER,  0x04    # Interrupt Enable Register
.equ UART_FCR,  0x08    # FIFO Control Register (write)
.equ UART_LCR,  0x0C    # Line Control Register
.equ UART_MCR,  0x10    # Modem Control Register
.equ UART_LSR,  0x14    # Line Status Register
.equ UART_DLL,  0x00    # Divisor Latch Low (when DLAB=1)
.equ UART_DLM,  0x04    # Divisor Latch High (when DLAB=1)

# LSR bit masks
.equ LSR_DR,    0x01    # Data Ready
.equ LSR_THRE,  0x20    # THR Empty

# RAM layout
.equ STACK_TOP, 0x80001000   # Top of 4KB RAM

_start:
    # =====================================================
    # 1. Initialize stack pointer
    # =====================================================
    li      sp, STACK_TOP

    # =====================================================
    # 2. Initialize UART: 115200 8N1
    # =====================================================
    li      a0, UART_BASE

    # Set DLAB=1 to access divisor registers
    li      t0, 0x83            # DLAB=1, 8-bit word length, 1 stop bit, no parity
    sb      t0, UART_LCR(a0)

    # Set divisor = 11 for 115200 baud at 20MHz
    # baud = 20,000,000 / (16 * 11) = 113636 (~1.4% error)
    li      t0, 11
    sb      t0, UART_DLL(a0)
    li      t0, 0
    sb      t0, UART_DLM(a0)

    # Clear DLAB, keep 8N1
    li      t0, 0x03            # DLAB=0, 8-bit word length
    sb      t0, UART_LCR(a0)

    # Enable FIFOs, clear TX/RX FIFOs
    li      t0, 0x07            # FIFO enable + reset TX + reset RX
    sb      t0, UART_FCR(a0)

    # Disable all interrupts (polling mode)
    li      t0, 0x00
    sb      t0, UART_IER(a0)

    # DTR + RTS
    li      t0, 0x03
    sb      t0, UART_MCR(a0)

    # =====================================================
    # 3. Print banner
    # =====================================================
    la      a1, banner_str
    jal     ra, print_string

    # =====================================================
    # 4. Echo loop
    # =====================================================
echo_loop:
    jal     ra, uart_getc       # Read one character -> a0

    # Check for Enter (CR = 0x0D)
    li      t0, 0x0D
    beq     a0, t0, handle_enter

    # Check for Backspace (0x08 or 0x7F)
    li      t0, 0x08
    beq     a0, t0, handle_backspace
    li      t0, 0x7F
    beq     a0, t0, handle_backspace

    # Normal character: echo it back
    jal     ra, uart_putc
    j       echo_loop

handle_enter:
    # Print "\r\n> "
    la      a1, prompt_str
    jal     ra, print_string
    j       echo_loop

handle_backspace:
    # Backspace: send BS + space + BS to erase character
    li      a0, 0x08
    jal     ra, uart_putc
    li      a0, 0x20
    jal     ra, uart_putc
    li      a0, 0x08
    jal     ra, uart_putc
    j       echo_loop

# =============================================================================
# UART subroutines
# =============================================================================

# uart_putc: Send byte in a0 via UART
# Clobbers: t0, t1
uart_putc:
    li      t1, UART_BASE
1:  lbu     t0, UART_LSR(t1)   # Read LSR
    andi    t0, t0, LSR_THRE   # Check THR Empty
    beqz    t0, 1b              # Spin until ready
    sb      a0, UART_THR(t1)   # Write byte
    ret

# uart_getc: Read byte from UART -> a0
# Clobbers: t0, t1
uart_getc:
    li      t1, UART_BASE
1:  lbu     t0, UART_LSR(t1)   # Read LSR
    andi    t0, t0, LSR_DR     # Check Data Ready
    beqz    t0, 1b              # Spin until data available
    lbu     a0, UART_THR(t1)   # Read received byte
    ret

# print_string: Print null-terminated string at a1
# Clobbers: a0, a1, t0, t1, ra (uses uart_putc)
print_string:
    addi    sp, sp, -4
    sw      ra, 0(sp)
1:  lbu     a0, 0(a1)
    beqz    a0, 2f              # Null terminator
    addi    a1, a1, 1
    jal     ra, uart_putc
    j       1b
2:  lw      ra, 0(sp)
    addi    sp, sp, 4
    ret

# =============================================================================
# Data section
# =============================================================================

.section .rodata

banner_str:
    .asciz  "\r\n\r\nRISC-V RV32IMAC SoC on Arty S7-25\r\nClock: 20MHz | RAM: 4KB | UART: 115200 8N1\r\n> "

prompt_str:
    .asciz  "\r\n> "
