// =============================================================================
// 16550-Compatible UART
// =============================================================================
// Simple UART implementation compatible with the NS16550 register interface.
// Linux uses this as the standard serial console.
//
// Register Map (active when DLAB=0):
//   0x0: RBR (read) / THR (write) - Receive/Transmit buffer
//   0x1: IER - Interrupt Enable Register
//   0x2: IIR (read) / FCR (write) - Interrupt ID / FIFO Control
//   0x3: LCR - Line Control Register
//   0x4: MCR - Modem Control Register
//   0x5: LSR - Line Status Register
//   0x6: MSR - Modem Status Register
//   0x7: SCR - Scratch Register
//
// Register Map (active when DLAB=1):
//   0x0: DLL - Divisor Latch Low
//   0x1: DLM - Divisor Latch High
// =============================================================================

module uart #(
    parameter BASE_ADDR = 32'h10000000,
    parameter FIFO_DEPTH = 16
) (
    input         clk,
    input         rst,
    
    // Memory-mapped interface
    input  [31:0] addr,
    input  [31:0] wdata,
    input  [3:0]  wstrb,
    input         read_en,
    output [31:0] rdata,
    output        addr_valid,
    
    // Interrupt output
    output        uart_irq,
    
    // External serial interface
    output reg    tx_out,
    input         rx_in,
    
    // Simulation interface
    output reg [7:0]  sim_tx_data,
    output reg        sim_tx_valid,
    input  [7:0]      sim_rx_data,
    input             sim_rx_valid
);

    // =========================================================================
    // Address Decoding
    // =========================================================================
    // Support 32-bit register spacing (mmio32): THR=0x00, LSR=0x14, etc.
    wire in_range = (addr >= BASE_ADDR) && (addr < BASE_ADDR + 32'h20);
    assign addr_valid = in_range;
    wire [2:0] reg_addr = addr[4:2];  // 32-bit aligned: bits [4:2] select register
    wire do_write_level = in_range && (wstrb != 4'b0);
    wire do_read = in_range && read_en;
    
    // Edge detection for writes to prevent double writes
    reg do_write_prev;
    wire do_write = do_write_level && !do_write_prev;  // Rising edge only
    
    // =========================================================================
    // Registers
    // =========================================================================
    reg [7:0] dll;          // Divisor Latch Low
    reg [7:0] dlm;          // Divisor Latch High
    reg [7:0] lcr;          // Line Control Register
    reg [7:0] ier;          // Interrupt Enable Register
    reg [7:0] fcr;          // FIFO Control Register
    reg [7:0] mcr;          // Modem Control Register
    reg [7:0] scr;          // Scratch Register
    reg       overrun_error;
    
    wire dlab = lcr[7];     // Divisor Latch Access Bit
    
    // =========================================================================
    // TX FIFO
    // =========================================================================
    reg [7:0] tx_fifo [0:FIFO_DEPTH-1];
    reg [4:0] tx_head, tx_tail, tx_count;
    
    wire tx_fifo_empty = (tx_count == 0);
    wire tx_fifo_full  = (tx_count == FIFO_DEPTH);
    
    // =========================================================================
    // RX FIFO  
    // =========================================================================
    reg [7:0] rx_fifo [0:FIFO_DEPTH-1];
    reg [4:0] rx_head, rx_tail, rx_count;
    
    wire rx_fifo_empty = (rx_count == 0);
    wire rx_fifo_full  = (rx_count == FIFO_DEPTH);
    
    // =========================================================================
    // Line Status Register
    // =========================================================================
    wire [7:0] lsr = {
        1'b0,                    // Bit 7: Error in RCVR FIFO
        tx_fifo_empty,           // Bit 6: Transmitter Empty
        !tx_fifo_full,           // Bit 5: THR Empty
        1'b0,                    // Bit 4: Break Interrupt
        1'b0,                    // Bit 3: Framing Error
        1'b0,                    // Bit 2: Parity Error
        overrun_error,           // Bit 1: Overrun Error
        !rx_fifo_empty           // Bit 0: Data Ready
    };
    
    // =========================================================================
    // Interrupt Logic
    // =========================================================================
    wire int_rx_avail = ier[0] && !rx_fifo_empty;
    wire int_tx_empty = ier[1] && tx_fifo_empty;
    wire int_rx_status = ier[2] && overrun_error;
    
    assign uart_irq = int_rx_avail || int_tx_empty || int_rx_status;
    
    // IIR with priority encoding
    wire [7:0] iir;
    assign iir[7:6] = fcr[0] ? 2'b11 : 2'b00;  // FIFO enabled flags
    assign iir[5:4] = 2'b00;
    assign iir[3:0] = int_rx_status ? 4'b0110 :  // Receiver Line Status
                      int_rx_avail  ? 4'b0100 :  // RX Data Available
                      int_tx_empty  ? 4'b0010 :  // THR Empty
                                      4'b0001;   // No interrupt
    
    // =========================================================================
    // Register Read (Combinational)
    // =========================================================================
    // Return data in the correct byte lane based on address[1:0]
    // This allows the CPU's standard byte extraction logic to work properly
    reg [31:0] rdata_reg;
    assign rdata = rdata_reg;

    // Get the raw register value
    reg [7:0] reg_value;
    always @(*) begin
        case (reg_addr)
            3'd0: reg_value = dlab ? dll : rx_fifo[rx_head];
            3'd1: reg_value = dlab ? dlm : ier;
            3'd2: reg_value = iir;
            3'd3: reg_value = lcr;
            3'd4: reg_value = mcr;
            3'd5: reg_value = lsr;
            3'd6: reg_value = 8'h0;  // MSR - no modem
            3'd7: reg_value = scr;
            default: reg_value = 8'h0;
        endcase
    end

    // Place data in correct byte lane based on address[1:0]
    always @(*) begin
        rdata_reg = 32'h0;
        if (do_read) begin
            case (addr[1:0])
                2'b00: rdata_reg = {24'h0, reg_value};
                2'b01: rdata_reg = {16'h0, reg_value, 8'h0};
                2'b10: rdata_reg = {8'h0, reg_value, 16'h0};
                2'b11: rdata_reg = {reg_value, 24'h0};
            endcase
        end
    end
    
    // =========================================================================
    // Hardware TX/RX State Machine Declarations
    // =========================================================================
    // Declared unconditionally so both sim and synthesis paths compile.
    // In simulation, these regs are unused (sim uses instant FIFO drain).

    // Baud divisor from DLL/DLM registers: baud = clk_freq / (16 * divisor)
    wire [15:0] baud_divisor = {dlm, dll};

    // TX state machine
    localparam TX_IDLE  = 2'd0;
    localparam TX_START = 2'd1;
    localparam TX_DATA  = 2'd2;
    localparam TX_STOP  = 2'd3;

    reg [1:0]  tx_state;
    reg [7:0]  tx_shift;
    reg [2:0]  tx_bit_cnt;
    reg [19:0] tx_baud_cnt;
    reg [7:0]  sim_tx_byte;  // Saved byte for sim output in HW TX mode

    // RX state machine
    localparam RX_IDLE  = 2'd0;
    localparam RX_START = 2'd1;
    localparam RX_DATA  = 2'd2;
    localparam RX_STOP  = 2'd3;

    reg [1:0]  rx_state;
    reg [7:0]  rx_shift;
    reg [2:0]  rx_bit_cnt;
    reg [19:0] rx_baud_cnt;
    reg        rx_done_tick;

    // =========================================================================
    // Sequential Logic
    // =========================================================================
    integer i;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            dll <= 8'h01;
            dlm <= 8'h00;
            lcr <= 8'h00;
            ier <= 8'h00;
            fcr <= 8'h00;
            mcr <= 8'h00;
            scr <= 8'h00;
            overrun_error <= 1'b0;
            
            tx_head <= 0;
            tx_tail <= 0;
            tx_count <= 0;
            
            rx_head <= 0;
            rx_tail <= 0;
            rx_count <= 0;
            
            sim_tx_valid <= 1'b0;
            sim_tx_data <= 8'h0;
            sim_tx_byte <= 8'h0;
            tx_out <= 1'b1;
            tx_state <= TX_IDLE;
            tx_shift <= 8'h0;
            tx_bit_cnt <= 0;
            tx_baud_cnt <= 0;
            do_write_prev <= 1'b0;
            
            for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
                tx_fifo[i] <= 8'h0;
                rx_fifo[i] <= 8'h0;
            end
        end else begin
            // Default: clear single-cycle signals
            sim_tx_valid <= 1'b0;
            
            // Update edge detection register
            do_write_prev <= do_write_level;
            
            // =====================================================
            // Handle Register Writes
            // =====================================================
            if (do_write) begin
                case (reg_addr)
                    3'd0: begin
                        if (dlab) begin
                            dll <= wdata[7:0];
                        end else if (!tx_fifo_full) begin
                            // Write to THR
                            tx_fifo[tx_tail] <= wdata[7:0];
                            tx_tail <= (tx_tail + 1) % FIFO_DEPTH;
                            tx_count <= tx_count + 1;
                        end
                    end
                    3'd1: begin
                        if (dlab)
                            dlm <= wdata[7:0];
                        else
                            ier <= wdata[7:0];
                    end
                    3'd2: begin
                        fcr <= wdata[7:0];
                        if (wdata[1]) begin  // Reset RX FIFO
                            rx_head <= 0;
                            rx_tail <= 0;
                            rx_count <= 0;
                        end
                        if (wdata[2]) begin  // Reset TX FIFO
                            tx_head <= 0;
                            tx_tail <= 0;
                            tx_count <= 0;
                        end
                    end
                    3'd3: lcr <= wdata[7:0];
                    3'd4: mcr <= wdata[7:0];
                    3'd7: scr <= wdata[7:0];
                    default: ;
                endcase
            end
            
            // =====================================================
            // Handle Read Side Effects
            // =====================================================
            if (do_read) begin
                case (reg_addr)
                    3'd0: begin
                        // Reading RBR pops from RX FIFO
                        if (!dlab && !rx_fifo_empty) begin
                            rx_head <= (rx_head + 1) % FIFO_DEPTH;
                            rx_count <= rx_count - 1;
                        end
                    end
                    3'd5: begin
                        // Reading LSR clears overrun error
                        overrun_error <= 1'b0;
                    end
                    default: ;
                endcase
            end
            
            // =====================================================
            // TX: Transmit from FIFO
            // =====================================================
            `ifdef FAST_UART
            // Fast simulation: instant single-cycle transmit
            if (!tx_fifo_empty && !(do_write && reg_addr == 3'd0 && !dlab)) begin
                sim_tx_data <= tx_fifo[tx_head];
                sim_tx_valid <= 1'b1;
                tx_head <= (tx_head + 1) % FIFO_DEPTH;
                tx_count <= tx_count - 1;
            end
            `else
            // Hardware: TX shift register pulls from FIFO when idle
            if (tx_state == TX_IDLE && !tx_fifo_empty && !(do_write && reg_addr == 3'd0 && !dlab)) begin
                tx_shift <= tx_fifo[tx_head];
                sim_tx_byte <= tx_fifo[tx_head];
                tx_head <= (tx_head + 1) % FIFO_DEPTH;
                tx_count <= tx_count - 1;
                tx_state <= TX_START;
                tx_bit_cnt <= 0;
                tx_baud_cnt <= 0;
            end
            `endif

            // =====================================================
            // RX: Receive into FIFO
            // =====================================================
            `ifdef FAST_UART
            if (sim_rx_valid) begin
                if (!rx_fifo_full) begin
                    rx_fifo[rx_tail] <= sim_rx_data;
                    rx_tail <= (rx_tail + 1) % FIFO_DEPTH;
                    rx_count <= rx_count + 1;
                end else begin
                    overrun_error <= 1'b1;
                end
            end
            `else
            // Hardware: RX deserializer pushes to FIFO when byte complete
            if (rx_done_tick) begin
                if (!rx_fifo_full) begin
                    rx_fifo[rx_tail] <= rx_shift;
                    rx_tail <= (rx_tail + 1) % FIFO_DEPTH;
                    rx_count <= rx_count + 1;
                end else begin
                    overrun_error <= 1'b1;
                end
            end
            `endif

            // =====================================================
            // TX State Machine (HW baud-rate timing)
            // =====================================================
            `ifndef FAST_UART
            case (tx_state)
                TX_START: begin
                    // Send start bit (low) for one baud period
                    tx_out <= 1'b0;
                    if (tx_baud_cnt >= baud_full) begin
                        tx_baud_cnt <= 0;
                        tx_state <= TX_DATA;
                        tx_bit_cnt <= 0;
                    end else begin
                        tx_baud_cnt <= tx_baud_cnt + 1;
                    end
                end
                TX_DATA: begin
                    tx_out <= tx_shift[0];
                    if (tx_baud_cnt >= baud_full) begin
                        tx_baud_cnt <= 0;
                        tx_shift <= {1'b0, tx_shift[7:1]};
                        if (tx_bit_cnt == 7) begin
                            tx_state <= TX_STOP;
                        end else begin
                            tx_bit_cnt <= tx_bit_cnt + 1;
                        end
                    end else begin
                        tx_baud_cnt <= tx_baud_cnt + 1;
                    end
                end
                TX_STOP: begin
                    tx_out <= 1'b1;
                    if (tx_baud_cnt >= baud_full) begin
                        tx_baud_cnt <= 0;
                        tx_state <= TX_IDLE;
                        sim_tx_data <= sim_tx_byte;
                        sim_tx_valid <= 1'b1;
                    end else begin
                        tx_baud_cnt <= tx_baud_cnt + 1;
                    end
                end
                default: begin
                    tx_out <= 1'b1;  // Idle line high
                end
            endcase
            `endif
        end
    end

    // =========================================================================
    // Hardware RX (when not using fast UART)
    // =========================================================================
    `ifndef FAST_UART

    // Baud tick counts
    // Full period = 16 * divisor clock cycles per bit
    // Half period = 8 * divisor clock cycles (for mid-bit sampling)
    wire [19:0] baud_full = {baud_divisor, 4'b0000} - 1;  // 16 * divisor - 1
    wire [19:0] baud_half = {1'b0, baud_divisor, 3'b000} - 1;  // 8 * divisor - 1

    // --- RX State Machine ---
    // Synchronize rx_in to clock domain (2-stage)
    reg rx_sync1, rx_sync2;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
        end else begin
            rx_sync1 <= rx_in;
            rx_sync2 <= rx_sync1;
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rx_state <= RX_IDLE;
            rx_shift <= 8'h0;
            rx_bit_cnt <= 0;
            rx_baud_cnt <= 0;
            rx_done_tick <= 1'b0;
        end else begin
            rx_done_tick <= 1'b0;
            case (rx_state)
                RX_IDLE: begin
                    if (rx_sync2 == 1'b0) begin
                        // Possible start bit detected
                        rx_state <= RX_START;
                        rx_baud_cnt <= 0;
                    end
                end
                RX_START: begin
                    // Sample at mid-point of start bit (half period)
                    if (rx_baud_cnt >= baud_half) begin
                        rx_baud_cnt <= 0;
                        if (rx_sync2 == 1'b0) begin
                            // Valid start bit
                            rx_state <= RX_DATA;
                            rx_bit_cnt <= 0;
                        end else begin
                            // False start
                            rx_state <= RX_IDLE;
                        end
                    end else begin
                        rx_baud_cnt <= rx_baud_cnt + 1;
                    end
                end
                RX_DATA: begin
                    // Sample at mid-point of each data bit (full period between samples)
                    if (rx_baud_cnt >= baud_full) begin
                        rx_baud_cnt <= 0;
                        rx_shift <= {rx_sync2, rx_shift[7:1]};  // LSB first
                        if (rx_bit_cnt == 7) begin
                            rx_state <= RX_STOP;
                        end else begin
                            rx_bit_cnt <= rx_bit_cnt + 1;
                        end
                    end else begin
                        rx_baud_cnt <= rx_baud_cnt + 1;
                    end
                end
                RX_STOP: begin
                    // Wait for stop bit
                    if (rx_baud_cnt >= baud_full) begin
                        rx_baud_cnt <= 0;
                        rx_done_tick <= 1'b1;  // Byte received
                        rx_state <= RX_IDLE;
                    end else begin
                        rx_baud_cnt <= rx_baud_cnt + 1;
                    end
                end
            endcase
        end
    end

    `endif // FAST_UART

endmodule