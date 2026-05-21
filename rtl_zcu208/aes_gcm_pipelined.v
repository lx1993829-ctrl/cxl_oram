`timescale 1ns / 1ps
//============================================================================
// AES-GCM Pipelined - High Throughput with Automatic IV Rotation
//
// CHANGE FROM ORIGINAL:
//   The IV is no longer static across operations. After each GCM operation
//   completes, a 96-bit LFSR generates a new pseudo-random IV for the next
//   operation. Additionally, if the 32-bit CTR counter wraps to 0 during
//   a single (very large) operation, the module flags a counter-overflow
//   error (since continuing with a wrapped counter would repeat keystream).
//
// New ports:
//   iv_out       [95:0]  - Current/active IV (read back for decrypt pairing)
//   iv_updated           - Pulses high for one cycle when IV changes
//   counter_overflow     - Asserted if 32-bit counter wraps mid-operation
//
// IV lifecycle:
//   1. First start: external 'iv' input seeds the LFSR and is used directly
//   2. Each subsequent start (same key): LFSR-generated IV is used
//   3. If key changes: external 'iv' re-seeds the LFSR
//
// Throughput: 1 block/cycle after 12-cycle initial latency
//============================================================================

module aes_gcm_pipelined (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         enable,
    
    // Control interface
    input  wire         start,
    input  wire         encrypt,
    input  wire [127:0] key,
    input  wire [95:0]  iv,
    
    // Data interface
    input  wire [127:0] data_in,
    input  wire         data_valid,
    input  wire         data_last,
    input  wire [3:0]   data_bytes_valid,
    
    // Output interface
    output wire [127:0] data_out,
    output wire         data_out_valid,
    output wire         data_out_last,
    
    // Tag interface
    input  wire [127:0] tag_in,
    output reg  [127:0] tag_out,
    output reg          tag_valid,
    output reg          tag_match,
    
    // Status
    output wire         ready,
    output wire         busy,
    output wire         input_ready,
    
    // IV management (NEW)
    output wire [95:0]  iv_out,
    output reg          iv_updated,
    output reg          counter_overflow
);

    //=========================================================================
    // Parameters and Constants
    //=========================================================================
    localparam FIFO_DEPTH = 16;
    localparam FIFO_ADDR_BITS = 4;
    localparam GHASH_FIFO_DEPTH = 16;
    localparam GHASH_FIFO_ADDR_BITS = 4;
    
    localparam [3:0]
        ST_IDLE       = 4'd0,
        ST_INIT_H     = 4'd1,
        ST_INIT_EY0   = 4'd2,
        ST_WAIT_INIT  = 4'd3,
        ST_READY      = 4'd4,
        ST_PROCESSING = 4'd5,
        ST_DRAINING   = 4'd6,
        ST_FINAL_GHASH= 4'd7,
        ST_COMPUTE_TAG= 4'd8,
        ST_DONE       = 4'd9;
    
    reg [3:0] state;
    
    //=========================================================================
    // IV LFSR - 96-bit pseudo-random generator
    //
    // Polynomial: x^96 + x^10 + x^9 + x^6 + 1  (maximal-length)
    // Feedback: bit[95] ^ bit[9] ^ bit[8] ^ bit[5]
    //
    // We advance 96 steps per operation to ensure all bits are mixed.
    // A maximal-length LFSR with XOR feedback will NEVER reach the
    // all-zero state - the zero state is not part of the sequence.
    //
    // To avoid the zero-state trap, we also guard the seed: if the
    // external IV is all-zero, we substitute a fixed non-zero seed.
    //=========================================================================
    reg [95:0] iv_lfsr;
    reg [95:0] active_iv;         // IV being used for current operation
    reg        iv_seeded;         // LFSR has been seeded at least once
    
    // Advance LFSR by 96 steps in one cycle (unrolled combinational chain)
    // This uses a generate-style function to produce the final state
    wire [95:0] lfsr_advanced;
    
    // Advance function: 96 single-bit LFSR shifts chained combinationally
    function [95:0] advance_lfsr_96;
        input [95:0] state_in;
        reg [95:0] s;
        reg fb;
        integer step;
        begin
            s = state_in;
            for (step = 0; step < 96; step = step + 1) begin
                fb = s[95] ^ s[9] ^ s[8] ^ s[5];
                s = {s[94:0], fb};
            end
            advance_lfsr_96 = s;
        end
    endfunction
    
    assign lfsr_advanced = advance_lfsr_96(iv_lfsr);
    
    // Safe seed: if external IV is all-zero, use a fixed non-zero value
    wire [95:0] safe_seed = (iv == 96'b0) ? 96'hA5A5A5A5A5A5A5A5A5A5A5A5 : iv;
    
    // Output the active IV
    assign iv_out = active_iv;
    
    //=========================================================================
    // Internal Registers
    //=========================================================================
    
    reg [127:0] key_reg;
    reg [95:0]  iv_reg;
    reg         is_encrypt;
    reg [127:0] tag_in_reg;
    
    reg [127:0] h_key;
    reg [127:0] e_y0;
    reg         h_key_valid;
    reg         e_y0_valid;
    
    reg [31:0]  counter;
    
    reg [63:0]  aad_len_bits;
    reg [63:0]  data_len_bits;
    
    // H/E(Y0) caching
    reg         h_e_y0_valid;
    reg [127:0] last_key;
    // Pre-registered key comparison to break critical path
    reg         key_changed;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) key_changed <= 1'b1;
        else        key_changed <= (key != last_key);
    end
    reg [95:0]  last_iv;
    
    //=========================================================================
    // FIFO for Plaintext/Ciphertext (small - 16 entries, OK as registers)
    //=========================================================================
    reg [127:0] data_fifo [0:FIFO_DEPTH-1];
    reg [3:0]   bytes_fifo [0:FIFO_DEPTH-1];
    reg         last_fifo [0:FIFO_DEPTH-1];
    reg [FIFO_ADDR_BITS-1:0] fifo_wr_ptr;
    reg [FIFO_ADDR_BITS-1:0] fifo_rd_ptr;
    reg [FIFO_ADDR_BITS:0]   fifo_count;
    
    wire fifo_empty = (fifo_count == 0);
    wire fifo_full = (fifo_count == FIFO_DEPTH);
    
    //=========================================================================
    // GHASH Pending FIFO (16 entries - small enough for registers)
    //=========================================================================
    reg [127:0] ghash_pending_fifo [0:GHASH_FIFO_DEPTH-1];
    reg [GHASH_FIFO_ADDR_BITS-1:0] ghash_pending_wr_ptr;
    reg [GHASH_FIFO_ADDR_BITS-1:0] ghash_pending_rd_ptr;
    reg [GHASH_FIFO_ADDR_BITS:0]   ghash_pending_count;
    
    wire ghash_pending_empty = (ghash_pending_count == 0);
    wire ghash_pending_full = (ghash_pending_count == GHASH_FIFO_DEPTH);
    
    reg ghash_busy;
    
    //=========================================================================
    // Pipeline Tracking
    //=========================================================================
    reg [4:0] blocks_in_aes;
    reg [4:0] blocks_sent;
    reg [4:0] blocks_received;
    reg       last_block_sent;
    reg       last_block_received;
    
    //=========================================================================
    // AES Core Interface
    //=========================================================================
    reg  [127:0] aes_plaintext;
    reg          aes_valid_in;
    wire [127:0] aes_ciphertext;
    wire         aes_valid_out;
    
    reg         init_complete;
    reg [2:0]   init_outputs_remaining;
    
    //=========================================================================
    // GHASH Interface
    //=========================================================================
    reg  [127:0] ghash_data;
    reg          ghash_data_valid;
    wire [127:0] ghash_result;
    wire         ghash_result_valid;
    wire         ghash_ready;
    reg          ghash_start;
    
    //=========================================================================
    // Output Signals
    //=========================================================================
    reg [127:0] data_out_reg;
    reg         data_out_valid_reg;
    reg         data_out_last_reg;
    
    assign data_out = data_out_reg;
    assign data_out_valid = data_out_valid_reg;
    assign data_out_last = data_out_last_reg;
    
    assign ready = (state == ST_IDLE);
    assign busy = (state != ST_IDLE);
    assign input_ready = (state == ST_READY || state == ST_PROCESSING) && 
                         !fifo_full && 
                         !last_block_sent;
    
    //=========================================================================
    // Module Instantiations
    //=========================================================================
    
    aes128_encrypt_pipeline_fpga aes_core (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .plaintext(aes_plaintext),
        .key(key_reg),
        .valid_in(aes_valid_in),
        .ciphertext(aes_ciphertext),
        .valid_out(aes_valid_out)
    );
    
    ghash_single_cycle_fpga ghash_unit (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .start(ghash_start),
        .h_key(h_key),
        .data_in(ghash_data),
        .data_valid(ghash_data_valid),
        .ghash_out(ghash_result),
        .ghash_valid(ghash_result_valid),
        .ready(ghash_ready)
    );
    
    //=========================================================================
    // Mask function for partial blocks
    //=========================================================================
    function [127:0] get_mask;
        input [3:0] valid_bytes;
        begin
            case (valid_bytes)
                4'd0:  get_mask = 128'hFFFFFFFF_FFFFFFFF_FFFFFFFF_FFFFFFFF;
                4'd1:  get_mask = 128'hFF000000_00000000_00000000_00000000;
                4'd2:  get_mask = 128'hFFFF0000_00000000_00000000_00000000;
                4'd3:  get_mask = 128'hFFFFFF00_00000000_00000000_00000000;
                4'd4:  get_mask = 128'hFFFFFFFF_00000000_00000000_00000000;
                4'd5:  get_mask = 128'hFFFFFFFF_FF000000_00000000_00000000;
                4'd6:  get_mask = 128'hFFFFFFFF_FFFF0000_00000000_00000000;
                4'd7:  get_mask = 128'hFFFFFFFF_FFFFFF00_00000000_00000000;
                4'd8:  get_mask = 128'hFFFFFFFF_FFFFFFFF_00000000_00000000;
                4'd9:  get_mask = 128'hFFFFFFFF_FFFFFFFF_FF000000_00000000;
                4'd10: get_mask = 128'hFFFFFFFF_FFFFFFFF_FFFF0000_00000000;
                4'd11: get_mask = 128'hFFFFFFFF_FFFFFFFF_FFFFFF00_00000000;
                4'd12: get_mask = 128'hFFFFFFFF_FFFFFFFF_FFFFFFFF_00000000;
                4'd13: get_mask = 128'hFFFFFFFF_FFFFFFFF_FFFFFFFF_FF000000;
                4'd14: get_mask = 128'hFFFFFFFF_FFFFFFFF_FFFFFFFF_FFFF0000;
                4'd15: get_mask = 128'hFFFFFFFF_FFFFFFFF_FFFFFFFF_FFFFFF00;
                default: get_mask = 128'hFFFFFFFF_FFFFFFFF_FFFFFFFF_FFFFFFFF;
            endcase
        end
    endfunction
    
    //=========================================================================
    // Main State Machine
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            key_reg <= 128'b0;
            iv_reg <= 96'b0;
            is_encrypt <= 1'b1;
            tag_in_reg <= 128'b0;
            h_e_y0_valid <= 1'b0;
            last_key <= 128'b0;
            last_iv <= 96'b0;
            h_key <= 128'b0;
            e_y0 <= 128'b0;
            h_key_valid <= 1'b0;
            e_y0_valid <= 1'b0;
            counter <= 32'd2;
            aad_len_bits <= 64'b0;
            data_len_bits <= 64'b0;
            fifo_wr_ptr <= 0;
            fifo_rd_ptr <= 0;
            fifo_count <= 0;
            ghash_pending_wr_ptr <= 0;
            ghash_pending_rd_ptr <= 0;
            ghash_pending_count <= 0;
            ghash_busy <= 1'b0;
            blocks_in_aes <= 0;
            blocks_sent <= 0;
            blocks_received <= 0;
            last_block_sent <= 1'b0;
            last_block_received <= 1'b0;
            aes_plaintext <= 128'b0;
            aes_valid_in <= 1'b0;
            init_complete <= 1'b0;
            init_outputs_remaining <= 3'd7;
            ghash_data <= 128'b0;
            ghash_data_valid <= 1'b0;
            ghash_start <= 1'b0;
            data_out_reg <= 128'b0;
            data_out_valid_reg <= 1'b0;
            data_out_last_reg <= 1'b0;
            tag_out <= 128'b0;
            tag_valid <= 1'b0;
            tag_match <= 1'b0;
            // IV LFSR init
            iv_lfsr <= 96'b0;
            active_iv <= 96'b0;
            iv_seeded <= 1'b0;
            iv_updated <= 1'b0;
            counter_overflow <= 1'b0;
        end else if (enable) begin
            // Default: clear pulse signals
            if (ghash_start) $display("[%0t] [GCM-GHASH] ghash_start=1", $time);
            aes_valid_in <= 1'b0;
            ghash_data_valid <= 1'b0;
            ghash_start <= 1'b0;
            data_out_valid_reg <= 1'b0;
            data_out_last_reg <= 1'b0;
            tag_valid <= 1'b0;
            iv_updated <= 1'b0;
            
            case (state)

                //=============================================================
                // IDLE: Determine IV and start initialization
                //=============================================================
                ST_IDLE: begin
                    if (start) begin
                        // Capture inputs
                        is_encrypt <= encrypt;
                        tag_in_reg <= tag_in;
                        
                        // Reset operational state
                        counter <= 32'd2;
                        aad_len_bits <= 64'b0;
                        data_len_bits <= 64'b0;
                        fifo_wr_ptr <= 0;
                        fifo_rd_ptr <= 0;
                        fifo_count <= 0;
                        ghash_pending_wr_ptr <= 0;
                        ghash_pending_rd_ptr <= 0;
                        ghash_pending_count <= 0;
                        ghash_busy <= 1'b0;
                        blocks_in_aes <= 0;
                        blocks_sent <= 0;
                        blocks_received <= 0;
                        last_block_sent <= 1'b0;
                        last_block_received <= 1'b0;
                        init_complete <= 1'b0;
                        counter_overflow <= 1'b0;
                        
                        //=====================================================
                        // IV Selection Logic
                        //=====================================================
                        // For ENCRYPT:
                        //   - First operation or key change: use external IV,
                        //     seed LFSR with it
                        //   - Subsequent operations (same key): use LFSR IV
                        //
                        // For DECRYPT:
                        //   - Always use the externally-provided IV
                        //     (must match what encryptor used)
                        //=====================================================
                        
                        if (!encrypt) begin
                            // DECRYPT: always use the provided IV
                            key_reg <= key;
                            iv_reg <= iv;
                            active_iv <= iv;
                            
                            // Check if we need re-init
                            if (!h_e_y0_valid || key_changed || (iv != last_iv)) begin
                                last_key <= key;
                                last_iv <= iv;
                                h_key_valid <= 1'b0;
                                e_y0_valid <= 1'b0;
                                h_e_y0_valid <= 1'b0;
                                init_outputs_remaining <= 3'd7;
                                state <= ST_INIT_H;
                                $display("[%0t] [AES-GCM] DECRYPT: using provided IV=0x%024h", $time, iv);
                            end else begin
                                key_reg <= key;
                                iv_reg <= iv;
                                init_complete <= 1'b1;
                                ghash_start <= 1'b1;
                                state <= ST_READY;
                                $display("[%0t] [AES-GCM] DECRYPT: reusing cached H/E(Y0), IV=0x%024h", $time, iv);
                            end
                        end
                        else begin
                            // ENCRYPT: determine IV
                            if (!iv_seeded || key_changed) begin
                                // First operation or key changed: seed LFSR with safe IV
                                iv_lfsr <= safe_seed;
                                active_iv <= safe_seed;
                                iv_reg <= safe_seed;
                                iv_seeded <= 1'b1;
                                $display("[%0t] [AES-GCM] ENCRYPT: seeding LFSR with IV=0x%024h", $time, safe_seed);
                            end else begin
                                // Subsequent operation, same key: use LFSR-generated IV
                                active_iv <= iv_lfsr;
                                iv_reg <= iv_lfsr;
                                $display("[%0t] [AES-GCM] ENCRYPT: using LFSR IV=0x%024h", $time, iv_lfsr);
                            end
                            
                            key_reg <= key;
                            
                            // Always need new H and E(Y0) since IV changed
                            // (H only depends on key, but E(Y0) = AES(K, IV||1))
                            // Optimization: if only IV changed (not key), we can
                            // reuse H but must recompute E(Y0).
                            // For simplicity & correctness, recompute both.
                            last_key <= key;
                            if (!iv_seeded || key_changed) begin
                                last_iv <= safe_seed;
                            end else begin
                                last_iv <= iv_lfsr;
                            end
                            h_key_valid <= 1'b0;
                            e_y0_valid <= 1'b0;
                            h_e_y0_valid <= 1'b0;
                            init_outputs_remaining <= 3'd7;
                            state <= ST_INIT_H;
                        end
                    end
                end
                
                //=============================================================
                ST_INIT_H: begin
                    // Round key registers need 3 clock cycles to settle after key_reg changes.
                    // Countdown: 5=wait, 4=wait, 3=wait, 2=send H, 1=send E(Y0)
                    if (init_outputs_remaining > 3'd2) begin
                        // Wait cycles: round key combinational path settling
                        init_outputs_remaining <= init_outputs_remaining - 1'b1;
                    end else if (init_outputs_remaining == 3'd2) begin
                        // Send H = AES(K, 0). Round keys are valid now.
                        aes_plaintext <= 128'b0;
                        aes_valid_in <= 1'b1;
                        init_outputs_remaining <= 3'd1;
                    end else begin
                        // Send E(Y0) = AES(K, IV||1)
                        aes_plaintext <= {iv_reg, 32'h00000001};
                        aes_valid_in <= 1'b1;
                        state <= ST_WAIT_INIT;
                    end
                end
                
                //=============================================================
                ST_WAIT_INIT: begin
                    if (aes_valid_out) begin
                        if (!h_key_valid) begin
                            h_key <= aes_ciphertext;
                            h_key_valid <= 1'b1;
                            $display("[%0t] [AES-GCM] H key computed: 0x%032h", $time, aes_ciphertext);
                        end else if (!e_y0_valid) begin
                            e_y0 <= aes_ciphertext;
                            e_y0_valid <= 1'b1;
                            init_complete <= 1'b1;
                            $display("[%0t] [AES-GCM] E(Y0) computed: 0x%032h", $time, aes_ciphertext);
                        end
                    end
                    
                    if (h_key_valid && e_y0_valid) begin
                        h_e_y0_valid <= 1'b1;
                        $display("[%0t] [AES-GCM] Init complete, IV=0x%024h", $time, iv_reg);
                        ghash_start <= 1'b1;
                        state <= ST_READY;
                    end
                end
                
                //=============================================================
                ST_READY: begin
                    if (data_valid && !fifo_full) begin
                        data_fifo[fifo_wr_ptr] <= data_in;
                        bytes_fifo[fifo_wr_ptr] <= data_bytes_valid;
                        last_fifo[fifo_wr_ptr] <= data_last;
                        fifo_wr_ptr <= fifo_wr_ptr + 1;
                        fifo_count <= fifo_count + 1;
                        
                        // Counter overflow detection
                        if (counter == 32'hFFFFFFFF) begin
                            counter_overflow <= 1'b1;
                            $display("[%0t] [AES-GCM] WARNING: 32-bit counter overflow!", $time);
                        end
                        
                        aes_plaintext <= {iv_reg, counter};
                        aes_valid_in <= 1'b1;
                        counter <= counter + 1;
                        blocks_in_aes <= blocks_in_aes + 1;
                        blocks_sent <= blocks_sent + 1;
                        
                        if (data_bytes_valid == 4'd0)
                            data_len_bits <= data_len_bits + 64'd128;
                        else
                            data_len_bits <= data_len_bits + {57'b0, data_bytes_valid, 3'b0};
                        
                        if (data_last)
                            last_block_sent <= 1'b1;
                        
                        state <= ST_PROCESSING;
                    end else if (!data_valid && data_last) begin
                        state <= ST_FINAL_GHASH;
                    end
                end
                
                //=============================================================
                ST_PROCESSING: begin
                    // INPUT SIDE
                    if (data_valid && !fifo_full && !last_block_sent) begin
                        data_fifo[fifo_wr_ptr] <= data_in;
                        bytes_fifo[fifo_wr_ptr] <= data_bytes_valid;
                        last_fifo[fifo_wr_ptr] <= data_last;
                        fifo_wr_ptr <= fifo_wr_ptr + 1;
                        fifo_count <= fifo_count + 1;
                        
                        // Counter overflow detection
                        if (counter == 32'hFFFFFFFF) begin
                            counter_overflow <= 1'b1;
                            $display("[%0t] [AES-GCM] WARNING: 32-bit counter overflow!", $time);
                        end
                        
                        aes_plaintext <= {iv_reg, counter};
                        aes_valid_in <= 1'b1;
                        counter <= counter + 1;
                        blocks_in_aes <= blocks_in_aes + 1;
                        blocks_sent <= blocks_sent + 1;
                        
                        if (data_bytes_valid == 4'd0)
                            data_len_bits <= data_len_bits + 64'd128;
                        else
                            data_len_bits <= data_len_bits + {57'b0, data_bytes_valid, 3'b0};
                        
                        if (data_last)
                            last_block_sent <= 1'b1;
                    end
                    
                    // OUTPUT SIDE
                    if (aes_valid_out && init_complete) begin
                        if (is_encrypt) begin
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0)
                                data_out_reg <= (data_fifo[fifo_rd_ptr] ^ aes_ciphertext) & 
                                               get_mask(bytes_fifo[fifo_rd_ptr]);
                            else
                                data_out_reg <= data_fifo[fifo_rd_ptr] ^ aes_ciphertext;
                        end else begin
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0)
                                data_out_reg <= (data_fifo[fifo_rd_ptr] ^ aes_ciphertext) & 
                                               get_mask(bytes_fifo[fifo_rd_ptr]);
                            else
                                data_out_reg <= data_fifo[fifo_rd_ptr] ^ aes_ciphertext;
                        end
                        
                        data_out_valid_reg <= 1'b1;
                        data_out_last_reg <= last_fifo[fifo_rd_ptr];
                        
                        // Queue for GHASH
                        if (is_encrypt) begin
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0)
                                ghash_pending_fifo[ghash_pending_wr_ptr] <=
                                    (data_fifo[fifo_rd_ptr] ^ aes_ciphertext) & get_mask(bytes_fifo[fifo_rd_ptr]);
                            else
                                ghash_pending_fifo[ghash_pending_wr_ptr] <=
                                    data_fifo[fifo_rd_ptr] ^ aes_ciphertext;
                        end else begin
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0)
                                ghash_pending_fifo[ghash_pending_wr_ptr] <=
                                    data_fifo[fifo_rd_ptr] & get_mask(bytes_fifo[fifo_rd_ptr]);
                            else
                                ghash_pending_fifo[ghash_pending_wr_ptr] <= data_fifo[fifo_rd_ptr];
                        end
                        ghash_pending_wr_ptr <= ghash_pending_wr_ptr + 1;
                        ghash_pending_count <= ghash_pending_count + 1;
                        
                        fifo_rd_ptr <= fifo_rd_ptr + 1;
                        fifo_count <= fifo_count - 1;
                        blocks_in_aes <= blocks_in_aes - 1;
                        blocks_received <= blocks_received + 1;
                        
                        if (last_fifo[fifo_rd_ptr])
                            last_block_received <= 1'b1;
                    end
                    
                    // GHASH Processing - only feed when GHASH is ready (2-cycle pipeline)
                    if (!ghash_pending_empty && ghash_ready) begin
                        ghash_data <= ghash_pending_fifo[ghash_pending_rd_ptr];
                        ghash_data_valid <= 1'b1;
                        ghash_pending_rd_ptr <= ghash_pending_rd_ptr + 1;
                        ghash_pending_count <= ghash_pending_count - 1;
                    end
                    
                    if (ghash_result_valid)
                        ghash_busy <= 1'b0;
                    
                    // Simultaneous adjustments
                    if (aes_valid_out && init_complete &&
                        !ghash_pending_empty && ghash_ready)
                        ghash_pending_count <= ghash_pending_count;
                    
                    if (data_valid && !fifo_full && !last_block_sent &&
                        aes_valid_out && init_complete)
                        fifo_count <= fifo_count;
                    
                    // State transitions
                    if (last_block_sent && last_block_received && ghash_pending_empty)
                        state <= ST_FINAL_GHASH;
                    else if (last_block_sent && !aes_valid_out)
                        state <= ST_DRAINING;
                end
                
                //=============================================================
                ST_DRAINING: begin
                    if (aes_valid_out && init_complete) begin
                        if (is_encrypt) begin
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0)
                                data_out_reg <= (data_fifo[fifo_rd_ptr] ^ aes_ciphertext) & 
                                               get_mask(bytes_fifo[fifo_rd_ptr]);
                            else
                                data_out_reg <= data_fifo[fifo_rd_ptr] ^ aes_ciphertext;
                        end else begin
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0)
                                data_out_reg <= (data_fifo[fifo_rd_ptr] ^ aes_ciphertext) & 
                                               get_mask(bytes_fifo[fifo_rd_ptr]);
                            else
                                data_out_reg <= data_fifo[fifo_rd_ptr] ^ aes_ciphertext;
                        end
                        
                        data_out_valid_reg <= 1'b1;
                        data_out_last_reg <= last_fifo[fifo_rd_ptr];
                        
                        if (is_encrypt) begin
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0)
                                ghash_pending_fifo[ghash_pending_wr_ptr] <=
                                    (data_fifo[fifo_rd_ptr] ^ aes_ciphertext) & get_mask(bytes_fifo[fifo_rd_ptr]);
                            else
                                ghash_pending_fifo[ghash_pending_wr_ptr] <=
                                    data_fifo[fifo_rd_ptr] ^ aes_ciphertext;
                        end else begin
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0)
                                ghash_pending_fifo[ghash_pending_wr_ptr] <=
                                    data_fifo[fifo_rd_ptr] & get_mask(bytes_fifo[fifo_rd_ptr]);
                            else
                                ghash_pending_fifo[ghash_pending_wr_ptr] <= data_fifo[fifo_rd_ptr];
                        end
                        ghash_pending_wr_ptr <= ghash_pending_wr_ptr + 1;
                        ghash_pending_count <= ghash_pending_count + 1;
                        
                        fifo_rd_ptr <= fifo_rd_ptr + 1;
                        fifo_count <= fifo_count - 1;
                        blocks_in_aes <= blocks_in_aes - 1;
                        blocks_received <= blocks_received + 1;
                        
                        if (last_fifo[fifo_rd_ptr])
                            last_block_received <= 1'b1;
                    end
                    
                    if (!ghash_pending_empty && ghash_ready) begin
                        ghash_data <= ghash_pending_fifo[ghash_pending_rd_ptr];
                        ghash_data_valid <= 1'b1;
                        ghash_pending_rd_ptr <= ghash_pending_rd_ptr + 1;
                        ghash_pending_count <= ghash_pending_count - 1;
                    end
                    
                    if (ghash_result_valid)
                        ghash_busy <= 1'b0;
                    
                    if (aes_valid_out && init_complete &&
                        !ghash_pending_empty && ghash_ready)
                        ghash_pending_count <= ghash_pending_count;
                    
                    if (last_block_received && ghash_pending_empty)
                        state <= ST_FINAL_GHASH;
                end
                
                //=============================================================
                ST_FINAL_GHASH: begin
                    if (ghash_ready) begin
                        ghash_data <= {aad_len_bits, data_len_bits};
                        ghash_data_valid <= 1'b1;
                        state <= ST_COMPUTE_TAG;
                    end
                    
                    if (ghash_result_valid)
                        ghash_busy <= 1'b0;
                end
                
                //=============================================================
                ST_COMPUTE_TAG: begin
                    if (ghash_result_valid) begin
                        tag_out <= ghash_result ^ e_y0;
                        tag_valid <= 1'b1;
                        
                        if (!is_encrypt)
                            tag_match <= ((ghash_result ^ e_y0) == tag_in);
                        else
                            tag_match <= 1'b1;
                        
                        state <= ST_DONE;
                    end
                end
                
                //=============================================================
                // DONE: Advance LFSR for next IV (encrypt only)
                //=============================================================
                ST_DONE: begin
                    // Advance LFSR after encrypt operations
                    // so next encrypt gets a different IV
                    if (is_encrypt) begin
                        iv_lfsr <= lfsr_advanced;
                        iv_updated <= 1'b1;
                        $display("[%0t] [AES-GCM] IV rotated: next LFSR IV=0x%024h",
                                 $time, lfsr_advanced);
                    end
                    
                    // Invalidate H/E(Y0) cache since IV will change next time
                    // (H depends only on key so could be kept, but E(Y0) depends
                    //  on IV, so we must recompute. Invalidate both for safety.)
                    if (is_encrypt)
                        h_e_y0_valid <= 1'b0;
                    
                    state <= ST_IDLE;
                end
                
                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule