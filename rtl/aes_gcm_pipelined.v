`timescale 1ns / 1ps
//============================================================================
// AES-GCM Pipelined - High Throughput Implementation
// 
// This module fully utilizes the 12-stage AES pipeline by:
// 1. Feeding new blocks every cycle (not waiting for output)
// 2. Using FIFOs to match AES latency
// 3. Processing GHASH in parallel with AES
//
// Throughput: 1 block/cycle after 12-cycle initial latency
//            = 12.8 Gbps @ 100MHz
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
    
    // Data interface - can accept 1 block per cycle
    input  wire [127:0] data_in,
    input  wire         data_valid,
    input  wire         data_last,
    input  wire [3:0]   data_bytes_valid,  // 0 = 16 bytes
    
    // Output interface - outputs 1 block per cycle (after latency)
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
    output wire         input_ready  // Can accept new data this cycle
);

    //=========================================================================
    // Parameters and Constants
    //=========================================================================
    localparam FIFO_DEPTH = 16;  // Must be >= AES pipeline depth (12)
    localparam FIFO_ADDR_BITS = 4;
    localparam GHASH_FIFO_DEPTH = 128;  // Large enough for big bursts
    localparam GHASH_FIFO_ADDR_BITS = 7;
    
    localparam [3:0]
        ST_IDLE       = 4'd0,
        ST_INIT_H     = 4'd1,   // Computing H = AES(K, 0)
        ST_INIT_EY0   = 4'd2,   // Computing E(Y0) = AES(K, IV||1)
        ST_WAIT_INIT  = 4'd3,   // Waiting for H and E(Y0)
        ST_READY      = 4'd4,   // Ready to accept data
        ST_PROCESSING = 4'd5,   // Processing data blocks
        ST_DRAINING   = 4'd6,   // Draining pipeline (no more input)
        ST_FINAL_GHASH= 4'd7,   // Final GHASH with length block
        ST_COMPUTE_TAG= 4'd8,   // Computing final tag
        ST_DONE       = 4'd9;
    
    reg [3:0] state;
    
    //=========================================================================
    // Internal Registers
    //=========================================================================
    
    // Key and IV registers
    reg [127:0] key_reg;
    reg [95:0]  iv_reg;
    reg         is_encrypt;
    reg [127:0] tag_in_reg;
    
    // H key and E(Y0) for tag computation
    reg [127:0] h_key;
    reg [127:0] e_y0;
    reg         h_key_valid;
    reg         e_y0_valid;
    
    // Counter for CTR mode
    reg [31:0]  counter;
    
    // Length tracking (in bits)
    reg [63:0]  aad_len_bits;
    reg [63:0]  data_len_bits;
    
    //=========================================================================
    // FIFO for Plaintext/Ciphertext (to match AES pipeline latency)
    //=========================================================================
    reg [127:0] data_fifo [0:FIFO_DEPTH-1];
    reg [3:0]   bytes_fifo [0:FIFO_DEPTH-1];
    reg         last_fifo [0:FIFO_DEPTH-1];
    reg [FIFO_ADDR_BITS-1:0] fifo_wr_ptr;
    reg [FIFO_ADDR_BITS-1:0] fifo_rd_ptr;
    reg [FIFO_ADDR_BITS:0]   fifo_count;  // One extra bit for full detection
    
    wire fifo_empty = (fifo_count == 0);
    wire fifo_full = (fifo_count == FIFO_DEPTH);
    
    //=========================================================================
    // GHASH Pending FIFO (to handle AES outputs faster than GHASH can process)
    //=========================================================================
    reg [127:0] ghash_pending_fifo [0:GHASH_FIFO_DEPTH-1];
    reg [GHASH_FIFO_ADDR_BITS-1:0] ghash_pending_wr_ptr;
    reg [GHASH_FIFO_ADDR_BITS-1:0] ghash_pending_rd_ptr;
    reg [GHASH_FIFO_ADDR_BITS:0]   ghash_pending_count;
    
    wire ghash_pending_empty = (ghash_pending_count == 0);
    wire ghash_pending_full = (ghash_pending_count == GHASH_FIFO_DEPTH);
    
    // GHASH is busy if we just sent data (waiting for result)
    reg ghash_busy;
    
    //=========================================================================
    // Pipeline Tracking
    //=========================================================================
    reg [4:0] blocks_in_aes;      // Blocks currently in AES pipeline
    reg [4:0] blocks_sent;        // Total blocks sent to AES
    reg [4:0] blocks_received;    // Total blocks received from AES
    reg       last_block_sent;    // Last data block has been sent
    reg       last_block_received;// Last data block has been received
    
    //=========================================================================
    // AES Core Interface
    //=========================================================================
    reg  [127:0] aes_plaintext;
    reg          aes_valid_in;
    wire [127:0] aes_ciphertext;
    wire         aes_valid_out;
    
    // Track initialization - after H and E(Y0) received, all outputs are DATA
    reg         init_complete;
    reg [1:0]   init_outputs_remaining;  // Count down from 2 (H, E(Y0))
    
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
    // Add these registers at the top with other register declarations (around line 60-80)
    reg         h_e_y0_valid;
    reg [127:0] last_key;
    reg [95:0]  last_iv;
    
    
    assign data_out = data_out_reg;
    assign data_out_valid = data_out_valid_reg;
    assign data_out_last = data_out_last_reg;
    
    assign ready = (state == ST_IDLE);
    assign busy = (state != ST_IDLE);
    // Simple input_ready - just check if we can accept data
    assign input_ready = (state == ST_READY || state == ST_PROCESSING) && 
                         !fifo_full && 
                         !last_block_sent;
    
    //=========================================================================
    // Module Instantiations
    //=========================================================================
    
    // AES-128 Pipeline (12 stages, 1 block/cycle throughput)
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
    
    // Single-cycle GHASH
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
                4'd0:  get_mask = 128'hFFFFFFFF_FFFFFFFF_FFFFFFFF_FFFFFFFF; // 16 bytes
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
            h_e_y0_valid <= 1'b0;        // ? ADD THIS
            last_key <= 128'b0;          // ? ADD THIS
            last_iv <= 96'b0;            // ? ADD THIS
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
            init_outputs_remaining <= 2'd2;
            ghash_data <= 128'b0;
            ghash_data_valid <= 1'b0;
            ghash_start <= 1'b0;
            data_out_reg <= 128'b0;
            data_out_valid_reg <= 1'b0;
            data_out_last_reg <= 1'b0;
            tag_out <= 128'b0;
            tag_valid <= 1'b0;
            tag_match <= 1'b0;
        end else if (enable) begin
            // Default: clear pulse signals
            if (ghash_start) $display("[%0t] [GCM-GHASH] ghash_start=1", $time);
            aes_valid_in <= 1'b0;
            ghash_data_valid <= 1'b0;
            ghash_start <= 1'b0;
            data_out_valid_reg <= 1'b0;
            data_out_last_reg <= 1'b0;
            tag_valid <= 1'b0;
            
            case (state)
            /*
                //=============================================================
                ST_IDLE: begin
                    if (start) begin
                        // Capture inputs
                        key_reg <= key;
                        iv_reg <= iv;
                        is_encrypt <= encrypt;
                        tag_in_reg <= tag_in;
                        
                        // Reset state
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
                        init_complete <= 1'b0;
                        init_outputs_remaining <= 2'd2;
                        
                        // Start computing H = AES(K, 0)
                        aes_plaintext <= 128'b0;
                        aes_valid_in <= 1'b1;
                        
                        state <= ST_INIT_H;
                    end
                end
              */
              ST_IDLE: begin
    if (start) begin
        // Capture inputs
        is_encrypt <= encrypt;
        tag_in_reg <= tag_in;
        
        // Reset state (not H/E(Y0) related)
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
        
        // Check if we need to re-initialize H and E(Y0)
        if (!h_e_y0_valid || (key != last_key) || (iv != last_iv)) begin
            // Need to initialize/re-initialize
            $display("[%0t] [AES-GCM] Initializing H and E(Y0) for new key/IV", $time);
            key_reg <= key;
            iv_reg <= iv;
            last_key <= key;
            last_iv <= iv;
            h_key_valid <= 1'b0;
            e_y0_valid <= 1'b0;
            h_e_y0_valid <= 1'b0;
            init_outputs_remaining <= 2'd2;
            
            // Start computing H = AES(K, 0)
            aes_plaintext <= 128'b0;
            aes_valid_in <= 1'b1;
            
            state <= ST_INIT_H;
        end else begin
            // Already initialized - reuse cached H and E(Y0)
            $display("[%0t] [AES-GCM] Reusing cached H and E(Y0)", $time);
            key_reg <= key;
            iv_reg <= iv;
            
            // Mark init as complete since we're reusing
            init_complete <= 1'b1;  // ? ADD THIS - very important!
            
            // Initialize GHASH with cached H
            ghash_start <= 1'b1;
            
            // Go directly to READY state
            state <= ST_READY;  // ? This is correct for aes_gcm_pipelined
        end
    end
end  
                //=============================================================
                ST_INIT_H: begin
                    // Send E(Y0) = AES(K, IV || 0^31 || 1)
                    aes_plaintext <= {iv_reg, 32'h00000001};
                    aes_valid_in <= 1'b1;
                    
                    state <= ST_WAIT_INIT;
                end
                /*
                //=============================================================
                ST_WAIT_INIT: begin
                    // Wait for H and E(Y0) from AES pipeline
                    // First output is H, second is E(Y0)
                    if (aes_valid_out) begin
                        if (init_outputs_remaining == 2'd2) begin
                            // First output: H
                            h_key <= aes_ciphertext;
                            h_key_valid <= 1'b1;
                            init_outputs_remaining <= 2'd1;
                        end else if (init_outputs_remaining == 2'd1) begin
                            // Second output: E(Y0)
                            e_y0 <= aes_ciphertext;
                            e_y0_valid <= 1'b1;
                            init_outputs_remaining <= 2'd0;
                            init_complete <= 1'b1;
                        end
                    end
                    
                    // Both ready? Initialize GHASH with the computed H key
                    if (h_key_valid && e_y0_valid) begin
                        ghash_start <= 1'b1;  // Now h_key has the correct value
                        state <= ST_READY;
                    end
                end
                */
                //=============================================================
                ST_WAIT_INIT: begin
                    // Wait for H and E(Y0) from AES pipeline
                    // First output is H, second is E(Y0)
                    if (aes_valid_out) begin
                        if (init_outputs_remaining == 2'd2) begin
                            // First output: H
                            h_key <= aes_ciphertext;
                            h_key_valid <= 1'b1;
                            init_outputs_remaining <= 2'd1;
                            $display("[%0t] [AES-GCM] H key computed: 0x%032h", $time, aes_ciphertext);
                        end else if (init_outputs_remaining == 2'd1) begin
                            // Second output: E(Y0)
                            e_y0 <= aes_ciphertext;
                            e_y0_valid <= 1'b1;
                            init_outputs_remaining <= 2'd0;
                            init_complete <= 1'b1;
                            $display("[%0t] [AES-GCM] E(Y0) computed: 0x%032h", $time, aes_ciphertext);
                        end
                    end
                    
                    // Both ready? Initialize GHASH with the computed H key
                    if (h_key_valid && e_y0_valid) begin
                        h_e_y0_valid <= 1'b1;  // ? ADD THIS: Mark as valid for future reuse
                        $display("[%0t] [AES-GCM] Initialization complete, marking H/E(Y0) as valid", $time);
                        ghash_start <= 1'b1;
                        state <= ST_READY;
                    end
                end
                //=============================================================
                ST_READY: begin
                    // Ready to accept data
                    if (data_valid && !fifo_full) begin
                        // Store in FIFO
                        data_fifo[fifo_wr_ptr] <= data_in;
                        bytes_fifo[fifo_wr_ptr] <= data_bytes_valid;
                        last_fifo[fifo_wr_ptr] <= data_last;
                        fifo_wr_ptr <= fifo_wr_ptr + 1;
                        fifo_count <= fifo_count + 1;
                        
                        // Send counter block to AES
                        aes_plaintext <= {iv_reg, counter};
                        aes_valid_in <= 1'b1;
                        counter <= counter + 1;
                        blocks_in_aes <= blocks_in_aes + 1;
                        blocks_sent <= blocks_sent + 1;
                        
                        // Track data length
                        if (data_bytes_valid == 4'd0) begin
                            data_len_bits <= data_len_bits + 64'd128;
                        end else begin
                            data_len_bits <= data_len_bits + {57'b0, data_bytes_valid, 3'b0};
                        end
                        
                        if (data_last) begin
                            last_block_sent <= 1'b1;
                        end
                        
                        state <= ST_PROCESSING;
                    end else if (!data_valid && data_last) begin
                        // No data, just compute tag
                        state <= ST_FINAL_GHASH;
                    end
                end
                
                //=============================================================
                ST_PROCESSING: begin
                    //=========================================================
                    // INPUT SIDE: Accept new data and feed AES pipeline
                    //=========================================================
                    if (data_valid && !fifo_full && !last_block_sent) begin
                        // Store in FIFO
                        data_fifo[fifo_wr_ptr] <= data_in;
                        bytes_fifo[fifo_wr_ptr] <= data_bytes_valid;
                        last_fifo[fifo_wr_ptr] <= data_last;
                        fifo_wr_ptr <= fifo_wr_ptr + 1;
                        fifo_count <= fifo_count + 1;
                        
                        // Send counter block to AES
                        aes_plaintext <= {iv_reg, counter};
                        aes_valid_in <= 1'b1;
                        counter <= counter + 1;
                        blocks_in_aes <= blocks_in_aes + 1;
                        blocks_sent <= blocks_sent + 1;
                        
                        // Track data length
                        if (data_bytes_valid == 4'd0) begin
                            data_len_bits <= data_len_bits + 64'd128;
                        end else begin
                            data_len_bits <= data_len_bits + {57'b0, data_bytes_valid, 3'b0};
                        end
                        
                        if (data_last) begin
                            last_block_sent <= 1'b1;
                        end
                    end
                    
                    //=========================================================
                    // OUTPUT SIDE: Handle AES outputs (all outputs are DATA after init)
                    //=========================================================
                    if (aes_valid_out && init_complete) begin
                        // Get plaintext/ciphertext from FIFO
                        // XOR with AES output to get ciphertext/plaintext
                        if (is_encrypt) begin
                            // Encryption: CT = PT XOR AES(counter)
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0) begin
                                data_out_reg <= (data_fifo[fifo_rd_ptr] ^ aes_ciphertext) & 
                                               get_mask(bytes_fifo[fifo_rd_ptr]);
                            end else begin
                                data_out_reg <= data_fifo[fifo_rd_ptr] ^ aes_ciphertext;
                            end
                        end else begin
                            // Decryption: PT = CT XOR AES(counter)
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0) begin
                                data_out_reg <= (data_fifo[fifo_rd_ptr] ^ aes_ciphertext) & 
                                               get_mask(bytes_fifo[fifo_rd_ptr]);
                            end else begin
                                data_out_reg <= data_fifo[fifo_rd_ptr] ^ aes_ciphertext;
                            end
                        end
                        
                        data_out_valid_reg <= 1'b1;
                        data_out_last_reg <= last_fifo[fifo_rd_ptr];
                        
                        // Queue ciphertext for GHASH (don't send directly - use pending FIFO)
                        if (is_encrypt) begin
                            // For encryption: GHASH the output (ciphertext)
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0) begin
                                ghash_pending_fifo[ghash_pending_wr_ptr] <= 
                                    (data_fifo[fifo_rd_ptr] ^ aes_ciphertext) & get_mask(bytes_fifo[fifo_rd_ptr]);
                            end else begin
                                ghash_pending_fifo[ghash_pending_wr_ptr] <= 
                                    data_fifo[fifo_rd_ptr] ^ aes_ciphertext;
                            end
                        end else begin
                            // For decryption: GHASH the input (ciphertext)
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0) begin
                                ghash_pending_fifo[ghash_pending_wr_ptr] <= 
                                    data_fifo[fifo_rd_ptr] & get_mask(bytes_fifo[fifo_rd_ptr]);
                            end else begin
                                ghash_pending_fifo[ghash_pending_wr_ptr] <= data_fifo[fifo_rd_ptr];
                            end
                        end
                        ghash_pending_wr_ptr <= ghash_pending_wr_ptr + 1;
                        ghash_pending_count <= ghash_pending_count + 1;
                        
                        // Update FIFO read pointer
                        fifo_rd_ptr <= fifo_rd_ptr + 1;
                        fifo_count <= fifo_count - 1;
                        blocks_in_aes <= blocks_in_aes - 1;
                        blocks_received <= blocks_received + 1;
                        
                        if (last_fifo[fifo_rd_ptr]) begin
                            last_block_received <= 1'b1;
                        end
                    end
                    
                    //=========================================================
                    // GHASH Processing: Send from pending FIFO when GHASH is ready
                    //=========================================================
                    if (!ghash_pending_empty && !ghash_busy) begin
                        ghash_data <= ghash_pending_fifo[ghash_pending_rd_ptr];
                        ghash_data_valid <= 1'b1;
                        ghash_pending_rd_ptr <= ghash_pending_rd_ptr + 1;
                        ghash_pending_count <= ghash_pending_count - 1;
                        ghash_busy <= 1'b1;
                    end
                    
                    // Clear ghash_busy when result is valid
                    if (ghash_result_valid) begin
                        ghash_busy <= 1'b0;
                    end
                    
                    // Handle simultaneous GHASH pending write and read
                    if (aes_valid_out && init_complete &&
                        !ghash_pending_empty && !ghash_busy) begin
                        ghash_pending_count <= ghash_pending_count; // No net change
                    end
                    
                    // Adjust fifo_count for simultaneous read/write
                    if (data_valid && !fifo_full && !last_block_sent &&
                        aes_valid_out && init_complete) begin
                        fifo_count <= fifo_count; // No net change
                    end
                    
                    //=========================================================
                    // State transitions
                    //=========================================================
                    if (last_block_sent && last_block_received && ghash_pending_empty && !ghash_busy) begin
                        state <= ST_FINAL_GHASH;
                    end else if (last_block_sent && !aes_valid_out) begin
                        state <= ST_DRAINING;
                    end
                end
                
                //=============================================================
                ST_DRAINING: begin
                    // Drain remaining blocks from AES pipeline
                    if (aes_valid_out && init_complete) begin
                        // Same output handling as ST_PROCESSING
                        if (is_encrypt) begin
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0) begin
                                data_out_reg <= (data_fifo[fifo_rd_ptr] ^ aes_ciphertext) & 
                                               get_mask(bytes_fifo[fifo_rd_ptr]);
                            end else begin
                                data_out_reg <= data_fifo[fifo_rd_ptr] ^ aes_ciphertext;
                            end
                        end else begin
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0) begin
                                data_out_reg <= (data_fifo[fifo_rd_ptr] ^ aes_ciphertext) & 
                                               get_mask(bytes_fifo[fifo_rd_ptr]);
                            end else begin
                                data_out_reg <= data_fifo[fifo_rd_ptr] ^ aes_ciphertext;
                            end
                        end
                        
                        data_out_valid_reg <= 1'b1;
                        data_out_last_reg <= last_fifo[fifo_rd_ptr];
                        
                        // Queue for GHASH
                        if (is_encrypt) begin
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0) begin
                                ghash_pending_fifo[ghash_pending_wr_ptr] <= 
                                    (data_fifo[fifo_rd_ptr] ^ aes_ciphertext) & get_mask(bytes_fifo[fifo_rd_ptr]);
                            end else begin
                                ghash_pending_fifo[ghash_pending_wr_ptr] <= 
                                    data_fifo[fifo_rd_ptr] ^ aes_ciphertext;
                            end
                        end else begin
                            if (bytes_fifo[fifo_rd_ptr] != 4'd0) begin
                                ghash_pending_fifo[ghash_pending_wr_ptr] <= 
                                    data_fifo[fifo_rd_ptr] & get_mask(bytes_fifo[fifo_rd_ptr]);
                            end else begin
                                ghash_pending_fifo[ghash_pending_wr_ptr] <= data_fifo[fifo_rd_ptr];
                            end
                        end
                        ghash_pending_wr_ptr <= ghash_pending_wr_ptr + 1;
                        ghash_pending_count <= ghash_pending_count + 1;
                        
                        fifo_rd_ptr <= fifo_rd_ptr + 1;
                        fifo_count <= fifo_count - 1;
                        blocks_in_aes <= blocks_in_aes - 1;
                        blocks_received <= blocks_received + 1;
                        
                        if (last_fifo[fifo_rd_ptr]) begin
                            last_block_received <= 1'b1;
                        end
                    end
                    
                    // GHASH Processing: Send from pending FIFO when GHASH is ready
                    if (!ghash_pending_empty && !ghash_busy) begin
                        ghash_data <= ghash_pending_fifo[ghash_pending_rd_ptr];
                        ghash_data_valid <= 1'b1;
                        ghash_pending_rd_ptr <= ghash_pending_rd_ptr + 1;
                        ghash_pending_count <= ghash_pending_count - 1;
                        ghash_busy <= 1'b1;
                    end
                    
                    // Clear ghash_busy when result is valid
                    if (ghash_result_valid) begin
                        ghash_busy <= 1'b0;
                    end
                    
                    // Handle simultaneous GHASH pending write and read
                    if (aes_valid_out && init_complete &&
                        !ghash_pending_empty && !ghash_busy) begin
                        ghash_pending_count <= ghash_pending_count; // No net change
                    end
                    
                    // Transition when all data received and GHASH queue empty
                    if (last_block_received && ghash_pending_empty && !ghash_busy) begin
                        state <= ST_FINAL_GHASH;
                    end
                end
                
                //=============================================================
                ST_FINAL_GHASH: begin
                    // Make sure GHASH is not busy before sending length block
                    if (!ghash_busy) begin
                        // GHASH the length block: AAD_len (64 bits) || Data_len (64 bits)
                        ghash_data <= {aad_len_bits, data_len_bits};
                        ghash_data_valid <= 1'b1;
                        ghash_busy <= 1'b1;
                        state <= ST_COMPUTE_TAG;
                    end
                    
                    // Clear ghash_busy when result is valid
                    if (ghash_result_valid) begin
                        ghash_busy <= 1'b0;
                    end
                end
                
                //=============================================================
                ST_COMPUTE_TAG: begin
                    // Wait for GHASH result, then XOR with E(Y0)
                    if (ghash_result_valid) begin
                        tag_out <= ghash_result ^ e_y0;
                        tag_valid <= 1'b1;
                        
                        // For decryption, check tag
                        if (!is_encrypt) begin
                            tag_match <= ((ghash_result ^ e_y0) == tag_in);
                            //tag_match <= ((ghash_result ^ e_y0) == tag_in_reg);
                        end else begin
                            tag_match <= 1'b1;
                        end
                        
                        state <= ST_DONE;
                    end
                end
                
                //=============================================================
                ST_DONE: begin
                    state <= ST_IDLE;
                end
                
                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
