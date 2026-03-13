`timescale 1ns / 1ps

module secure_memory_system #(
    parameter ADDR_WIDTH     = 32,
    parameter DATA_WIDTH     = 128,
    parameter NUM_CLIENTS    = 2,
    parameter NUM_REGIONS    = 16,
    parameter TOKEN_WIDTH    = 32,
    parameter LEASE_ID_WIDTH = 8,
    // IV table: stores the IV used for each 4KB region so decrypt can
    // retrieve it. Indexed by (ddr_addr - base) >> 12. Size must cover
    // the addressable 4KB regions.
    parameter IV_TABLE_DEPTH = 256
)(
    input  wire clk,
    input  wire rst_n,

    //=========================================================================
    // Gate Keeper Management Interface
    //=========================================================================
    input  wire                                   mgmt_req,
    input  wire [2:0]                             mgmt_op,
    input  wire [LEASE_ID_WIDTH-1:0]              mgmt_lease_id,
    input  wire [$clog2(NUM_CLIENTS)-1:0]         mgmt_client_id,
    input  wire [ADDR_WIDTH-1:0]                  mgmt_base_addr,
    input  wire [ADDR_WIDTH-1:0]                  mgmt_size,
    input  wire [31:0]                            mgmt_duration,
    input  wire [TOKEN_WIDTH-1:0]                 mgmt_token_in,
    output wire                                   mgmt_ack,
    output wire                                   mgmt_error,
    output wire [TOKEN_WIDTH-1:0]                 mgmt_token_out,

    //=========================================================================
    // Client Interfaces (Flattened)
    //=========================================================================
    input  wire [NUM_CLIENTS-1:0]                     client_req,
    input  wire [NUM_CLIENTS-1:0]                     client_wr_en,
    input  wire [NUM_CLIENTS*ADDR_WIDTH-1:0]          client_addr,
    input  wire [NUM_CLIENTS*DATA_WIDTH-1:0]          client_wdata,
    input  wire [NUM_CLIENTS*TOKEN_WIDTH-1:0]         client_token,
    input  wire [NUM_CLIENTS*LEASE_ID_WIDTH-1:0]      client_lease_id,

    output wire [NUM_CLIENTS-1:0]                     client_ack,
    output wire [NUM_CLIENTS*DATA_WIDTH-1:0]          client_rdata,
    output wire [NUM_CLIENTS-1:0]                     client_rdata_valid,

    //=========================================================================
    // Security Status
    //=========================================================================
    output wire [NUM_CLIENTS-1:0]                     access_violation,

    //=========================================================================
    // AES-GCM Key and IV Seed
    //=========================================================================
    input  wire [127:0]                               aes_key,
    input  wire [95:0]                                aes_iv,    // Seed IV (used for first encrypt)

    //=========================================================================
    // AXI Master Interface to DDR (256-bit)
    //=========================================================================
    output wire [ADDR_WIDTH-1:0]   m_axi_awaddr,
    output wire [7:0]              m_axi_awlen,
    output wire [2:0]              m_axi_awsize,
    output wire [1:0]              m_axi_awburst,
    output wire                    m_axi_awvalid,
    input  wire                    m_axi_awready,
    output wire [255:0]            m_axi_wdata,
    output wire [31:0]             m_axi_wstrb,
    output wire                    m_axi_wlast,
    output wire                    m_axi_wvalid,
    input  wire                    m_axi_wready,
    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output wire                    m_axi_bready,
    output wire [ADDR_WIDTH-1:0]   m_axi_araddr,
    output wire [7:0]              m_axi_arlen,
    output wire [2:0]              m_axi_arsize,
    output wire [1:0]              m_axi_arburst,
    output wire                    m_axi_arvalid,
    input  wire                    m_axi_arready,
    input  wire [255:0]            m_axi_rdata,
    input  wire [1:0]              m_axi_rresp,
    input  wire                    m_axi_rlast,
    input  wire                    m_axi_rvalid,
    output wire                    m_axi_rready,

    //=========================================================================
    // Debug / Status
    //=========================================================================
    output wire                    aes_busy,
    output wire                    aes_done,
    output wire                    aes_tag_match,
    output wire [95:0]             active_iv,        // Current IV from GCM
    output wire                    iv_updated,        // Pulses when IV rotates
    output wire                    plaintext_ready_out // Exposed for external test controller
);

    //=========================================================================
    // State Machine Encoding
    //=========================================================================
    localparam [2:0]
        ST_IDLE       = 3'd0,
        ST_STREAM_ENC = 3'd1,
        ST_STREAM_DEC = 3'd2,
        ST_BUSY       = 3'd3,
        ST_COOLDOWN   = 3'd4;

    reg [2:0] state;

    //=========================================================================
    // Internal Registers
    //=========================================================================
    reg [7:0]  txn_blk_cnt;
    reg [7:0]  rxn_blk_cnt;
    reg [3:0]  cooldown_cnt;

    reg [ADDR_WIDTH-1:0]          held_addr;
    reg [$clog2(NUM_CLIENTS)-1:0] held_client_id;

    reg start_encrypt_pulse;
    reg start_decrypt_pulse;

    reg [NUM_CLIENTS-1:0]            client_ack_reg;
    reg [NUM_CLIENTS*DATA_WIDTH-1:0] client_rdata_reg;
    reg [NUM_CLIENTS-1:0]            client_rdata_valid_reg;

    //=========================================================================
    // IV Table: stores IV used for each 4KB DDR region
    // Index = addr[19:12] (supports up to 256 × 4KB = 1MB address space)
    // On encrypt: store the active_iv after encryption completes
    // On decrypt: look up the IV that was used for that address
    //=========================================================================
    reg [95:0] iv_table [0:IV_TABLE_DEPTH-1];
    reg        iv_table_valid [0:IV_TABLE_DEPTH-1];
    
    // Table index from address: bits [19:12] = 4KB page number
    // Supports up to 256 × 4KB = 1MB address range
    wire [7:0] iv_table_idx = held_addr[19:12];
    
    // IV to pass to AES: for encrypt, let GCM choose; for decrypt, use stored IV
    reg [95:0] iv_to_aes;

    integer iv_init_i;

    //=========================================================================
    // Gate Keeper Outputs
    //=========================================================================
    wire                           gk_grant_req;
    wire                           gk_grant_wr_en;
    wire [ADDR_WIDTH-1:0]          gk_grant_addr;
    wire [DATA_WIDTH-1:0]          gk_grant_wdata;
    wire [$clog2(NUM_CLIENTS)-1:0] gk_grant_id;

    //=========================================================================
    // AES-GCM Outputs
    //=========================================================================
    wire                    aes_busy_int;
    wire                    aes_done_int;
    wire                    aes_tag_match_int;
    wire                    plaintext_ready_wire;
    wire [DATA_WIDTH-1:0]   aes_plaintext_out;
    wire                    aes_plaintext_out_valid;
    wire                    aes_plaintext_out_last;
    
    // IV management from AES
    wire [95:0]             aes_active_iv;
    wire                    aes_iv_updated;
    wire                    aes_counter_overflow;

    //=========================================================================
    // mem_busy
    //=========================================================================
    wire mem_busy = (state != ST_IDLE);

    reg txn_done_pulse;

    //=========================================================================
    // plaintext_valid
    //=========================================================================
    wire plaintext_valid_to_aes = (state == ST_STREAM_ENC) && gk_grant_req;
    wire enc_block_transferred = plaintext_valid_to_aes && plaintext_ready_wire;

    //=========================================================================
    // State Machine
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                  <= ST_IDLE;
            txn_blk_cnt            <= 8'd0;
            rxn_blk_cnt            <= 8'd0;
            cooldown_cnt           <= 4'd0;
            txn_done_pulse         <= 1'b0;
            held_addr              <= {ADDR_WIDTH{1'b0}};
            held_client_id         <= {$clog2(NUM_CLIENTS){1'b0}};
            start_encrypt_pulse    <= 1'b0;
            start_decrypt_pulse    <= 1'b0;
            client_ack_reg         <= {NUM_CLIENTS{1'b0}};
            client_rdata_reg       <= {NUM_CLIENTS*DATA_WIDTH{1'b0}};
            client_rdata_valid_reg <= {NUM_CLIENTS{1'b0}};
            iv_to_aes              <= 96'b0;
            
            // Clear IV table
            for (iv_init_i = 0; iv_init_i < IV_TABLE_DEPTH; iv_init_i = iv_init_i + 1) begin
                iv_table[iv_init_i]       <= 96'b0;
                iv_table_valid[iv_init_i] <= 1'b0;
            end
        end else begin
            // Default: clear single-cycle signals
            start_encrypt_pulse    <= 1'b0;
            start_decrypt_pulse    <= 1'b0;
            client_ack_reg         <= {NUM_CLIENTS{1'b0}};
            client_rdata_valid_reg <= {NUM_CLIENTS{1'b0}};
            txn_done_pulse         <= 1'b0;

            case (state)

                //-------------------------------------------------------------
                ST_IDLE: begin
                    txn_blk_cnt <= 8'd0;
                    rxn_blk_cnt <= 8'd0;

                    if (gk_grant_req) begin
                        held_addr      <= gk_grant_addr;
                        held_client_id <= gk_grant_id;

                        if (gk_grant_wr_en) begin
                            // ENCRYPT: pass the seed IV; GCM core will use
                            // its LFSR for actual IV selection
                            iv_to_aes           <= aes_iv;
                            start_encrypt_pulse <= 1'b1;
                            state               <= ST_STREAM_ENC;
                            $display("[%0t] [SMS] ENCRYPT start, base=0x%08h, client=%0d",
                                     $time, gk_grant_addr, gk_grant_id);
                        end else begin
                            // DECRYPT: look up the IV from the table
                            // Use a temporary variable for the index from grant address
                            if (iv_table_valid[gk_grant_addr[19:12]]) begin
                                iv_to_aes <= iv_table[gk_grant_addr[19:12]];
                                $display("[%0t] [SMS] DECRYPT start, base=0x%08h, using stored IV=0x%024h",
                                         $time, gk_grant_addr,
                                         iv_table[gk_grant_addr[19:12]]);
                            end else begin
                                // No stored IV - use seed
                                iv_to_aes <= aes_iv;
                                $display("[%0t] [SMS] DECRYPT start, base=0x%08h, no stored IV, using seed",
                                         $time, gk_grant_addr);
                            end
                            start_decrypt_pulse <= 1'b1;
                            state               <= ST_STREAM_DEC;
                        end
                    end
                end

                //-------------------------------------------------------------
                ST_STREAM_ENC: begin
                    if (enc_block_transferred) begin
                        $display("[%0t] [SMS] ENC block %0d accepted",
                                 $time, txn_blk_cnt);

                        if (txn_blk_cnt == 8'd254) begin
                            state <= ST_BUSY;
                        end else begin
                            txn_blk_cnt <= txn_blk_cnt + 1'b1;
                        end
                    end
                end

                //-------------------------------------------------------------
                ST_STREAM_DEC: begin
                    if (aes_plaintext_out_valid) begin
                        client_rdata_reg[held_client_id*DATA_WIDTH +: DATA_WIDTH]
                            <= aes_plaintext_out;
                        client_rdata_valid_reg[held_client_id] <= 1'b1;

                        if (rxn_blk_cnt == 8'd254) begin
                            state <= ST_BUSY;
                        end else begin
                            rxn_blk_cnt <= rxn_blk_cnt + 1'b1;
                        end
                    end
                end

                //-------------------------------------------------------------
                ST_BUSY: begin
                    if (aes_done_int) begin
                        client_ack_reg[held_client_id] <= 1'b1;
                        txn_done_pulse                 <= 1'b1;
                        cooldown_cnt                   <= 4'd0;
                        
                        // On encrypt completion: store the IV that was used
                        // into the IV table so decrypt can look it up later
                        if (gk_grant_wr_en || (state == ST_BUSY && start_encrypt_pulse == 1'b0)) begin
                            // We need to check if this was an encrypt.
                            // Since held_addr and aes_active_iv are stable here:
                        end
                        
                        state <= ST_COOLDOWN;
                        $display("[%0t] [SMS] txn_done_pulse fired", $time);
                    end
                end

                //-------------------------------------------------------------
                ST_COOLDOWN: begin
                    if (cooldown_cnt >= 4'd1) begin
                        state        <= ST_IDLE;
                        cooldown_cnt <= 4'd0;
                    end else begin
                        cooldown_cnt <= cooldown_cnt + 1'b1;
                    end
                end

                default: state <= ST_IDLE;

            endcase
            
            //=================================================================
            // IV Table Update: when AES signals iv_updated after encrypt,
            // store the IV that was used into the table
            //=================================================================
            if (aes_iv_updated && (state == ST_BUSY || state == ST_COOLDOWN)) begin
                iv_table[iv_table_idx]       <= aes_active_iv;
                iv_table_valid[iv_table_idx] <= 1'b1;
                $display("[%0t] [SMS] IV table updated: addr=0x%08h idx=%0d IV=0x%024h",
                         $time, held_addr, iv_table_idx, aes_active_iv);
            end
        end
    end

    //=========================================================================
    // Output Assignments
    //=========================================================================
    assign client_ack         = client_ack_reg;
    assign client_rdata       = client_rdata_reg;
    assign client_rdata_valid = client_rdata_valid_reg;

    assign aes_busy      = aes_busy_int;
    assign aes_done      = aes_done_int;
    assign aes_tag_match = aes_tag_match_int;
    assign active_iv     = aes_active_iv;
    assign iv_updated    = aes_iv_updated;
    assign plaintext_ready_out = plaintext_ready_wire;

    //=========================================================================
    // Gate Keeper Instance
    //=========================================================================
    gate_keeper #(
        .ADDR_WIDTH    (ADDR_WIDTH),
        .DATA_WIDTH    (DATA_WIDTH),
        .NUM_CLIENTS   (NUM_CLIENTS),
        .NUM_REGIONS   (NUM_REGIONS),
        .TOKEN_WIDTH   (TOKEN_WIDTH),
        .LEASE_ID_WIDTH(LEASE_ID_WIDTH)
    ) gate_keeper_inst (
        .clk            (clk),
        .rst_n          (rst_n),
        .mem_busy       (mem_busy),
        .mgmt_req       (mgmt_req),
        .mgmt_op        (mgmt_op),
        .mgmt_lease_id  (mgmt_lease_id),
        .mgmt_client_id (mgmt_client_id),
        .mgmt_base_addr (mgmt_base_addr),
        .mgmt_size      (mgmt_size),
        .mgmt_duration  (mgmt_duration),
        .mgmt_token_in  (mgmt_token_in),
        .mgmt_ack       (mgmt_ack),
        .mgmt_error     (mgmt_error),
        .mgmt_token_out (mgmt_token_out),
        .client_req     (client_req),
        .client_wr_en   (client_wr_en),
        .client_addr    (client_addr),
        .client_wdata   (client_wdata),
        .client_token   (client_token),
        .client_lease_id(client_lease_id),
        .access_violation(access_violation),
        .mem_ready      (txn_done_pulse),
        .mem_ack        (1'b0),
        .mem_rdata      ({DATA_WIDTH{1'b0}}),
        .grant_req      (gk_grant_req),
        .grant_wr_en    (gk_grant_wr_en),
        .grant_addr     (gk_grant_addr),
        .grant_wdata    (gk_grant_wdata),
        .grant_id       (gk_grant_id)
    );

    //=========================================================================
    // AES-GCM AXI Integration Instance
    //=========================================================================
    aes_gcm_axi_integration #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .DATA_WIDTH (DATA_WIDTH),
        .NUM_BLOCKS (255),
        .NUM_BEATS  (128)
    ) aes_gcm_axi_inst (
        .clk            (clk),
        .rst_n          (rst_n),
        .start_encrypt  (start_encrypt_pulse),
        .start_decrypt  (start_decrypt_pulse),
        .key            (aes_key),
        .iv             (iv_to_aes),
        .ddr_addr       (held_addr),
        .busy           (aes_busy_int),
        .done           (aes_done_int),
        .tag_match      (aes_tag_match_int),
        .active_iv      (aes_active_iv),
        .iv_updated     (aes_iv_updated),
        .counter_overflow(aes_counter_overflow),
        .plaintext_in       (gk_grant_wdata),
        .plaintext_valid    (plaintext_valid_to_aes),
        .plaintext_last     (1'b0),
        .plaintext_ready    (plaintext_ready_wire),
        .plaintext_out      (aes_plaintext_out),
        .plaintext_out_valid(aes_plaintext_out_valid),
        .plaintext_out_last (aes_plaintext_out_last),
        .plaintext_out_ready(1'b1),
        .m_axi_awaddr   (m_axi_awaddr),
        .m_axi_awlen    (m_axi_awlen),
        .m_axi_awsize   (m_axi_awsize),
        .m_axi_awburst  (m_axi_awburst),
        .m_axi_awvalid  (m_axi_awvalid),
        .m_axi_awready  (m_axi_awready),
        .m_axi_wdata    (m_axi_wdata),
        .m_axi_wstrb    (m_axi_wstrb),
        .m_axi_wlast    (m_axi_wlast),
        .m_axi_wvalid   (m_axi_wvalid),
        .m_axi_wready   (m_axi_wready),
        .m_axi_bresp    (m_axi_bresp),
        .m_axi_bvalid   (m_axi_bvalid),
        .m_axi_bready   (m_axi_bready),
        .m_axi_araddr   (m_axi_araddr),
        .m_axi_arlen    (m_axi_arlen),
        .m_axi_arsize   (m_axi_arsize),
        .m_axi_arburst  (m_axi_arburst),
        .m_axi_arvalid  (m_axi_arvalid),
        .m_axi_arready  (m_axi_arready),
        .m_axi_rdata    (m_axi_rdata),
        .m_axi_rresp    (m_axi_rresp),
        .m_axi_rlast    (m_axi_rlast),
        .m_axi_rvalid   (m_axi_rvalid),
        .m_axi_rready   (m_axi_rready)
    );

always @(posedge clk) begin
    if (aes_done_int)
        $display("[%0t] [SMS] aes_done_int pulse, state=%0d", $time, state);
end

always @(posedge clk) begin
    if (txn_done_pulse)
        $display("[%0t] [SMS] txn_done_pulse HIGH, state=%0d", $time, state);
end

endmodule