
`timescale 1ns / 1ps

module secure_memory_system #(
    parameter ADDR_WIDTH     = 32,
    parameter DATA_WIDTH     = 128,
    parameter NUM_CLIENTS    = 2,
    parameter NUM_REGIONS    = 16,
    parameter TOKEN_WIDTH    = 32,
    parameter LEASE_ID_WIDTH = 8
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
    // For 4KB transactions:
    //   - client_addr  : base address of the 4KB region
    //   - client_wdata : current plaintext block (updated each cycle,
    //                    held stable until plaintext_ready pulses)
    //   - client_req   : held high for entire 4KB transaction
    //   - client_ack   : pulses high once when all 255 blocks are done
    //=========================================================================
    input  wire [NUM_CLIENTS-1:0]                     client_req,
    input  wire [NUM_CLIENTS-1:0]                     client_wr_en,
    input  wire [NUM_CLIENTS*ADDR_WIDTH-1:0]          client_addr,
    input  wire [NUM_CLIENTS*DATA_WIDTH-1:0]          client_wdata,
    input  wire [NUM_CLIENTS*TOKEN_WIDTH-1:0]         client_token,
    input  wire [NUM_CLIENTS*LEASE_ID_WIDTH-1:0]      client_lease_id,

    // For decrypt: client_rdata pulses each block as it is decrypted.
    // client_rdata_valid indicates which cycle the block is valid.
    // client_ack pulses once at the very end (tag verified, all done).
    output wire [NUM_CLIENTS-1:0]                     client_ack,
    output wire [NUM_CLIENTS*DATA_WIDTH-1:0]          client_rdata,
    output wire [NUM_CLIENTS-1:0]                     client_rdata_valid,

    //=========================================================================
    // Security Status
    //=========================================================================
    output wire [NUM_CLIENTS-1:0]                     access_violation,

    //=========================================================================
    // AES-GCM Key and IV
    //=========================================================================
    input  wire [127:0]                               aes_key,
    input  wire [95:0]                                aes_iv,

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
    output wire                    plaintext_ready_out,
    //=========================================================================
    // Debug / Status
    //=========================================================================
    output wire                    aes_busy,
    output wire                    aes_done,
    output wire                    aes_tag_match
);

    //=========================================================================
    // State Machine Encoding
    //=========================================================================
    localparam [2:0]
        ST_IDLE       = 3'd0,
        ST_STREAM_ENC = 3'd1,   // feeding 255 PT blocks into AES (encrypt)
        ST_STREAM_DEC = 3'd2,   // receiving 255 PT blocks from AES (decrypt)
        ST_BUSY       = 3'd3,   // waiting for aes_done after last block
        ST_COOLDOWN   = 3'd4;   // brief pause before returning to IDLE

    reg [2:0] state;

    //=========================================================================
    // Internal Registers
    //=========================================================================
    reg [7:0]  txn_blk_cnt;    // counts PT blocks fed to AES   (0..254, encrypt)
    reg [7:0]  rxn_blk_cnt;    // counts PT blocks received from AES (0..254, decrypt)
    reg [3:0]  cooldown_cnt;

    // Held copies of the granted transaction parameters
    reg [ADDR_WIDTH-1:0]          held_addr;
    reg [$clog2(NUM_CLIENTS)-1:0] held_client_id;

    // One-cycle start pulses to aes_gcm_axi_integration
    reg start_encrypt_pulse;
    reg start_decrypt_pulse;

    // Client output registers
    reg [NUM_CLIENTS-1:0]            client_ack_reg;
    reg [NUM_CLIENTS*DATA_WIDTH-1:0] client_rdata_reg;
    reg [NUM_CLIENTS-1:0]            client_rdata_valid_reg;

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
    wire                    plaintext_ready_wire;     // AES ready to accept next block
    wire [DATA_WIDTH-1:0]   aes_plaintext_out;        // decrypted block from AES
    wire                    aes_plaintext_out_valid;  // decrypted block valid
    wire                    aes_plaintext_out_last;   // last decrypted block

    //=========================================================================
    // mem_busy: tells gate_keeper not to grant new requests mid-transaction
    //=========================================================================
    wire mem_busy = (state != ST_IDLE);

    //=========================================================================
    // mem_ready / mem_ack: not used for AES path, tie off
    //=========================================================================
    reg txn_done_pulse;
    

    //=========================================================================
    // plaintext_valid: sustained high while we are streaming encrypt blocks
    // Transfer occurs each cycle where BOTH this AND plaintext_ready_wire = 1
    //=========================================================================
    wire plaintext_valid_to_aes = (state == ST_STREAM_ENC) && gk_grant_req;

    // Detect each accepted block transfer (encrypt path)
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
            txn_done_pulse <= 1'b0;
            held_addr              <= {ADDR_WIDTH{1'b0}};
            held_client_id         <= {$clog2(NUM_CLIENTS){1'b0}};
            start_encrypt_pulse    <= 1'b0;
            start_decrypt_pulse    <= 1'b0;
            client_ack_reg         <= {NUM_CLIENTS{1'b0}};
            client_rdata_reg       <= {NUM_CLIENTS*DATA_WIDTH{1'b0}};
            client_rdata_valid_reg <= {NUM_CLIENTS{1'b0}};
        end else begin
            // Default: clear single-cycle signals
            start_encrypt_pulse    <= 1'b0;
            start_decrypt_pulse    <= 1'b0;
            client_ack_reg         <= {NUM_CLIENTS{1'b0}};
            client_rdata_valid_reg <= {NUM_CLIENTS{1'b0}};
            txn_done_pulse <= 1'b0;
            case (state)

                //-------------------------------------------------------------
                // IDLE: wait for gate keeper to grant a request
                //-------------------------------------------------------------
                ST_IDLE: begin
                    txn_blk_cnt <= 8'd0;
                    rxn_blk_cnt <= 8'd0;

                    if (gk_grant_req) begin
                        // Latch transaction parameters
                        held_addr      <= gk_grant_addr;
                        held_client_id <= gk_grant_id;

                        if (gk_grant_wr_en) begin
                            //--------------------------------------------------
                            // ENCRYPT path:
                            // Fire one-cycle start pulse to AES.
                            // Next cycle AES enters ST_ENC_RECV_EVEN and will
                            // assert plaintext_ready_wire when gcm_input_ready.
                            // We go to ST_STREAM_ENC and hold plaintext_valid
                            // high - transfers happen automatically via handshake.
                            //--------------------------------------------------
                            start_encrypt_pulse <= 1'b1;
                            state               <= ST_STREAM_ENC;
                            $display("[%0t] [SMS] ENCRYPT start, base=0x%08h, client=%0d",
                                     $time, gk_grant_addr, gk_grant_id);
                        end else begin
                            //--------------------------------------------------
                            // DECRYPT path:
                            // Fire one-cycle start pulse to AES.
                            // AES will immediately begin reading DDR beats.
                            // We go to ST_STREAM_DEC and wait for AES to push
                            // decrypted blocks back via aes_plaintext_out_valid.
                            //--------------------------------------------------
                            start_decrypt_pulse <= 1'b1;
                            state               <= ST_STREAM_DEC;
                            $display("[%0t] [SMS] DECRYPT start, base=0x%08h, client=%0d",
                                     $time, gk_grant_addr, gk_grant_id);
                        end
                    end
                end

                //-------------------------------------------------------------
                // ST_STREAM_ENC:
                // plaintext_valid_to_aes is held high combinatorially (wire).
                // Each cycle where plaintext_ready_wire=1, one block is consumed.
                // The client must present the next block on gk_grant_wdata the
                // following cycle (standard AXI-Stream producer behaviour).
                // We simply count transfers and leave when all 255 are done.
                //-------------------------------------------------------------
                ST_STREAM_ENC: begin
                    $display("[%0t] [SMS-DBG] STREAM_ENC txn_blk_cnt=%0d plaintext_valid=%b plaintext_ready=%b gk_grant_req=%b",
             $time, txn_blk_cnt, plaintext_valid_to_aes, plaintext_ready_wire, gk_grant_req);
                    if (enc_block_transferred) begin
                        $display("[%0t] [SMS] ENC block %0d accepted, PT=0x%032h",
                                 $time, txn_blk_cnt, gk_grant_wdata);

                        if (txn_blk_cnt == 8'd254) begin
                            // All 255 blocks handed to AES.
                            // AES still needs to finish the last DDR write.
                            state <= ST_BUSY;
                            $display("[%0t] [SMS] All 255 ENC blocks fed, waiting for done",
                                     $time);
                        end else begin
                            txn_blk_cnt <= txn_blk_cnt + 1'b1;
                            // Stay in ST_STREAM_ENC for next block
                        end
                    end
                end

                //-------------------------------------------------------------
                // ST_STREAM_DEC:
                // AES reads DDR beats and pushes decrypted blocks via
                // aes_plaintext_out_valid / aes_plaintext_out.
                // We forward each block to the client immediately and count.
                // When the 255th block arrives we go to ST_BUSY to wait for
                // the tag verification done pulse.
                //-------------------------------------------------------------
                ST_STREAM_DEC: begin
                    if (aes_plaintext_out_valid) begin
                        // Forward block to the correct client
                        client_rdata_reg[held_client_id*DATA_WIDTH +: DATA_WIDTH]
                            <= aes_plaintext_out;
                        client_rdata_valid_reg[held_client_id] <= 1'b1;

                        $display("[%0t] [SMS] DEC block %0d ready, PT=0x%032h",
                                 $time, rxn_blk_cnt, aes_plaintext_out);

                        if (rxn_blk_cnt == 8'd254) begin
                            // Last block received from AES.
                            // Now wait for tag verification (aes_done_int).
                            state <= ST_BUSY;
                            $display("[%0t] [SMS] All 255 DEC blocks received",
                                     $time);
                        end else begin
                            rxn_blk_cnt <= rxn_blk_cnt + 1'b1;
                        end
                    end
                end

                //-------------------------------------------------------------
                // ST_BUSY:
                // Encrypt: AES is finishing the last {TAG, CT[254]} DDR write.
                // Decrypt: AES is finishing tag verification.
                // In both cases we wait for aes_done_int then ack the client.
                //-------------------------------------------------------------
                ST_BUSY: begin
                    if (aes_done_int) begin
                        client_ack_reg[held_client_id] <= 1'b1;
                        txn_done_pulse                 <= 1'b1;
                        cooldown_cnt                   <= 4'd0;
                        state                          <= ST_COOLDOWN;
                        $display("[%0t] [SMS] txn_done_pulse fired", $time);
                    end
                end
                //-------------------------------------------------------------
                // ST_COOLDOWN:
                // Give AES one cycle to reset internals before we accept a new
                // transaction. client_ack is cleared here (was set for one cycle
                // in ST_BUSY transition above, cleared by default at top of FSM).
                //-------------------------------------------------------------
                ST_COOLDOWN: begin
                    // txn_done_pulse already cleared by default assignment above
                    if (cooldown_cnt >= 4'd1) begin
                        state        <= ST_IDLE;
                        cooldown_cnt <= 4'd0;
                    end else begin
                        cooldown_cnt <= cooldown_cnt + 1'b1;
                    end
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

    //=========================================================================
    // Output Assignments
    //=========================================================================
    assign client_ack         = client_ack_reg;
    assign client_rdata       = client_rdata_reg;
    assign client_rdata_valid = client_rdata_valid_reg;
    assign plaintext_ready_out = plaintext_ready_wire;
    assign aes_busy      = aes_busy_int;
    assign aes_done      = aes_done_int;
    assign aes_tag_match = aes_tag_match_int;

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

        // Management
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

        // Clients
        .client_req     (client_req),
        .client_wr_en   (client_wr_en),
        .client_addr    (client_addr),
        .client_wdata   (client_wdata),
        .client_token   (client_token),
        .client_lease_id(client_lease_id),
        .access_violation(access_violation),

        // Memory side (to our FSM)
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

        // Control
        .start_encrypt  (start_encrypt_pulse),
        .start_decrypt  (start_decrypt_pulse),
        .key            (aes_key),
        .iv             (aes_iv),
        .ddr_addr       (held_addr),

        // Status
        .busy           (aes_busy_int),
        .done           (aes_done_int),
        .tag_match      (aes_tag_match_int),

        // Plaintext input stream (encrypt)
        // gk_grant_wdata is the live output from the arbiter -
        // the client updates it each cycle after a transfer is accepted.
        .plaintext_in       (gk_grant_wdata),
        .plaintext_valid    (plaintext_valid_to_aes),
        .plaintext_last     (1'b0),          // AES counts to 255 internally
        .plaintext_ready    (plaintext_ready_wire),

        // Plaintext output stream (decrypt)
        .plaintext_out      (aes_plaintext_out),
        .plaintext_out_valid(aes_plaintext_out_valid),
        .plaintext_out_last (aes_plaintext_out_last),
        .plaintext_out_ready(1'b1),          // we always accept output

        // AXI Master
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
/*
module secure_memory_system #(
    parameter ADDR_WIDTH     = 32,
    parameter DATA_WIDTH     = 128,
    parameter NUM_CLIENTS    = 2,
    parameter NUM_REGIONS    = 16,
    parameter TOKEN_WIDTH    = 32,
    parameter LEASE_ID_WIDTH = 8
)(
    input  wire clk,
    input  wire rst_n,
    
    //=========================================================================
    // Gate Keeper Management Interface
    //=========================================================================
    input  wire                         mgmt_req,
    input  wire [2:0]                   mgmt_op,
    input  wire [LEASE_ID_WIDTH-1:0]    mgmt_lease_id,
    input  wire [$clog2(NUM_CLIENTS)-1:0] mgmt_client_id,
    input  wire [ADDR_WIDTH-1:0]        mgmt_base_addr,
    input  wire [ADDR_WIDTH-1:0]        mgmt_size,
    input  wire [31:0]                  mgmt_duration,
    input  wire [TOKEN_WIDTH-1:0]       mgmt_token_in,
    output wire                         mgmt_ack,
    output wire                         mgmt_error,
    output wire [TOKEN_WIDTH-1:0]       mgmt_token_out,

    //=========================================================================
    // Gate Keeper Client Interfaces (Flattened)
    //=========================================================================
    input  wire [NUM_CLIENTS-1:0]            client_req,
    input  wire [NUM_CLIENTS-1:0]            client_wr_en,
    input  wire [NUM_CLIENTS*ADDR_WIDTH-1:0] client_addr,
    input  wire [NUM_CLIENTS*DATA_WIDTH-1:0] client_wdata,
    input  wire [NUM_CLIENTS*TOKEN_WIDTH-1:0] client_token,
    input  wire [NUM_CLIENTS*LEASE_ID_WIDTH-1:0] client_lease_id,
    
    output wire [NUM_CLIENTS-1:0]            client_ack,
    output wire [NUM_CLIENTS*DATA_WIDTH-1:0] client_rdata,
    
    //=========================================================================
    // Gate Keeper Security Status
    //=========================================================================
    output wire [NUM_CLIENTS-1:0]            access_violation,
    
    //=========================================================================
    // AES-GCM Control (from VIO or other source)
    //=========================================================================
    input  wire [127:0]                 aes_key,
    input  wire [95:0]                  aes_iv,
    
    //=========================================================================
    // AXI Master Interface to DDR (256-bit)
    //=========================================================================
    output wire [ADDR_WIDTH-1:0]        m_axi_awaddr,
    output wire [7:0]                   m_axi_awlen,
    output wire [2:0]                   m_axi_awsize,
    output wire [1:0]                   m_axi_awburst,
    output wire                         m_axi_awvalid,
    input  wire                         m_axi_awready,
    output wire [255:0]                 m_axi_wdata,
    output wire [31:0]                  m_axi_wstrb,
    output wire                         m_axi_wlast,
    output wire                         m_axi_wvalid,
    input  wire                         m_axi_wready,
    input  wire [1:0]                   m_axi_bresp,
    input  wire                         m_axi_bvalid,
    output wire                         m_axi_bready,
    output wire [ADDR_WIDTH-1:0]        m_axi_araddr,
    output wire [7:0]                   m_axi_arlen,
    output wire [2:0]                   m_axi_arsize,
    output wire [1:0]                   m_axi_arburst,
    output wire                         m_axi_arvalid,
    input  wire                         m_axi_arready,
    input  wire [255:0]                 m_axi_rdata,
    input  wire [1:0]                   m_axi_rresp,
    input  wire                         m_axi_rlast,
    input  wire                         m_axi_rvalid,
    output wire                         m_axi_rready,
    
    //=========================================================================
    // Debug/Status outputs
    //=========================================================================
    output wire                         aes_busy,
    output wire                         aes_done,
    output wire                         aes_tag_match
);

    //=========================================================================
    // Gate Keeper to AES Integration Signals
    //=========================================================================
    wire                              gk_grant_req;
    wire                              gk_grant_wr_en;
    wire [ADDR_WIDTH-1:0]             gk_grant_addr;
    wire [DATA_WIDTH-1:0]             gk_grant_wdata;
    wire [$clog2(NUM_CLIENTS)-1:0]    gk_grant_id;
    wire [$clog2(NUM_CLIENTS)-1:0]    gk_grant_client_id;  // ? ADD THIS LINE
    // Alias for clarity
    assign gk_grant_client_id = gk_grant_id;  // ? ADD THIS LINE
    
    // Add client_ack register
    reg [NUM_CLIENTS-1:0] client_ack_reg;  // ? ADD THIS LINE
    
    // Add plaintext output valid signal
    wire aes_plaintext_out_valid;  // ? ADD THIS LINE
    // Memory interface signals (tied to 1 as per user requirements)
    wire                              mem_ready;
    wire                              mem_ack;
    wire [DATA_WIDTH-1:0]             mem_rdata;
    
    assign mem_ready = 1'b1;  // User will handle
    assign mem_ack   = 1'b1;  // User will handle
    
    //=========================================================================
    // Pulse Generation and Handshaking State Machine
    //=========================================================================
    localparam [1:0]
        ST_IDLE = 2'd0,
        ST_WAIT_READY = 2'd1,
        ST_BUSY = 2'd2,
        ST_COOLDOWN = 2'd3;
    // At the top with other registers
    reg [(NUM_CLIENTS*DATA_WIDTH)-1:0] client_rdata_reg;
    reg [1:0] state;
    reg [3:0] cooldown_cnt;
    reg [ADDR_WIDTH-1:0] prev_grant_addr;
    reg                  prev_grant_wr_en;
    reg [DATA_WIDTH-1:0] held_plaintext;
    reg [ADDR_WIDTH-1:0] held_addr;
    reg                  start_encrypt_pulse;
    reg                  start_decrypt_pulse;
    reg                  plaintext_valid_pulse;
    wire                 plaintext_ready_wire;
        
    //=========================================================================
    // AES-GCM Interface Signals
    //=========================================================================
    wire                         aes_busy_int;
    wire                         aes_done_int;
    wire                         aes_tag_match_int;
    wire [DATA_WIDTH-1:0]        aes_plaintext_out;
    
    //=========================================================================
    // Memory Interface Assignment
    //=========================================================================
    assign mem_rdata = aes_plaintext_out;
    
    //=========================================================================
    // Debug/Status Outputs
    //=========================================================================
    assign aes_busy      = aes_busy_int;
    assign aes_done      = aes_done_int;
    assign aes_tag_match = aes_tag_match_int;
    
    //=========================================================================
    // Gate Keeper Instance
    //=========================================================================
    wire mem_busy;
    assign mem_busy = (state != ST_IDLE);
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                   <= ST_IDLE;
            cooldown_cnt            <= 4'd0;
            // In the reset section
            client_rdata_reg <= {(NUM_CLIENTS*DATA_WIDTH){1'b0}};
            client_ack_reg          <= {NUM_CLIENTS{1'b0}};  // ? ADD THIS LINE
            prev_grant_addr         <= {ADDR_WIDTH{1'b0}};
            prev_grant_wr_en        <= 1'b0;
            held_plaintext          <= {DATA_WIDTH{1'b0}};
            held_addr               <= {ADDR_WIDTH{1'b0}};
            start_encrypt_pulse     <= 1'b0;
            start_decrypt_pulse     <= 1'b0;
            plaintext_valid_pulse   <= 1'b0;
        end else begin
            // Clear pulses by default
            start_encrypt_pulse     <= 1'b0;
            start_decrypt_pulse     <= 1'b0;
            
            // Combinational generation of plaintext_valid when ready
            if (state == ST_WAIT_READY && plaintext_ready_wire) begin
                plaintext_valid_pulse <= 1'b1;
            end else begin
                plaintext_valid_pulse <= 1'b0;
            end
            
            case (state)
                ST_IDLE: begin
                    // Detect NEW request: address or mode changed
                    client_ack_reg <= {NUM_CLIENTS{1'b0}};  // ? ADD THIS LINE to clear ack
                    if (gk_grant_req) begin
                  //      if ((gk_grant_addr != prev_grant_addr) || (gk_grant_wr_en != prev_grant_wr_en)) begin
                            prev_grant_addr <= gk_grant_addr;
                            prev_grant_wr_en <= gk_grant_wr_en;
                            held_addr <= gk_grant_addr;
                            
                            if (gk_grant_wr_en) begin
                                // Encryption: hold data and start
                                held_plaintext <= gk_grant_wdata;
                                start_encrypt_pulse <= 1'b1;
                                state <= ST_WAIT_READY;
                            end else begin
                                // Decryption: just start
                                start_decrypt_pulse <= 1'b1;
                                state <= ST_BUSY;
                            end
                      //  end
                    end
                end
                
                ST_WAIT_READY: begin
                    // Wait for plaintext_ready from AES module
                    // plaintext_valid is generated combinationally above
                    if (plaintext_ready_wire) begin
                        state <= ST_BUSY;
                    end
                end
                
                ST_BUSY: begin
                    // Wait for done
                    if (aes_done_int) begin
                        cooldown_cnt <= 4'd0;
                        state <= ST_COOLDOWN;
                    end
                    // Capture decrypted plaintext when valid
                    if (!gk_grant_wr_en && aes_plaintext_out_valid) begin
                        client_rdata_reg[gk_grant_client_id*DATA_WIDTH +: DATA_WIDTH] <= aes_plaintext_out;
                        $display("[%0t] [CLIENT-RDATA] Captured PT=0x%032h for client %0d", 
                                 $time, aes_plaintext_out, gk_grant_client_id);
                    end
                    
                    // Set client_ack when done
                    if (aes_done_int) begin
                        client_ack_reg[gk_grant_client_id] <= 1'b1;
                        $display("[%0t] [CLIENT-ACK] Setting ack for client %0d", $time, gk_grant_client_id);
                        state <= ST_COOLDOWN;
                    end
                end
                
                ST_COOLDOWN: begin
                    // Wait a few cycles for AES to fully reset
                    // Clear ack immediately when entering cooldown
                    client_ack_reg <= {NUM_CLIENTS{1'b0}};  // ? ADD THIS LINE
                    if (cooldown_cnt >= 4'd1) begin
                        state <= ST_IDLE;
                        cooldown_cnt <= 4'd0;  // ? Reset counter when exiting
                        $display("[%0t] [COOLDOWN] Exiting to IDLE", $time);
                    end else begin
                        cooldown_cnt <= cooldown_cnt + 1;
                        $display("[%0t] [COOLDOWN] cnt=%0d", $time, cooldown_cnt);
                    end
                end
                
                default: state <= ST_IDLE;
            endcase
        end
    end
    // RIGHT HERE - assign immediately after the always block
    assign client_ack = client_ack_reg;
    assign client_rdata = client_rdata_reg;
    gate_keeper #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .NUM_CLIENTS(NUM_CLIENTS),
        .NUM_REGIONS(NUM_REGIONS),
        .TOKEN_WIDTH(TOKEN_WIDTH),
        .LEASE_ID_WIDTH(LEASE_ID_WIDTH)
    ) gate_keeper_inst (
        .clk(clk),
        .rst_n(rst_n),
        
        // Management Interface
        .mgmt_req(mgmt_req),
        .mgmt_op(mgmt_op),
        .mgmt_lease_id(mgmt_lease_id),
        .mgmt_client_id(mgmt_client_id),
        .mgmt_base_addr(mgmt_base_addr),
        .mgmt_size(mgmt_size),
        .mgmt_duration(mgmt_duration),
        .mgmt_token_in(mgmt_token_in),
        .mgmt_ack(mgmt_ack),
        .mgmt_error(mgmt_error),
        .mgmt_token_out(mgmt_token_out),
        
        // Client Interfaces
        .client_req(client_req),
        .client_wr_en(client_wr_en),
        .client_addr(client_addr),
        .client_wdata(client_wdata),
        .client_token(client_token),
        .client_lease_id(client_lease_id),
   //     .client_ack(client_ack),
   //     .client_rdata(client_rdata),
        
        // Security Status
        .access_violation(access_violation),
        
        // External Memory Interface
        .mem_busy(mem_busy),
        .mem_ready(mem_ready),
        .mem_ack(mem_ack),
        .mem_rdata(mem_rdata),
        .grant_req(gk_grant_req),
        .grant_wr_en(gk_grant_wr_en),
        .grant_addr(gk_grant_addr),
        .grant_wdata(gk_grant_wdata),
        .grant_id(gk_grant_id)
    );
    
    //=========================================================================
    // AES-GCM AXI Integration Instance
    //=========================================================================
    aes_gcm_axi_integration #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) aes_gcm_axi_inst (
        .clk(clk),
        .rst_n(rst_n),
        
        // Control - use generated pulses
        .start_encrypt(start_encrypt_pulse),
        .start_decrypt(start_decrypt_pulse),
        .key(aes_key),
        .iv(aes_iv),
        .ddr_addr(held_addr),
        
        // Status
        .busy(aes_busy_int),
        .done(aes_done_int),
        .tag_match(aes_tag_match_int),
        
        // Plaintext input (for encryption)
        .plaintext_in(held_plaintext),
        .plaintext_valid(plaintext_valid_pulse),
        .plaintext_last(plaintext_valid_pulse),
        .plaintext_ready(plaintext_ready_wire),
        
        // Plaintext output (for decryption)
        .plaintext_out(aes_plaintext_out),
        .plaintext_out_valid(aes_plaintext_out_valid),  
        .plaintext_out_last(),   // Not used
        .plaintext_out_ready(1'b1),
        
        // AXI Master Interface (256-bit to DDR)
        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awlen(m_axi_awlen),
        .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata),
        .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast),
        .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid),
        .m_axi_bready(m_axi_bready),
        .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen),
        .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst),
        .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(m_axi_rresp),
        .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid),
        .m_axi_rready(m_axi_rready)
    );

endmodule
*/
