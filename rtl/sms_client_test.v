`timescale 1ns / 1ps
//=============================================================================
// sms_test_client.v
//
// Single-client test driver for secure_memory_system.
// Only one client runs at a time. vio_client_sel picks which slot (0 or 1).
// The other slot is driven to zero so it never accidentally requests.
//
// Port widths match SMS exactly - plug directly in.
//
// WORKFLOW:
//   1. Grant a lease via mgmt interface, note mgmt_token_out value.
//   2. Set vio_client_sel, vio_token, vio_lease_id, vio_base_addr in VIO.
//   3. Pulse vio_start.
//   4. Watch vio_test_pass / vio_test_fail.
//=============================================================================
module sms_test_client #(
    parameter ADDR_WIDTH     = 32,
    parameter DATA_WIDTH     = 128,
    parameter TOKEN_WIDTH    = 32,
    parameter LEASE_ID_WIDTH = 8,
    parameter NUM_CLIENTS    = 2,
    parameter NUM_BLOCKS     = 255
)(
    input  wire clk,
    input  wire rst_n,

    //=========================================================================
    // VIO inputs
    //=========================================================================
    input  wire                      vio_clear,
    input  wire                      vio_start,
    input  wire                      vio_client_sel,      // 0 or 1
    input  wire [TOKEN_WIDTH-1:0]    vio_token,           // from mgmt_token_out
    input  wire [LEASE_ID_WIDTH-1:0] vio_lease_id,
    input  wire [ADDR_WIDTH-1:0]     vio_base_addr,
    input  wire                      vio_enc_only,        // 1 = skip decrypt
    input  wire                      vio_use_custom_pt,   // 0 = counter pattern
    input  wire [DATA_WIDTH-1:0]     vio_custom_pt,

    //=========================================================================
    // VIO outputs
    //=========================================================================
    output reg                       vio_enc_done,
    output reg                       vio_dec_done,
    output reg                       vio_tag_ok,
    output reg                       vio_test_pass,
    output reg                       vio_test_fail,
    output reg                       vio_test_done,
    output reg                       vio_access_viol,
    output reg  [7:0]                vio_enc_blk_cnt,
    output reg  [7:0]                vio_dec_blk_cnt,
    output reg  [7:0]                vio_mismatch_cnt,
    output reg  [3:0]                vio_state,
    output reg  [DATA_WIDTH-1:0]     vio_last_dec_blk,

    //=========================================================================
    // Flattened buses - connect directly to secure_memory_system
    // Inactive client slot is always driven 0
    //=========================================================================
    output wire [NUM_CLIENTS-1:0]                    client_req,
    output wire [NUM_CLIENTS-1:0]                    client_wr_en,
    output wire [NUM_CLIENTS*ADDR_WIDTH-1:0]         client_addr,
    output wire [NUM_CLIENTS*DATA_WIDTH-1:0]         client_wdata,
    output wire [NUM_CLIENTS*TOKEN_WIDTH-1:0]        client_token,
    output wire [NUM_CLIENTS*LEASE_ID_WIDTH-1:0]     client_lease_id,

    input  wire [NUM_CLIENTS-1:0]                    client_ack,
    input  wire [NUM_CLIENTS*DATA_WIDTH-1:0]         client_rdata,
    input  wire [NUM_CLIENTS-1:0]                    client_rdata_valid,
    input  wire [NUM_CLIENTS-1:0]                    access_violation,

    input  wire                                      plaintext_ready,
    input  wire                                      aes_tag_match
);

    //=========================================================================
    // Internal scalar regs - always drive the selected client slot,
    // zero out the other slot via the assign statements below
    //=========================================================================
    reg                      req_r;
    reg                      wr_en_r;
    reg [ADDR_WIDTH-1:0]     addr_r;
    reg [DATA_WIDTH-1:0]     wdata_r;
    reg [TOKEN_WIDTH-1:0]    token_r;
    reg [LEASE_ID_WIDTH-1:0] leaseid_r;

    // Route scalar regs into the correct slot of each flattened bus
    assign client_req      = vio_client_sel ? {req_r,     1'b0}
                                            : {1'b0,      req_r};

    assign client_wr_en    = vio_client_sel ? {wr_en_r,   1'b0}
                                            : {1'b0,      wr_en_r};

    assign client_addr     = vio_client_sel
                             ? {addr_r,    {ADDR_WIDTH{1'b0}}}
                             : {{ADDR_WIDTH{1'b0}}, addr_r};

    assign client_wdata    = vio_client_sel
                             ? {wdata_r,   {DATA_WIDTH{1'b0}}}
                             : {{DATA_WIDTH{1'b0}}, wdata_r};

    assign client_token    = vio_client_sel
                             ? {token_r,   {TOKEN_WIDTH{1'b0}}}
                             : {{TOKEN_WIDTH{1'b0}}, token_r};

    assign client_lease_id = vio_client_sel
                             ? {leaseid_r, {LEASE_ID_WIDTH{1'b0}}}
                             : {{LEASE_ID_WIDTH{1'b0}}, leaseid_r};

    // Pick the relevant ack/rdata/violation from the selected slot
    wire                    sel_ack         = client_ack[vio_client_sel];
    wire [DATA_WIDTH-1:0]   sel_rdata       = client_rdata[vio_client_sel*DATA_WIDTH +: DATA_WIDTH];
    wire                    sel_rdata_valid = client_rdata_valid[vio_client_sel];
    wire                    sel_violation   = access_violation[vio_client_sel];

    //=========================================================================
    // Violation latch
    // SMS fires access_violation as a single-cycle pulse. We latch it here
    // so it cannot be missed regardless of which FSM state we are in when
    // the pulse arrives. The latch is cleared when the FSM acts on it
    // (transitions to ST_DONE) or when vio_clear resets everything.
    //=========================================================================
    reg sel_violation_lat;

    //=========================================================================
    // State machine
    //=========================================================================
    localparam [3:0]
        ST_IDLE        = 4'd0,
        ST_ENC_PULSE   = 4'd1,
        ST_ENC_DROP    = 4'd2,
        ST_ENC_STREAM  = 4'd3,
        ST_GAP         = 4'd4,
        ST_DEC_PULSE   = 4'd5,
        ST_DEC_DROP    = 4'd6,
        ST_DEC_COLLECT = 4'd7,
        ST_CHECK       = 4'd8,
        ST_DONE        = 4'd9;

    reg [3:0]  state;
    reg [7:0]  enc_idx;
    reg [7:0]  dec_idx;
    reg [7:0]  mismatch_cnt_r;
    reg [7:0]  gap_cnt;
    reg        prev_ready;

    // Latched credentials
    reg [TOKEN_WIDTH-1:0]    lat_token;
    reg [LEASE_ID_WIDTH-1:0] lat_lease_id;
    reg [ADDR_WIDTH-1:0]     lat_addr;
    reg                      lat_enc_only;
    reg                      lat_use_cust;
    reg [DATA_WIDTH-1:0]     lat_cust_pt;

    // Decrypted block buffer
    reg [DATA_WIDTH-1:0] dec_buf [0:254];

    //=========================================================================
    // Plaintext generation
    //=========================================================================
    function [DATA_WIDTH-1:0] make_pt;
        input [7:0]            idx;
        input                  use_custom;
        input [DATA_WIDTH-1:0] custom_val;
        begin
            if (use_custom)
                make_pt = custom_val;
            else
                make_pt = {96'hDEAD_BEEF_CAFE_BABE_1234_5678, 24'b0, idx};
        end
    endfunction

    //=========================================================================
    // FSM
    //=========================================================================
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= ST_IDLE;
            req_r              <= 1'b0;
            wr_en_r            <= 1'b0;
            addr_r             <= {ADDR_WIDTH{1'b0}};
            wdata_r            <= {DATA_WIDTH{1'b0}};
            token_r            <= {TOKEN_WIDTH{1'b0}};
            leaseid_r          <= {LEASE_ID_WIDTH{1'b0}};
            enc_idx            <= 8'd0;
            dec_idx            <= 8'd0;
            mismatch_cnt_r     <= 8'd0;
            gap_cnt            <= 8'd0;
            prev_ready         <= 1'b0;
            sel_violation_lat  <= 1'b0;
            vio_enc_done       <= 1'b0;
            vio_dec_done       <= 1'b0;
            vio_tag_ok         <= 1'b0;
            vio_test_pass      <= 1'b0;
            vio_test_fail      <= 1'b0;
            vio_test_done      <= 1'b0;
            vio_access_viol    <= 1'b0;
            vio_enc_blk_cnt    <= 8'd0;
            vio_dec_blk_cnt    <= 8'd0;
            vio_mismatch_cnt   <= 8'd0;
            vio_state          <= 4'd0;
            vio_last_dec_blk   <= {DATA_WIDTH{1'b0}};
        end else begin
            req_r      <= 1'b0;
            wr_en_r    <= 1'b0;
            prev_ready <= plaintext_ready;

            // ----------------------------------------------------------------
            // Capture any violation pulse immediately, regardless of FSM state
            // ----------------------------------------------------------------
            if (sel_violation)
                sel_violation_lat <= 1'b1;

            vio_enc_blk_cnt  <= enc_idx;
            vio_dec_blk_cnt  <= dec_idx;
            vio_mismatch_cnt <= mismatch_cnt_r;
            vio_state        <= state;

            case (state)
                ST_IDLE: begin
                    if (vio_start) begin
                        lat_token         <= vio_token;
                        lat_lease_id      <= vio_lease_id;
                        lat_addr          <= vio_base_addr;
                        lat_enc_only      <= vio_enc_only;
                        lat_use_cust      <= vio_use_custom_pt;
                        lat_cust_pt       <= vio_custom_pt;
                        vio_enc_done      <= 1'b0;
                        vio_dec_done      <= 1'b0;
                        vio_tag_ok        <= 1'b0;
                        vio_test_pass     <= 1'b0;
                        vio_test_fail     <= 1'b0;
                        vio_test_done     <= 1'b0;
                        vio_access_viol   <= 1'b0;
                        vio_last_dec_blk  <= {DATA_WIDTH{1'b0}};
                        sel_violation_lat <= 1'b0;
                        enc_idx           <= 8'd0;
                        dec_idx           <= 8'd0;
                        mismatch_cnt_r    <= 8'd0;
                        gap_cnt           <= 8'd0;
                        state             <= ST_ENC_PULSE;
                    end
                end

                ST_ENC_PULSE: begin
                    req_r     <= 1'b1;
                    wr_en_r   <= 1'b1;
                    addr_r    <= lat_addr;
                    token_r   <= lat_token;
                    leaseid_r <= lat_lease_id;
                    wdata_r   <= make_pt(8'd0, lat_use_cust, lat_cust_pt);
                    enc_idx   <= 8'd0;
                    state     <= ST_ENC_DROP;
                end

                ST_ENC_DROP: begin
                    req_r   <= 1'b0;
                    wr_en_r <= 1'b0;
                    state   <= ST_ENC_STREAM;
                end

                ST_ENC_STREAM: begin
                    if (sel_violation_lat) begin
                        sel_violation_lat <= 1'b0;
                        vio_access_viol   <= 1'b1;
                        vio_test_fail     <= 1'b1;
                        state             <= ST_DONE;
                    end else if (sel_ack) begin
                        vio_enc_done <= 1'b1;
                        state        <= lat_enc_only ? ST_DONE : ST_GAP;
                    end else if (!prev_ready && plaintext_ready) begin
                        if (enc_idx < 8'd254) begin
                            enc_idx <= enc_idx + 8'd1;
                            wdata_r <= make_pt(enc_idx + 8'd1, lat_use_cust, lat_cust_pt);
                        end
                    end
                end

                ST_GAP: begin
                    if (gap_cnt >= 8'd30) begin
                        gap_cnt <= 8'd0;
                        state   <= ST_DEC_PULSE;
                    end else
                        gap_cnt <= gap_cnt + 8'd1;
                end

                ST_DEC_PULSE: begin
                    req_r     <= 1'b1;
                    wr_en_r   <= 1'b0;
                    addr_r    <= lat_addr;
                    token_r   <= lat_token;
                    leaseid_r <= lat_lease_id;
                    dec_idx   <= 8'd0;
                    state     <= ST_DEC_DROP;
                end

                ST_DEC_DROP: begin
                    req_r   <= 1'b0;
                    wr_en_r <= 1'b0;
                    state   <= ST_DEC_COLLECT;
                end

                ST_DEC_COLLECT: begin
                    if (sel_violation_lat) begin
                        sel_violation_lat <= 1'b0;
                        vio_access_viol   <= 1'b1;
                        vio_test_fail     <= 1'b1;
                        state             <= ST_DONE;
                    end else begin
                        if (sel_rdata_valid) begin
                            dec_buf[dec_idx] <= sel_rdata;
                            vio_last_dec_blk <= sel_rdata;
                            dec_idx          <= dec_idx + 8'd1;
                        end
                        if (sel_ack) begin
                            vio_tag_ok   <= aes_tag_match;
                            vio_dec_done <= 1'b1;
                            if (!aes_tag_match) begin
                                vio_test_fail <= 1'b1;
                                state         <= ST_DONE;
                            end else
                                state <= ST_CHECK;
                        end
                    end
                end

                ST_CHECK: begin
                    mismatch_cnt_r <= 8'd0;
                    for (i = 0; i < NUM_BLOCKS; i = i + 1) begin
                        if (dec_buf[i] != make_pt(i[7:0], lat_use_cust, lat_cust_pt))
                            mismatch_cnt_r <= mismatch_cnt_r + 8'd1;
                    end
                    state <= ST_DONE;
                end

                ST_DONE: begin
                    vio_test_done <= 1'b1;
                    if (!vio_test_fail) begin
                        vio_test_pass <= (mismatch_cnt_r == 8'd0) &&
                                          vio_tag_ok               &&
                                          vio_enc_done             &&
                                          (lat_enc_only || vio_dec_done);
                        vio_test_fail <= (mismatch_cnt_r != 8'd0) || !vio_tag_ok;
                    end
                    // Stay here until user asserts vio_clear
                    // Clear all status flags on the same cycle we exit
                    if (vio_clear) begin
                        state             <= ST_IDLE;
                        vio_test_done     <= 1'b0;
                        vio_test_fail     <= 1'b0;
                        vio_test_pass     <= 1'b0;
                        vio_access_viol   <= 1'b0;
                        sel_violation_lat <= 1'b0;
                    end
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

endmodule