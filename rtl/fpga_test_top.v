`timescale 1ns / 1ps
//============================================================================
// FPGA Top-Level: Test Controller + Secure Memory System - FIXED
//
// No hierarchical references. All signals use proper port connections.
//============================================================================

module fpga_test_top #(
    parameter ADDR_WIDTH     = 32,
    parameter DATA_WIDTH     = 128,
    parameter NUM_CLIENTS    = 2,
    parameter NUM_REGIONS    = 16,
    parameter TOKEN_WIDTH    = 32,
    parameter LEASE_ID_WIDTH = 8
)(
    input  wire         clk,
    input  wire         rst_n,
    
    // User interface
    input  wire         btn_start,
    output wire         led_running,
    output wire         led_pass,
    output wire         led_fail,
    output wire [3:0]   led_test_id,
    output wire [3:0]   led_fail_id,
    output wire [7:0]   led_error_count,
    
    // AXI Master to DDR (256-bit)
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
    output wire                    m_axi_rready
);

    //=========================================================================
    // AES Key and IV Seed
    //=========================================================================
    wire [127:0] aes_key = 128'h2b7e1516_28aed2a6_abf71588_09cf4f3c;
    wire [95:0]  aes_iv  = 96'hcafebabe_facedbad_decaf888;

    //=========================================================================
    // Button Edge Detector
    //=========================================================================
    reg btn_d1, btn_d2;
    wire btn_rising = btn_d1 && !btn_d2;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btn_d1 <= 1'b0;
            btn_d2 <= 1'b0;
        end else begin
            btn_d1 <= btn_start;
            btn_d2 <= btn_d1;
        end
    end

    //=========================================================================
    // Interconnect
    //=========================================================================
    wire                               mgmt_req;
    wire [2:0]                         mgmt_op;
    wire [LEASE_ID_WIDTH-1:0]          mgmt_lease_id;
    wire [$clog2(NUM_CLIENTS)-1:0]     mgmt_client_id;
    wire [ADDR_WIDTH-1:0]              mgmt_base_addr;
    wire [ADDR_WIDTH-1:0]              mgmt_size;
    wire [31:0]                        mgmt_duration;
    wire [TOKEN_WIDTH-1:0]             mgmt_token_in;
    wire                               mgmt_ack;
    wire                               mgmt_error;
    wire [TOKEN_WIDTH-1:0]             mgmt_token_out;

    wire [NUM_CLIENTS-1:0]                    client_req;
    wire [NUM_CLIENTS-1:0]                    client_wr_en;
    wire [NUM_CLIENTS*ADDR_WIDTH-1:0]         client_addr;
    wire [NUM_CLIENTS*DATA_WIDTH-1:0]         client_wdata;
    wire [NUM_CLIENTS*TOKEN_WIDTH-1:0]        client_token;
    wire [NUM_CLIENTS*LEASE_ID_WIDTH-1:0]     client_lease_id;
    wire [NUM_CLIENTS-1:0]                    client_ack;
    wire [NUM_CLIENTS*DATA_WIDTH-1:0]         client_rdata;
    wire [NUM_CLIENTS-1:0]                    client_rdata_valid;
    wire [NUM_CLIENTS-1:0]                    access_violation;

    wire aes_busy;
    wire aes_done;
    wire aes_tag_match;
    wire [95:0] active_iv;
    wire        iv_updated;
    wire        plaintext_ready_wire;   // proper port from SMS

    wire        test_running_w;
    wire        test_done_w;
    wire        test_pass_w;
    wire [3:0]  test_id_w;
    wire [3:0]  test_fail_id_w;
    wire [7:0]  error_count_w;

    //=========================================================================
    // Latch pass/fail after test_done
    //=========================================================================
    reg led_pass_reg;
    reg led_fail_reg;
    reg [3:0] led_fail_id_reg;
    reg [7:0] led_error_count_reg;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            led_pass_reg        <= 1'b0;
            led_fail_reg        <= 1'b0;
            led_fail_id_reg     <= 4'd0;
            led_error_count_reg <= 8'd0;
        end else if (test_done_w) begin
            led_pass_reg        <= test_pass_w;
            led_fail_reg        <= ~test_pass_w;
            led_fail_id_reg     <= test_fail_id_w;
            led_error_count_reg <= error_count_w;
        end else if (btn_rising) begin
            led_pass_reg        <= 1'b0;
            led_fail_reg        <= 1'b0;
            led_fail_id_reg     <= 4'd0;
            led_error_count_reg <= 8'd0;
        end
    end

    assign led_running     = test_running_w;
    assign led_pass        = led_pass_reg;
    assign led_fail        = led_fail_reg;
    assign led_test_id     = test_id_w;
    assign led_fail_id     = led_fail_id_reg;
    assign led_error_count = led_error_count_reg;

    //=========================================================================
    // Test Controller
    //=========================================================================
    fpga_test_controller #(
        .ADDR_WIDTH     (ADDR_WIDTH),
        .DATA_WIDTH     (DATA_WIDTH),
        .NUM_CLIENTS    (NUM_CLIENTS),
        .NUM_REGIONS    (NUM_REGIONS),
        .TOKEN_WIDTH    (TOKEN_WIDTH),
        .LEASE_ID_WIDTH (LEASE_ID_WIDTH),
        .NUM_BLOCKS     (255),
        .TIMEOUT_CYCLES (100000)
    ) test_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (btn_rising),
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
        .client_ack     (client_ack),
        .client_rdata   (client_rdata),
        .client_rdata_valid(client_rdata_valid),
        .access_violation(access_violation),
        .plaintext_ready(plaintext_ready_wire),
        .active_iv      (active_iv),
        .iv_updated     (iv_updated),
        .aes_tag_match  (aes_tag_match),
        .test_running   (test_running_w),
        .test_done      (test_done_w),
        .test_pass      (test_pass_w),
        .test_id        (test_id_w),
        .test_fail_id   (test_fail_id_w),
        .error_count    (error_count_w)
    );

    //=========================================================================
    // Secure Memory System
    //=========================================================================
    secure_memory_system #(
        .ADDR_WIDTH     (ADDR_WIDTH),
        .DATA_WIDTH     (DATA_WIDTH),
        .NUM_CLIENTS    (NUM_CLIENTS),
        .NUM_REGIONS    (NUM_REGIONS),
        .TOKEN_WIDTH    (TOKEN_WIDTH),
        .LEASE_ID_WIDTH (LEASE_ID_WIDTH)
    ) sms (
        .clk                (clk),
        .rst_n              (rst_n),
        .mgmt_req           (mgmt_req),
        .mgmt_op            (mgmt_op),
        .mgmt_lease_id      (mgmt_lease_id),
        .mgmt_client_id     (mgmt_client_id),
        .mgmt_base_addr     (mgmt_base_addr),
        .mgmt_size          (mgmt_size),
        .mgmt_duration      (mgmt_duration),
        .mgmt_token_in      (mgmt_token_in),
        .mgmt_ack           (mgmt_ack),
        .mgmt_error         (mgmt_error),
        .mgmt_token_out     (mgmt_token_out),
        .client_req         (client_req),
        .client_wr_en       (client_wr_en),
        .client_addr        (client_addr),
        .client_wdata       (client_wdata),
        .client_token       (client_token),
        .client_lease_id    (client_lease_id),
        .client_ack         (client_ack),
        .client_rdata       (client_rdata),
        .client_rdata_valid (client_rdata_valid),
        .access_violation   (access_violation),
        .aes_key            (aes_key),
        .aes_iv             (aes_iv),
        .m_axi_awaddr       (m_axi_awaddr),
        .m_axi_awlen        (m_axi_awlen),
        .m_axi_awsize       (m_axi_awsize),
        .m_axi_awburst      (m_axi_awburst),
        .m_axi_awvalid      (m_axi_awvalid),
        .m_axi_awready      (m_axi_awready),
        .m_axi_wdata        (m_axi_wdata),
        .m_axi_wstrb        (m_axi_wstrb),
        .m_axi_wlast        (m_axi_wlast),
        .m_axi_wvalid       (m_axi_wvalid),
        .m_axi_wready       (m_axi_wready),
        .m_axi_bresp        (m_axi_bresp),
        .m_axi_bvalid       (m_axi_bvalid),
        .m_axi_bready       (m_axi_bready),
        .m_axi_araddr       (m_axi_araddr),
        .m_axi_arlen        (m_axi_arlen),
        .m_axi_arsize       (m_axi_arsize),
        .m_axi_arburst      (m_axi_arburst),
        .m_axi_arvalid      (m_axi_arvalid),
        .m_axi_arready      (m_axi_arready),
        .m_axi_rdata        (m_axi_rdata),
        .m_axi_rresp        (m_axi_rresp),
        .m_axi_rlast        (m_axi_rlast),
        .m_axi_rvalid       (m_axi_rvalid),
        .m_axi_rready       (m_axi_rready),
        .aes_busy           (aes_busy),
        .aes_done           (aes_done),
        .aes_tag_match      (aes_tag_match),
        .active_iv          (active_iv),
        .iv_updated         (iv_updated),
        .plaintext_ready_out(plaintext_ready_wire)
    );

endmodule