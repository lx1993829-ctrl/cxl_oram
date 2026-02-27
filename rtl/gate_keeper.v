`timescale 1ns / 1ps
module gate_keeper #(
    parameter ADDR_WIDTH     = 32,
    parameter DATA_WIDTH     = 128,
    parameter NUM_CLIENTS    = 2,
    parameter NUM_REGIONS    = 16,
    parameter TOKEN_WIDTH    = 32,
    parameter LEASE_ID_WIDTH = 8
)(
    input  wire clk,
    input  wire rst_n, 
    input wire mem_busy, 
    // Management Interface (AXI-Lite side)
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

    // Client Interfaces (Flattened)
    input  wire [NUM_CLIENTS-1:0]            client_req,
    input  wire [NUM_CLIENTS-1:0]            client_wr_en,
    input  wire [NUM_CLIENTS*ADDR_WIDTH-1:0] client_addr,
    input  wire [NUM_CLIENTS*DATA_WIDTH-1:0] client_wdata,
    input  wire [NUM_CLIENTS*TOKEN_WIDTH-1:0] client_token,
    input  wire [NUM_CLIENTS*LEASE_ID_WIDTH-1:0] client_lease_id,
    
   // output wire [NUM_CLIENTS-1:0]            client_ack,
   // output wire [NUM_CLIENTS*DATA_WIDTH-1:0] client_rdata, // Matches flattened arbiter
    
    // Security Status
    output wire [NUM_CLIENTS-1:0]            access_violation,

    // External Memory Interface
    input  wire                              mem_ready,
    input  wire                              mem_ack,
    input  wire [DATA_WIDTH-1:0]             mem_rdata,
    output wire                              grant_req,
    output wire                              grant_wr_en,
    output wire [ADDR_WIDTH-1:0]             grant_addr,
    output wire [DATA_WIDTH-1:0]             grant_wdata,
    output wire [$clog2(NUM_CLIENTS)-1:0]    grant_id
);

    //=========================================================================
    // Internal Wiring
    //=========================================================================
    wire [NUM_CLIENTS-1:0] val_valid;
    wire [NUM_CLIENTS-1:0] secure_req;

    //=========================================================================
    // 1. Lease Token Table (The Gatekeeper)
    //=========================================================================
    lease_token_table #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .NUM_CLIENTS(NUM_CLIENTS),
        .NUM_REGIONS(NUM_REGIONS),
        .LEASE_ID_WIDTH(LEASE_ID_WIDTH),
        .TOKEN_WIDTH(TOKEN_WIDTH)
    ) lease_token_table_inst (
        .clk(clk),
        .rst_n(rst_n),
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
        .client_addr(client_addr),        // Flattened
        .client_token(client_token),      // Flattened
        .client_lease_id(client_lease_id), // Flattened
        .val_valid(val_valid),
        .active_leases() 
    );

    //=========================================================================
    // 2. Gatekeeping Logic
    //=========================================================================
    // Mask the raw client_req. If val_valid[i] is 0, the arbiter never sees the req.
    assign secure_req = client_req & val_valid;
    //assign secure_req = client_req & val_valid & ~{NUM_CLIENTS{mem_busy}};
    
    // Debug/Security signal: 1 if requester is blocked by the table
    assign access_violation = client_req & ~val_valid;

    //=========================================================================
    // 3. Flattened Client Arbiter
    //=========================================================================
    client_arbiter #(
        .NUM_CLIENTS(NUM_CLIENTS),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) arbiter_inst (
        .clk(clk),
        .rst_n(rst_n),
        .client_req(secure_req),      // <--- Only secure requests go here
        .client_wr_en(client_wr_en),
        .client_addr(client_addr),    // Flattened
        .client_wdata(client_wdata),  // Flattened
        .grant_id(grant_id),
        .grant_req(grant_req),
        .grant_wr_en(grant_wr_en),
        .grant_addr(grant_addr),
        .grant_wdata(grant_wdata),
        .mem_ready(mem_ready),
        .mem_ack(mem_ack),
        .mem_rdata(mem_rdata)
    //    .client_ack(client_ack),
    //    .client_rdata(client_rdata)   // Flattened
    );

endmodule
