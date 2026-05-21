`timescale 1ns/1ps
module lease_token_table #(
    parameter ADDR_WIDTH     = 32,
    parameter NUM_CLIENTS    = 2,
    parameter NUM_REGIONS    = 16,
    parameter LEASE_ID_WIDTH = 8,
    parameter TOKEN_WIDTH    = 32
)(
    input  wire clk,
    input  wire rst_n,
    
    input  wire mgmt_req,
    input  wire [2:0] mgmt_op,
    input  wire [LEASE_ID_WIDTH-1:0] mgmt_lease_id,
    input  wire [$clog2(NUM_CLIENTS)-1:0] mgmt_client_id,
    input  wire [ADDR_WIDTH-1:0] mgmt_base_addr,
    input  wire [ADDR_WIDTH-1:0] mgmt_size,
    input  wire [31:0] mgmt_duration,
    input  wire [TOKEN_WIDTH-1:0] mgmt_token_in,
    output reg  mgmt_ack,
    output reg  mgmt_error,
    output reg  [TOKEN_WIDTH-1:0] mgmt_token_out,
    
    input  wire [NUM_CLIENTS*ADDR_WIDTH-1:0]   client_addr,
    input  wire [NUM_CLIENTS*TOKEN_WIDTH-1:0]  client_token,
    input  wire [NUM_CLIENTS*LEASE_ID_WIDTH-1:0] client_lease_id,
    output wire [NUM_CLIENTS-1:0] val_valid,
    
    output wire [NUM_REGIONS-1:0] active_leases
);

localparam OP_GRANT    = 3'd0;
localparam OP_REVOKE   = 3'd1;
localparam OP_RENEW    = 3'd2;
localparam OP_QUERY    = 3'd3;
localparam OP_TRANSFER = 3'd4;

// =========================================================================
// Lease entry storage
// =========================================================================
reg              lease_valid_r    [0:NUM_REGIONS-1];
reg [LEASE_ID_WIDTH-1:0] lease_id_r [0:NUM_REGIONS-1];
reg [$clog2(NUM_CLIENTS)-1:0] lease_owner_r [0:NUM_REGIONS-1];
reg [ADDR_WIDTH-1:0] lease_base_addr_r [0:NUM_REGIONS-1];
reg [ADDR_WIDTH-1:0] lease_size_r      [0:NUM_REGIONS-1];
reg [TOKEN_WIDTH-1:0] lease_token_r    [0:NUM_REGIONS-1];
reg [31:0] lease_expiry_r   [0:NUM_REGIONS-1];
reg [31:0] lease_created_r  [0:NUM_REGIONS-1];

reg [31:0] system_timer;

reg [TOKEN_WIDTH-1:0] lfsr;
wire [TOKEN_WIDTH-1:0] next_token;

// =========================================================================
// State machine
// =========================================================================
localparam STATE_IDLE     = 3'd0;
localparam STATE_SEARCH   = 3'd1;
localparam STATE_GRANT    = 3'd2;
localparam STATE_REVOKE   = 3'd3;
localparam STATE_RENEW    = 3'd4;
localparam STATE_QUERY    = 3'd5;
localparam STATE_TRANSFER = 3'd6;
localparam STATE_DONE     = 3'd7;

reg [2:0] state, next_state;
reg [$clog2(NUM_REGIONS)-1:0] search_idx;
reg [$clog2(NUM_REGIONS)-1:0] free_slot;
reg [$clog2(NUM_REGIONS)-1:0] found_slot;
reg found_free;
reg found_lease;

// =========================================================================
// Token Generation
// =========================================================================
assign next_token = {lfsr[TOKEN_WIDTH-2:0], 
                     lfsr[TOKEN_WIDTH-1] ^ lfsr[TOKEN_WIDTH-3] ^ lfsr[TOKEN_WIDTH-4] ^ lfsr[0]};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) lfsr <= {{TOKEN_WIDTH-1{1'b0}}, 1'b1};
    else        lfsr <= next_token;
end

// =========================================================================
// System Timer
// =========================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) system_timer <= 32'b0;
    else        system_timer <= system_timer + 1'b1;
end

// =========================================================================
// State transition
// =========================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= STATE_IDLE;
    else        state <= next_state;
end

always @(*) begin
    next_state = state;
    case (state)
        STATE_IDLE: if (mgmt_req) next_state = STATE_SEARCH;
        STATE_SEARCH: if (search_idx == NUM_REGIONS-1)
            case (mgmt_op)
                OP_GRANT:    next_state = STATE_GRANT;
                OP_REVOKE:   next_state = STATE_REVOKE;
                OP_RENEW:    next_state = STATE_RENEW;
                OP_QUERY:    next_state = STATE_QUERY;
                OP_TRANSFER: next_state = STATE_TRANSFER;
                default:     next_state = STATE_DONE;
            endcase
        STATE_GRANT, STATE_REVOKE, STATE_RENEW, STATE_QUERY, STATE_TRANSFER:
            next_state = STATE_DONE;
        STATE_DONE:
            next_state = STATE_IDLE;
        default:
            next_state = STATE_IDLE;
    endcase
end

// =========================================================================
// Search logic (uses only reads from arrays - safe in XSim)
// =========================================================================
integer j;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        search_idx  <= 0;
        free_slot   <= 0;
        found_slot  <= 0;
        found_free  <= 1'b0;
        found_lease <= 1'b0;
        mgmt_ack    <= 1'b0;
        mgmt_error  <= 1'b0;
        mgmt_token_out <= {TOKEN_WIDTH{1'b0}};
    end else begin
        mgmt_ack   <= 1'b0;
        mgmt_error <= 1'b0;

        case (state)
            STATE_IDLE: begin
                search_idx  <= 0;
                found_free  <= 1'b0;
                found_lease <= 1'b0;
            end
            STATE_SEARCH: begin
                if (!lease_valid_r[search_idx] && !found_free) begin
                    free_slot  <= search_idx;
                    found_free <= 1'b1;
                end
                if (lease_valid_r[search_idx] && lease_id_r[search_idx] == mgmt_lease_id) begin
                    found_slot  <= search_idx;
                    found_lease <= 1'b1;
                end
                search_idx <= search_idx + 1'b1;
            end
            STATE_GRANT: begin
                if (found_free && !found_lease) begin
                    mgmt_token_out <= lfsr;
                    mgmt_ack       <= 1'b1;
                end else begin
                    mgmt_error <= 1'b1;
                end
            end
            STATE_REVOKE: begin
                if (found_lease) begin
                    if (lease_owner_r[found_slot] == mgmt_client_id || 
                        lease_token_r[found_slot] == mgmt_token_in) begin
                        mgmt_ack <= 1'b1;
                    end else mgmt_error <= 1'b1;
                end else mgmt_error <= 1'b1;
            end
            STATE_RENEW: begin
                if (found_lease && lease_token_r[found_slot] == mgmt_token_in && 
                    lease_owner_r[found_slot] == mgmt_client_id) begin
                    mgmt_ack <= 1'b1;
                end else mgmt_error <= 1'b1;
            end
            STATE_QUERY: begin
                if (found_lease) begin
                    mgmt_token_out <= lease_token_r[found_slot];
                    mgmt_ack <= 1'b1;
                end else mgmt_error <= 1'b1;
            end
            STATE_TRANSFER: begin
                if (found_lease && lease_token_r[found_slot] == mgmt_token_in &&
                    lease_owner_r[found_slot] == mgmt_client_id) begin
                    mgmt_token_out <= lfsr;
                    mgmt_ack <= 1'b1;
                end else mgmt_error <= 1'b1;
            end
        endcase
    end
end

// =========================================================================
// Lease metadata writes - XSim workaround
//
// XSim silently drops writes to unpacked arrays when the index is a
// variable (reg/wire). Using per-entry generate blocks with constant
// genvar indices avoids this.
// =========================================================================
`ifdef SYNTHESIS
    // Synthesis: single always block, variable-indexed writes (Vivado is fine)
    always @(posedge clk or negedge rst_n) begin : meta_write_synth
        integer i;
        if (!rst_n) begin
            for (i = 0; i < NUM_REGIONS; i = i + 1) begin
                lease_valid_r[i]     <= 1'b0;
                lease_id_r[i]        <= {LEASE_ID_WIDTH{1'b0}};
                lease_owner_r[i]     <= {$clog2(NUM_CLIENTS){1'b0}};
                lease_base_addr_r[i] <= {ADDR_WIDTH{1'b0}};
                lease_size_r[i]      <= {ADDR_WIDTH{1'b0}};
                lease_token_r[i]     <= {TOKEN_WIDTH{1'b0}};
                lease_expiry_r[i]    <= 32'b0;
                lease_created_r[i]   <= 32'b0;
            end
        end else begin
            // Expiration
            for (i = 0; i < NUM_REGIONS; i = i + 1) begin
                if (lease_valid_r[i] && (lease_expiry_r[i] != 32'hFFFFFFFF)) begin
                    if (system_timer >= lease_expiry_r[i])
                        lease_valid_r[i] <= 1'b0;
                end
            end
            // Operations
            case (state)
                STATE_GRANT: begin
                    if (found_free && !found_lease) begin
                        lease_valid_r[free_slot]     <= 1'b1;
                        lease_id_r[free_slot]        <= mgmt_lease_id;
                        lease_owner_r[free_slot]     <= mgmt_client_id;
                        lease_base_addr_r[free_slot] <= mgmt_base_addr;
                        lease_size_r[free_slot]      <= mgmt_size;
                        lease_token_r[free_slot]     <= lfsr;
                        lease_created_r[free_slot]   <= system_timer;
                        if (mgmt_duration == 32'hFFFFFFFF)
                            lease_expiry_r[free_slot] <= 32'hFFFFFFFF;
                        else
                            lease_expiry_r[free_slot] <= system_timer + mgmt_duration;
                    end
                end
                STATE_REVOKE: begin
                    if (found_lease) begin
                        if (lease_owner_r[found_slot] == mgmt_client_id || 
                            lease_token_r[found_slot] == mgmt_token_in)
                            lease_valid_r[found_slot] <= 1'b0;
                    end
                end
                STATE_RENEW: begin
                    if (found_lease && lease_token_r[found_slot] == mgmt_token_in && 
                        lease_owner_r[found_slot] == mgmt_client_id) begin
                        if (mgmt_duration == 32'hFFFFFFFF)
                            lease_expiry_r[found_slot] <= 32'hFFFFFFFF;
                        else
                            lease_expiry_r[found_slot] <= system_timer + mgmt_duration;
                    end
                end
                STATE_TRANSFER: begin
                    if (found_lease && lease_token_r[found_slot] == mgmt_token_in &&
                        lease_owner_r[found_slot] == mgmt_client_id) begin
                        lease_owner_r[found_slot] <= mgmt_base_addr[$clog2(NUM_CLIENTS)-1:0];
                        lease_token_r[found_slot] <= lfsr;
                        if (mgmt_duration == 32'hFFFFFFFF)
                            lease_expiry_r[found_slot] <= 32'hFFFFFFFF;
                        else
                            lease_expiry_r[found_slot] <= system_timer + mgmt_duration;
                    end
                end
            endcase
        end
    end
`else
    // Simulation (XSim): per-entry generate blocks with constant indices
    genvar gi;
    generate
        for (gi = 0; gi < NUM_REGIONS; gi = gi + 1) begin : gen_lease_meta
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    lease_valid_r[gi]     <= 1'b0;
                    lease_id_r[gi]        <= {LEASE_ID_WIDTH{1'b0}};
                    lease_owner_r[gi]     <= {$clog2(NUM_CLIENTS){1'b0}};
                    lease_base_addr_r[gi] <= {ADDR_WIDTH{1'b0}};
                    lease_size_r[gi]      <= {ADDR_WIDTH{1'b0}};
                    lease_token_r[gi]     <= {TOKEN_WIDTH{1'b0}};
                    lease_expiry_r[gi]    <= 32'b0;
                    lease_created_r[gi]   <= 32'b0;
                end else begin
                    // Expiration
                    if (lease_valid_r[gi] && (lease_expiry_r[gi] != 32'hFFFFFFFF)) begin
                        if (system_timer >= lease_expiry_r[gi])
                            lease_valid_r[gi] <= 1'b0;
                    end
                    // Grant
                    if (state == STATE_GRANT && found_free && !found_lease &&
                        free_slot == gi[$clog2(NUM_REGIONS)-1:0]) begin
                        lease_valid_r[gi]     <= 1'b1;
                        lease_id_r[gi]        <= mgmt_lease_id;
                        lease_owner_r[gi]     <= mgmt_client_id;
                        lease_base_addr_r[gi] <= mgmt_base_addr;
                        lease_size_r[gi]      <= mgmt_size;
                        lease_token_r[gi]     <= lfsr;
                        lease_created_r[gi]   <= system_timer;
                        if (mgmt_duration == 32'hFFFFFFFF)
                            lease_expiry_r[gi] <= 32'hFFFFFFFF;
                        else
                            lease_expiry_r[gi] <= system_timer + mgmt_duration;
                    end
                    // Revoke
                    if (state == STATE_REVOKE && found_lease &&
                        found_slot == gi[$clog2(NUM_REGIONS)-1:0]) begin
                        if (lease_owner_r[gi] == mgmt_client_id || 
                            lease_token_r[gi] == mgmt_token_in)
                            lease_valid_r[gi] <= 1'b0;
                    end
                    // Renew
                    if (state == STATE_RENEW && found_lease &&
                        found_slot == gi[$clog2(NUM_REGIONS)-1:0] &&
                        lease_token_r[gi] == mgmt_token_in && 
                        lease_owner_r[gi] == mgmt_client_id) begin
                        if (mgmt_duration == 32'hFFFFFFFF)
                            lease_expiry_r[gi] <= 32'hFFFFFFFF;
                        else
                            lease_expiry_r[gi] <= system_timer + mgmt_duration;
                    end
                    // Transfer
                    if (state == STATE_TRANSFER && found_lease &&
                        found_slot == gi[$clog2(NUM_REGIONS)-1:0] &&
                        lease_token_r[gi] == mgmt_token_in &&
                        lease_owner_r[gi] == mgmt_client_id) begin
                        lease_owner_r[gi] <= mgmt_base_addr[$clog2(NUM_CLIENTS)-1:0];
                        lease_token_r[gi] <= lfsr;
                        if (mgmt_duration == 32'hFFFFFFFF)
                            lease_expiry_r[gi] <= 32'hFFFFFFFF;
                        else
                            lease_expiry_r[gi] <= system_timer + mgmt_duration;
                    end
                end
            end
        end
    endgenerate
`endif

// =========================================================================
// Init (simulation)
// =========================================================================
integer init_i;
initial begin
    for (init_i = 0; init_i < NUM_REGIONS; init_i = init_i + 1) begin
        lease_valid_r[init_i]     = 1'b0;
        lease_id_r[init_i]        = {LEASE_ID_WIDTH{1'b0}};
        lease_owner_r[init_i]     = {$clog2(NUM_CLIENTS){1'b0}};
        lease_base_addr_r[init_i] = {ADDR_WIDTH{1'b0}};
        lease_size_r[init_i]      = {ADDR_WIDTH{1'b0}};
        lease_token_r[init_i]     = {TOKEN_WIDTH{1'b0}};
        lease_expiry_r[init_i]    = 32'b0;
        lease_created_r[init_i]   = 32'b0;
    end
end

// =========================================================================
// Validation (combinational - reads only, safe in XSim)
// =========================================================================
integer i, k;
reg [NUM_CLIENTS-1:0] val_valid_comb;
reg [ADDR_WIDTH-1:0] addr_i;
reg [ADDR_WIDTH-1:0] end_addr_i; 
reg [TOKEN_WIDTH-1:0] token_i;
reg [LEASE_ID_WIDTH-1:0] lease_i;
reg [$clog2(NUM_CLIENTS)-1:0] client_id_i;

always @(*) begin
    val_valid_comb = {NUM_CLIENTS{1'b0}};
    for (i = 0; i < NUM_CLIENTS; i = i + 1) begin
        addr_i  = client_addr[i * ADDR_WIDTH +: ADDR_WIDTH];
        token_i = client_token[i * TOKEN_WIDTH +: TOKEN_WIDTH];
        lease_i = client_lease_id[i * LEASE_ID_WIDTH +: LEASE_ID_WIDTH];
        end_addr_i  = addr_i + 32'h00000FFF; 
        client_id_i = i[$clog2(NUM_CLIENTS)-1:0];
        for (k = 0; k < NUM_REGIONS; k = k + 1) begin
            if (lease_valid_r[k] &&
                lease_owner_r[k] == client_id_i &&
                lease_id_r[k] == lease_i &&
                lease_token_r[k] == token_i &&
                addr_i >= lease_base_addr_r[k] &&
                end_addr_i <  lease_base_addr_r[k] + lease_size_r[k] &&
                (lease_expiry_r[k] == 32'hFFFFFFFF || system_timer < lease_expiry_r[k])) begin
                    val_valid_comb[i] = 1'b1;
            end
        end
    end
end

assign val_valid = val_valid_comb;

// =========================================================================
// Active Leases
// =========================================================================
genvar m;
generate
    for (m = 0; m < NUM_REGIONS; m = m + 1) begin : gen_active
        assign active_leases[m] = lease_valid_r[m];
    end
endgenerate

endmodule