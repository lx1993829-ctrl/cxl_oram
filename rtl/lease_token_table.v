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
    
    // Management interface
    input  wire mgmt_req,
    input  wire [2:0] mgmt_op,              // Operation code
    input  wire [LEASE_ID_WIDTH-1:0] mgmt_lease_id,
    input  wire [$clog2(NUM_CLIENTS)-1:0] mgmt_client_id,
    input  wire [ADDR_WIDTH-1:0] mgmt_base_addr,
    input  wire [ADDR_WIDTH-1:0] mgmt_size,
    input  wire [31:0] mgmt_duration,
    input  wire [TOKEN_WIDTH-1:0] mgmt_token_in,
    output reg  mgmt_ack,
    output reg  mgmt_error,
    output reg  [TOKEN_WIDTH-1:0] mgmt_token_out,
    
    // Validation interface
    input  wire [NUM_CLIENTS*ADDR_WIDTH-1:0]   client_addr,
    input  wire [NUM_CLIENTS*TOKEN_WIDTH-1:0]  client_token,
    input  wire [NUM_CLIENTS*LEASE_ID_WIDTH-1:0] client_lease_id,
    output wire [NUM_CLIENTS-1:0] val_valid,
    
    // Status
    output wire [NUM_REGIONS-1:0] active_leases
);

//=========================================================================
// Operation codes
//=========================================================================
localparam OP_GRANT    = 3'd0;
localparam OP_REVOKE   = 3'd1;
localparam OP_RENEW    = 3'd2;
localparam OP_QUERY    = 3'd3;
localparam OP_TRANSFER = 3'd4;

//=========================================================================
// Lease entry structure (stored in registers)
//=========================================================================
reg lease_valid_r    [NUM_REGIONS-1:0];
reg [LEASE_ID_WIDTH-1:0] lease_id_r   [NUM_REGIONS-1:0];
reg [$clog2(NUM_CLIENTS)-1:0] lease_owner_r [NUM_REGIONS-1:0];
reg [ADDR_WIDTH-1:0] lease_base_addr_r [NUM_REGIONS-1:0];
reg [ADDR_WIDTH-1:0] lease_size_r      [NUM_REGIONS-1:0];
reg [TOKEN_WIDTH-1:0] lease_token_r    [NUM_REGIONS-1:0];
reg [31:0] lease_expiry_r   [NUM_REGIONS-1:0];
reg [31:0] lease_created_r  [NUM_REGIONS-1:0];

// System timer for lease expiration
reg [31:0] system_timer;

// LFSR for token generation
reg [TOKEN_WIDTH-1:0] lfsr;
wire [TOKEN_WIDTH-1:0] next_token;

//=========================================================================
// State machine for management operations
//=========================================================================
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

//=========================================================================
// Token Generation (LFSR-based)
//=========================================================================
assign next_token = {lfsr[TOKEN_WIDTH-2:0], 
                     lfsr[TOKEN_WIDTH-1] ^ lfsr[TOKEN_WIDTH-3] ^ lfsr[TOKEN_WIDTH-4] ^ lfsr[0]};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        lfsr <= {{TOKEN_WIDTH-1{1'b0}}, 1'b1}; // Non-zero seed
    end else begin
        lfsr <= next_token;
    end
end

//=========================================================================
// System Timer
//=========================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        system_timer <= 32'b0;
    end else begin
        system_timer <= system_timer + 1'b1;
    end
end

//=========================================================================
// Management State Machine
//=========================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state <= STATE_IDLE;
    else
        state <= next_state;
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

//=========================================================================
// Search and Operation Logic - WITH FIX
//=========================================================================
integer j;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (j = 0; j < NUM_REGIONS; j = j + 1) begin
            lease_valid_r[j]   <= 1'b0;
            lease_id_r[j]      <= {LEASE_ID_WIDTH{1'b0}};
            lease_owner_r[j]   <= {$clog2(NUM_CLIENTS){1'b0}};
            lease_base_addr_r[j] <= {ADDR_WIDTH{1'b0}};
            lease_size_r[j]    <= {ADDR_WIDTH{1'b0}};
            lease_token_r[j]   <= {TOKEN_WIDTH{1'b0}};
            lease_expiry_r[j]  <= 32'b0;
            lease_created_r[j] <= 32'b0;
        end
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

        // =====================================================================
        // SOLUTION 1: FIXED AUTOMATIC LEASE EXPIRATION
        // Don't expire a lease we're currently granting in this cycle
        // =====================================================================
        /*
        for (j = 0; j < NUM_REGIONS; j = j + 1) begin
            if (lease_valid_r[j] && (system_timer >= lease_expiry_r[j])) begin
                // Don't expire a lease we're currently granting
                if (!(state == STATE_GRANT && found_free && j == free_slot)) begin
                    lease_valid_r[j] <= 1'b0;
                end
            end
        end
        */
        // Automatic lease expiration logic modification
        for (j = 0; j < NUM_REGIONS; j = j + 1) begin
            // Only expire if valid AND duration wasn't set to 'infinity' (32'hFFFFFFFF)
            if (lease_valid_r[j] && (lease_expiry_r[j] != 32'hFFFFFFFF)) begin
                if (system_timer >= lease_expiry_r[j])
                    lease_valid_r[j] <= 1'b0;
            end
        end
        
        case (state)
            STATE_IDLE: begin
                search_idx  <= 0;
                found_free  <= 1'b0;
                found_lease <= 1'b0;
            end
            STATE_SEARCH: begin
                // Search for free slot and matching lease
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
                    lease_valid_r[free_slot]   <= 1'b1;
                    lease_id_r[free_slot]      <= mgmt_lease_id;
                    lease_owner_r[free_slot]   <= mgmt_client_id;
                    lease_base_addr_r[free_slot] <= mgmt_base_addr;
                    lease_size_r[free_slot]    <= mgmt_size;
                    lease_token_r[free_slot]   <= lfsr;
                    lease_created_r[free_slot] <= system_timer;
                    
                    // FIX: Check for "Infinite" duration BEFORE adding to timer
                    if (mgmt_duration == 32'hFFFFFFFF) begin
                        lease_expiry_r[free_slot] <= 32'hFFFFFFFF;
                    end else begin
                        lease_expiry_r[free_slot] <= system_timer + mgmt_duration;
                    end
            
                    mgmt_token_out <= lfsr;
                    mgmt_ack       <= 1'b1;
                end else begin
                    mgmt_error <= 1'b1;
                end
            end
        STATE_REVOKE: begin
            if (found_lease) begin
                // Allow revoke if either the client ID matches OR the correct token is provided
                if (lease_owner_r[found_slot] == mgmt_client_id || 
                    lease_token_r[found_slot] == mgmt_token_in) begin
                    lease_valid_r[found_slot] <= 1'b0;
                    mgmt_ack <= 1'b1;
                end else mgmt_error <= 1'b1;
            end else mgmt_error <= 1'b1;
        end

        STATE_RENEW: begin
            if (found_lease && lease_token_r[found_slot] == mgmt_token_in && 
                lease_owner_r[found_slot] == mgmt_client_id) begin
                
                // FIX: Apply the same infinity check as STATE_GRANT
                if (mgmt_duration == 32'hFFFFFFFF) begin
                    lease_expiry_r[found_slot] <= 32'hFFFFFFFF;
                end else begin
                    lease_expiry_r[found_slot] <= system_timer + mgmt_duration;
                end
                
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
                
                // Update owner (using mgmt_base_addr as a container for the new ID)
                lease_owner_r[found_slot] <= mgmt_base_addr[$clog2(NUM_CLIENTS)-1:0];
                
                // Generate a new token for the new owner
                lease_token_r[found_slot] <= lfsr;
                mgmt_token_out            <= lfsr;
                
                // Optional: Reset the expiry on transfer if mgmt_duration is provided
                if (mgmt_duration == 32'hFFFFFFFF) begin
                    lease_expiry_r[found_slot] <= 32'hFFFFFFFF;
                end else begin
                    lease_expiry_r[found_slot] <= system_timer + mgmt_duration;
                end
                
                mgmt_ack <= 1'b1;
            end else mgmt_error <= 1'b1;
        end
        endcase
    end
end

//=========================================================================
// Validation Logic (combinational for fast access checking)
//=========================================================================
integer i, k;
reg [NUM_CLIENTS-1:0] val_valid_comb;  // Renamed to avoid confusion
// Extract flattened client inputs
reg [ADDR_WIDTH-1:0] addr_i;
reg [ADDR_WIDTH-1:0] end_addr_i; 
reg [TOKEN_WIDTH-1:0] token_i;
reg [LEASE_ID_WIDTH-1:0] lease_i;
reg [$clog2(NUM_CLIENTS)-1:0] client_id_i;  // same width as lease_owner_r

always @(*) begin
    // Initialize all outputs to 0
    val_valid_comb = {NUM_CLIENTS{1'b0}};
    
    for (i = 0; i < NUM_CLIENTS; i = i + 1) begin
        // Use the +: operator for variable indexing with a constant width
        addr_i  = client_addr[i * ADDR_WIDTH +: ADDR_WIDTH];
        token_i = client_token[i * TOKEN_WIDTH +: TOKEN_WIDTH];
        lease_i = client_lease_id[i * LEASE_ID_WIDTH +: LEASE_ID_WIDTH];
        end_addr_i  = addr_i + 32'h00000FFF; 
        client_id_i = i[$clog2(NUM_CLIENTS)-1:0];
        for (k = 0; k < NUM_REGIONS; k = k + 1) begin
            // Check the REGISTER ARRAY elements, not the local variable!
            if (lease_valid_r[k] &&
                lease_owner_r[k] == client_id_i &&
                lease_id_r[k] == lease_i &&
                lease_token_r[k] == token_i &&
                addr_i >= lease_base_addr_r[k] &&
                end_addr_i <  lease_base_addr_r[k] + lease_size_r[k] &&
                (lease_expiry_r[k] == 32'hFFFFFFFF || system_timer < lease_expiry_r[k])) begin // FIX HERE
                    val_valid_comb[i] = 1'b1;
            end
        end
    end
end

assign val_valid = val_valid_comb;

//=========================================================================
// Active Leases Status
//=========================================================================
genvar m;
generate
    for (m = 0; m < NUM_REGIONS; m = m + 1) begin : gen_active
        assign active_leases[m] = lease_valid_r[m];
    end
endgenerate

endmodule
