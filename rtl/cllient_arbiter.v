module client_arbiter #(
    parameter NUM_CLIENTS = 2,
    parameter ADDR_WIDTH  = 32,
    parameter DATA_WIDTH  = 128
)(
    input  wire                              clk,
    input  wire                              rst_n,
    input  wire [NUM_CLIENTS-1:0]            client_req,
    input  wire [NUM_CLIENTS-1:0]            client_wr_en,
    input  wire [ADDR_WIDTH*NUM_CLIENTS-1:0] client_addr,
    input  wire [DATA_WIDTH*NUM_CLIENTS-1:0] client_wdata,
    output reg  [$clog2(NUM_CLIENTS)-1:0]    grant_id,
    output reg                               grant_req,
    output reg                               grant_wr_en,
    output reg  [ADDR_WIDTH-1:0]             grant_addr,
    output reg  [DATA_WIDTH-1:0]             grant_wdata,
    input  wire                              mem_ready,
    input  wire                              mem_ack,
    input  wire [DATA_WIDTH-1:0]             mem_rdata
);

    localparam STATE_IDLE   = 2'd0;
    localparam STATE_GRANT  = 2'd1;
    localparam STATE_ACTIVE = 2'd2;

    reg [1:0] state;
    reg active_settled;   // prevents exit on first cycle of STATE_ACTIVE

    reg [$clog2(NUM_CLIENTS)-1:0] last_grant;
    reg [$clog2(NUM_CLIENTS)-1:0] next_grant;
    reg [$clog2(NUM_CLIENTS)-1:0] active_client;
    reg [$clog2(NUM_CLIENTS)-1:0] latched_grant;
    reg [NUM_CLIENTS-1:0]         priority_mask;
    reg [NUM_CLIENTS-1:0]         latched_wr_en;
    reg [ADDR_WIDTH*NUM_CLIENTS-1:0] latched_addr;

    wire [NUM_CLIENTS-1:0] masked_req;
    wire                   any_masked_req;
    wire                   any_req;

    integer i;
    always @(*) begin
        priority_mask = {NUM_CLIENTS{1'b0}};
        for (i = 0; i < NUM_CLIENTS; i = i + 1)
            if (i > last_grant)
                priority_mask[i] = 1'b1;
    end

    assign masked_req     = client_req & priority_mask;
    assign any_masked_req = |masked_req;
    assign any_req        = |client_req;

    function [$clog2(NUM_CLIENTS)-1:0] find_first_one;
        input [NUM_CLIENTS-1:0] req_vec;
        integer j;
        begin
            find_first_one = 0;
            for (j = NUM_CLIENTS-1; j >= 0; j = j - 1)
                if (req_vec[j])
                    find_first_one = j[$clog2(NUM_CLIENTS)-1:0];
        end
    endfunction

    always @(*) begin
        if      (any_masked_req) next_grant = find_first_one(masked_req);
        else if (any_req)        next_grant = find_first_one(client_req);
        else                     next_grant = last_grant;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= STATE_IDLE;
            active_settled <= 1'b0;
            last_grant     <= {$clog2(NUM_CLIENTS){1'b0}};
            latched_grant  <= {$clog2(NUM_CLIENTS){1'b0}};
            grant_id       <= {$clog2(NUM_CLIENTS){1'b0}};
            grant_req      <= 1'b0;
            grant_wr_en    <= 1'b0;
            grant_addr     <= {ADDR_WIDTH{1'b0}};
            grant_wdata    <= {DATA_WIDTH{1'b0}};
            active_client  <= {$clog2(NUM_CLIENTS){1'b0}};
            latched_wr_en  <= {NUM_CLIENTS{1'b0}};
            latched_addr   <= {ADDR_WIDTH*NUM_CLIENTS{1'b0}};
        end else begin
            case (state)

                STATE_IDLE: begin
                    grant_req <= 1'b0;
                    if (any_req) begin   // no mem_ready check needed - if we're in IDLE we're ready
                        latched_grant <= next_grant;
                        latched_wr_en <= client_wr_en;
                        latched_addr  <= client_addr;
                        state         <= STATE_GRANT;
                    end
                end

                STATE_GRANT: begin
                    grant_id       <= latched_grant;
                    grant_req      <= 1'b1;
                    grant_wr_en    <= latched_wr_en[latched_grant];
                    grant_addr     <= latched_addr[latched_grant * ADDR_WIDTH +: ADDR_WIDTH];
                    grant_wdata    <= client_wdata[latched_grant * DATA_WIDTH +: DATA_WIDTH];
                    active_client  <= latched_grant;
                    last_grant     <= latched_grant;
                    active_settled <= 1'b0;   // reset guard for STATE_ACTIVE
                    state          <= STATE_ACTIVE;
                end

                STATE_ACTIVE: begin
                    
                    grant_req   <= 1'b1;
                    grant_wdata <= client_wdata[active_client * DATA_WIDTH +: DATA_WIDTH];
                
                    if (mem_ready) begin   // mem_ready is now txn_done_pulse
                        $display("[%0t] [ARB] STATE_ACTIVE releasing on txn_done_pulse", $time);
                        grant_req <= 1'b0;
                        state     <= STATE_IDLE;
                    end
                end

                default: state <= STATE_IDLE;

            endcase
        end
    end

endmodule

/*
`timescale 1ns / 1ps
//=============================================================================

// Client Arbiter

// Round-robin arbiter with priority support for multiple memory clients

//=============================================================================

module client_arbiter #(
parameter NUM_CLIENTS = 2,
parameter ADDR_WIDTH = 32,
parameter DATA_WIDTH = 64
)(
input wire clk,
input wire rst_n,
// Client requests
input wire [NUM_CLIENTS-1:0] client_req,
input wire [NUM_CLIENTS-1:0] client_wr_en,
input wire [ADDR_WIDTH*NUM_CLIENTS-1:0] client_addr,

input wire [DATA_WIDTH*NUM_CLIENTS-1:0] client_wdata,


// Granted client output

output reg [$clog2(NUM_CLIENTS)-1:0] grant_id,

output reg grant_req,

output reg grant_wr_en,

output reg [ADDR_WIDTH-1:0] grant_addr,

output reg [DATA_WIDTH-1:0] grant_wdata,


// Memory interface feedback

input wire mem_ready,

input wire mem_ack,

input wire [DATA_WIDTH-1:0] mem_rdata


// Client acknowledgments

//output reg [NUM_CLIENTS-1:0] client_ack,

//output wire [DATA_WIDTH*NUM_CLIENTS-1:0] client_rdata 

);



//=========================================================================

// State Machine

//=========================================================================

localparam STATE_IDLE = 2'd0;

localparam STATE_GRANT = 2'd1;

localparam STATE_WAIT = 2'd2;

localparam STATE_RESPOND = 2'd3;


reg [1:0] state, next_state;


//=========================================================================

// Internal Signals

//=========================================================================


reg [$clog2(NUM_CLIENTS)-1:0] last_grant;

reg [$clog2(NUM_CLIENTS)-1:0] next_grant;

reg [NUM_CLIENTS-1:0] priority_mask;

reg [DATA_WIDTH-1:0] rdata_reg;

reg [$clog2(NUM_CLIENTS)-1:0] active_client;


// Find next requester (round-robin)

wire [NUM_CLIENTS-1:0] masked_req;

wire [NUM_CLIENTS-1:0] unmasked_grant;

wire [NUM_CLIENTS-1:0] masked_grant;

wire any_masked_req;

wire any_req;


//=========================================================================

// Round-Robin Priority Mask

//=========================================================================


// Generate priority mask - clients after last_grant have priority

integer i;

always @(*) begin

priority_mask = {NUM_CLIENTS{1'b0}};

for (i = 0; i < NUM_CLIENTS; i = i + 1) begin

if (i > last_grant) begin

priority_mask[i] = 1'b1;

end

end

end


assign masked_req = client_req & priority_mask;

assign any_masked_req = |masked_req;

assign any_req = |client_req;


//=========================================================================

// Grant Selection Logic (Priority Encoder)

//=========================================================================


// Find lowest numbered requester in masked requests

function [$clog2(NUM_CLIENTS)-1:0] find_first_one;

input [NUM_CLIENTS-1:0] req_vec;

integer j;

begin

find_first_one = 0;

for (j = NUM_CLIENTS-1; j >= 0; j = j - 1) begin

if (req_vec[j]) begin

find_first_one = j[$clog2(NUM_CLIENTS)-1:0];

end

end

end

endfunction


always @(*) begin

if (any_masked_req) begin

next_grant = find_first_one(masked_req);

end else if (any_req) begin

next_grant = find_first_one(client_req);

end else begin

next_grant = last_grant;

end

end


//=========================================================================

// State Machine - Sequential

//=========================================================================


always @(posedge clk or negedge rst_n) begin

if (!rst_n) begin

state <= STATE_IDLE;

end else begin

state <= next_state;

end

end


//=========================================================================

// State Machine - Combinational

//=========================================================================

always @(*) begin
    next_state = state;
    
    case (state)
        STATE_IDLE: begin
            if (any_req && mem_ready) begin
                next_state = STATE_GRANT;
            end
        end
        
        STATE_GRANT: begin
            next_state = STATE_WAIT;
        end
        
        STATE_WAIT: begin
            if (!client_req[active_client]) begin
                next_state = STATE_IDLE;  // ? Keep this here
            end else if (mem_ack) begin
                next_state = STATE_RESPOND;  // ? Keep this here
            end
        end
        
        STATE_RESPOND: begin
            next_state = STATE_IDLE;
        end
        
        default: next_state = STATE_IDLE;
    endcase
end


//=========================================================================

// Datapath Logic

//=========================================================================


integer k;


always @(posedge clk or negedge rst_n) begin

if (!rst_n) begin

last_grant <= {$clog2(NUM_CLIENTS){1'b0}};

grant_id <= {$clog2(NUM_CLIENTS){1'b0}};

grant_req <= 1'b0;

grant_wr_en <= 1'b0;

grant_addr <= {ADDR_WIDTH{1'b0}};

grant_wdata <= {DATA_WIDTH{1'b0}};

//client_ack <= {NUM_CLIENTS{1'b0}};

rdata_reg <= {DATA_WIDTH{1'b0}};

active_client <= {$clog2(NUM_CLIENTS){1'b0}};


end else begin

// Default values

grant_req <= 1'b0;

//client_ack <= {NUM_CLIENTS{1'b0}};


case (state)

STATE_IDLE: begin

// Ready for new arbitration

end


STATE_GRANT: begin

// Grant access to selected client

grant_id <= next_grant;

grant_req <= 1'b1;

grant_wr_en <= client_wr_en[next_grant];

grant_addr    <= client_addr[next_grant * ADDR_WIDTH +: ADDR_WIDTH];
grant_wdata   <= client_wdata[next_grant * DATA_WIDTH +: DATA_WIDTH];

active_client <= next_grant;

last_grant <= next_grant;

end


STATE_WAIT: begin
     if (mem_ack) begin
            rdata_reg <= mem_rdata;  // ADD THIS LINE
     end
end


STATE_RESPOND: begin

// Send acknowledgment to the granted client

//client_ack[active_client] <= 1'b1;

end

endcase

end

end


//=========================================================================

// Read Data Distribution

//=========================================================================


// Broadcast read data to all clients (they only use it when acked)

//genvar g;

//generate
//    for (g = 0; g < NUM_CLIENTS; g = g + 1) begin : gen_rdata
//        assign client_rdata[g*DATA_WIDTH +: DATA_WIDTH] = rdata_reg;
//    end
//endgenerate
endmodule
*/