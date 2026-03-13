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
