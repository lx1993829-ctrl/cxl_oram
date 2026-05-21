`include "oram_params.vh"
module pos_map #(
    parameter N          = `ORAM_N,
    parameter BUCKET_W   = `BUCKET_ID_W,
    parameter SLOT_W     = `SLOT_ID_W,
    parameter ADDR_W     = `SLOT_ADDR_W,
    parameter WORD_W     = BUCKET_W + 2
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [ADDR_W-1:0]    rd_slot_addr,
    input  wire                 rd_en,
    output reg  [BUCKET_W-1:0]  rd_bucket,
    output reg  [1:0]           rd_status,
    output reg                  rd_valid,
    input  wire [ADDR_W-1:0]    wr_slot_addr,
    input  wire [BUCKET_W-1:0]  wr_bucket,
    input  wire [1:0]           wr_status,
    input  wire                 wr_en
);
    (* ram_style = "block" *)
    reg [WORD_W-1:0] mem [0:N-1];

    localparam IDX_W = $clog2(N);
    wire [IDX_W-1:0] rd_idx = rd_slot_addr[IDX_W+11:12];
    wire [IDX_W-1:0] wr_idx = wr_slot_addr[IDX_W+11:12];

    // BRAM read - NO async reset on data outputs (enables BRAM inference)
    always @(posedge clk) begin
        if (rd_en) begin
            rd_bucket <= mem[rd_idx][BUCKET_W-1:0];
            rd_status <= mem[rd_idx][WORD_W-1:BUCKET_W];
        end
    end

    // Valid flag - can have async reset (just a flip-flop)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_valid <= 1'b0;
        else
            rd_valid <= rd_en;
    end

    always @(posedge clk) begin
        if (wr_en)
            mem[wr_idx] <= {wr_status, wr_bucket};
    end

    integer i;
    initial begin
        for (i = 0; i < N; i = i + 1)
            mem[i] = {`ST_DUMMY, {BUCKET_W{1'b0}}};
    end
endmodule