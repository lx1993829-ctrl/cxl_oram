`include "oram_params.vh"

module stash #(
    parameter DEPTH      = `ORAM_STASH_DEPTH,
    parameter PTR_W      = `STASH_PTR_W,
    parameter SLOT_AW    = `SLOT_ADDR_W,
    parameter BUCKET_W   = `BUCKET_ID_W,
    parameter AXI_DATA_W = `AXI_DATA_W,
    parameter BEATS      = `BEATS_PER_BLOCK,
    parameter BEAT_W     = `BEAT_CNT_W
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // CAM Port 1: find by slot_addr
    input  wire [SLOT_AW-1:0]       cam_slot_addr,
    input  wire                     cam_slot_en,
    output wire                     cam_slot_hit,
    output wire [PTR_W-1:0]         cam_slot_idx,

    // CAM Port 2: find by target_bucket
    input  wire [BUCKET_W-1:0]      cam_bkt_target,
    input  wire                     cam_bkt_en,
    output wire                     cam_bkt_hit,
    output wire [PTR_W-1:0]         cam_bkt_idx,
    output wire [SLOT_AW-1:0]       cam_bkt_slot_addr,
    output wire                     cam_bkt_inhibit_busy,

    // Insert Port
    input  wire [SLOT_AW-1:0]       ins_slot_addr,
    input  wire [BUCKET_W-1:0]      ins_target_bucket,
    input  wire                     ins_en,
    output wire [PTR_W-1:0]         ins_idx,
    output wire                     ins_full,

    // Remap Port
    input  wire [PTR_W-1:0]         remap_idx,
    input  wire [BUCKET_W-1:0]      remap_new_bucket,
    input  wire                     remap_en,

    // Evict Port
    input  wire [PTR_W-1:0]         evict_idx,
    input  wire                     evict_en,

    // Data Write Port
    input  wire [PTR_W-1:0]         dwr_entry,
    input  wire [BEAT_W-1:0]        dwr_beat,
    input  wire [AXI_DATA_W-1:0]    dwr_data,
    input  wire                     dwr_en,

    // Data Read Port
    input  wire [PTR_W-1:0]         drd_entry,
    input  wire [BEAT_W-1:0]        drd_beat,
    input  wire                     drd_en,
    output reg  [AXI_DATA_W-1:0]    drd_data,
    output reg                      drd_valid,

    // Status
    output wire [PTR_W:0]           occupancy
);

    // =========================================================================
    // Metadata storage (unpacked regs for TB force/release compat)
    // =========================================================================
    reg              valid_r  [0:DEPTH-1];
    reg [SLOT_AW-1:0]    slot_r   [0:DEPTH-1];
    reg [BUCKET_W-1:0]   target_r [0:DEPTH-1];

    // =========================================================================
    // Data memory - URAM (all entries on-chip, direct random access)
    // =========================================================================
    (* ram_style = "ultra" *)
    reg [AXI_DATA_W-1:0] data_mem [0:(DEPTH * BEATS)-1];

    wire [PTR_W+BEAT_W-1:0] dwr_addr = {dwr_entry, dwr_beat};
    wire [PTR_W+BEAT_W-1:0] drd_addr = {drd_entry, drd_beat};

    // -------------------------------------------------------------------------
    // Eviction inhibit (1-cycle mask after evict_en)
    // -------------------------------------------------------------------------
    reg              evict_inhibit_active;
    reg [PTR_W-1:0]  evict_inhibit_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            evict_inhibit_active <= 1'b0;
            evict_inhibit_idx    <= {PTR_W{1'b0}};
        end else begin
            evict_inhibit_active <= evict_en;
            evict_inhibit_idx    <= evict_idx;
        end
    end

    assign cam_bkt_inhibit_busy = evict_inhibit_active;

    // -------------------------------------------------------------------------
    // CAM: find by slot_addr (combinational)
    // -------------------------------------------------------------------------
    reg  [DEPTH-1:0] slot_match;
    integer          j;
    always @(*) begin
        for (j = 0; j < DEPTH; j = j + 1)
            slot_match[j] = valid_r[j] && (slot_r[j] == cam_slot_addr);
    end

    wire [DEPTH-1:0] cam_slot_match_gated = cam_slot_en ? slot_match : {DEPTH{1'b0}};
    wire             cam_slot_hit_raw;
    wire [PTR_W-1:0] slot_enc;
    priority_enc #(.W(DEPTH), .OUT_W(PTR_W)) u_slot_penc (
        .in(cam_slot_match_gated), .out(slot_enc), .hit(cam_slot_hit_raw)
    );
    assign cam_slot_hit = cam_slot_hit_raw & cam_slot_en;
    assign cam_slot_idx = slot_enc;

    // -------------------------------------------------------------------------
    // CAM: find by target_bucket (combinational)
    // -------------------------------------------------------------------------
    (* max_fanout = 16 *)
    reg [BUCKET_W-1:0] cam_bkt_target_buf;
    always @(*) cam_bkt_target_buf = cam_bkt_target;

    reg  [DEPTH-1:0] bkt_match;
    always @(*) begin
        for (j = 0; j < DEPTH; j = j + 1)
            bkt_match[j] = valid_r[j] && (target_r[j] == cam_bkt_target_buf);
    end

    genvar gi;
    wire [DEPTH-1:0] cam_bkt_inhibit_mask;
    generate
        for (gi = 0; gi < DEPTH; gi = gi + 1) begin : gen_inhibit
            assign cam_bkt_inhibit_mask[gi] =
                (evict_inhibit_active && (evict_inhibit_idx == gi[PTR_W-1:0])) ? 1'b0 : 1'b1;
        end
    endgenerate

    wire [DEPTH-1:0] cam_bkt_match_gated =
        cam_bkt_en ? (bkt_match & cam_bkt_inhibit_mask) : {DEPTH{1'b0}};
    wire             cam_bkt_hit_raw;
    wire [PTR_W-1:0] bkt_enc;
    priority_enc #(.W(DEPTH), .OUT_W(PTR_W)) u_bkt_penc (
        .in(cam_bkt_match_gated), .out(bkt_enc), .hit(cam_bkt_hit_raw)
    );
    assign cam_bkt_hit       = cam_bkt_hit_raw & cam_bkt_en;
    assign cam_bkt_idx       = bkt_enc;
    assign cam_bkt_slot_addr = slot_r[bkt_enc];

    // -------------------------------------------------------------------------
    // Free slot finder
    // -------------------------------------------------------------------------
    reg  [DEPTH-1:0] free_mask;
    always @(*) begin
        for (j = 0; j < DEPTH; j = j + 1)
            free_mask[j] = ~valid_r[j];
    end

    wire [PTR_W-1:0] free_enc;
    wire             free_hit;
    priority_enc #(.W(DEPTH), .OUT_W(PTR_W)) u_free_penc (
        .in(free_mask), .out(free_enc), .hit(free_hit)
    );
    assign ins_idx  = free_enc;
    assign ins_full = !free_hit;

    // -------------------------------------------------------------------------
    // Occupancy counter
    // -------------------------------------------------------------------------
    reg [PTR_W:0] occ_r;
    assign occupancy = occ_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            occ_r <= {(PTR_W+1){1'b0}};
        end else begin
            if (evict_en && !(ins_en && !ins_full))
                occ_r <= occ_r - 1'b1;
            else if (!evict_en && (ins_en && !ins_full))
                occ_r <= occ_r + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // Power-on initialisation (simulation only)
    // -------------------------------------------------------------------------
    integer init_i;
    initial begin
        occ_r = {(PTR_W+1){1'b0}};
        for (init_i = 0; init_i < DEPTH; init_i = init_i + 1) begin
            valid_r[init_i]  = 1'b0;
            slot_r[init_i]   = {SLOT_AW{1'b0}};
            target_r[init_i] = {BUCKET_W{1'b0}};
        end
        for (init_i = 0; init_i < DEPTH * BEATS; init_i = init_i + 1)
            data_mem[init_i] = {AXI_DATA_W{1'b0}};
    end

    // =========================================================================
    // Metadata write
    // =========================================================================
`ifdef SYNTHESIS
    always @(posedge clk or negedge rst_n) begin : meta_write_synth
        integer i;
        if (!rst_n) begin
            for (i = 0; i < DEPTH; i = i + 1) begin
                valid_r[i]  <= 1'b0;
                slot_r[i]   <= {SLOT_AW{1'b0}};
                target_r[i] <= {BUCKET_W{1'b0}};
            end
        end else begin
            if (evict_en)
                valid_r[evict_idx] <= 1'b0;
            if (ins_en && !ins_full) begin
                valid_r[free_enc]  <= 1'b1;
                slot_r[free_enc]   <= ins_slot_addr;
                target_r[free_enc] <= ins_target_bucket;
            end
            if (remap_en)
                target_r[remap_idx] <= remap_new_bucket;
        end
    end
`else
    generate
        for (gi = 0; gi < DEPTH; gi = gi + 1) begin : gen_meta
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    valid_r[gi]  <= 1'b0;
                    slot_r[gi]   <= {SLOT_AW{1'b0}};
                    target_r[gi] <= {BUCKET_W{1'b0}};
                end else begin
                    if (evict_en && (evict_idx == gi[PTR_W-1:0]))
                        valid_r[gi] <= 1'b0;
                    if (ins_en && !ins_full && (free_enc == gi[PTR_W-1:0])) begin
                        valid_r[gi]  <= 1'b1;
                        slot_r[gi]   <= ins_slot_addr;
                        target_r[gi] <= ins_target_bucket;
                    end
                    if (remap_en && (remap_idx == gi[PTR_W-1:0]))
                        target_r[gi] <= remap_new_bucket;
                end
            end
        end
    endgenerate
`endif

    // -------------------------------------------------------------------------
    // Data memory write
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (dwr_en)
            data_mem[dwr_addr] <= dwr_data;
    end

    // -------------------------------------------------------------------------
    // Data memory read - NO async reset on data (enables URAM inference)
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (drd_en)
            drd_data <= data_mem[drd_addr];
    end

    // Valid flag - can have async reset (just a flip-flop)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            drd_valid <= 1'b0;
        else
            drd_valid <= drd_en;
    end

endmodule


// =============================================================================
// priority_enc - Parameterized Priority Encoder (LSB wins)
// =============================================================================
module priority_enc #(
    parameter W     = 128,
    parameter OUT_W = 7
)(
    input  wire [W-1:0]     in,
    output reg  [OUT_W-1:0] out,
    output wire             hit
);
    assign hit = |in;
    integer i;
    always @(*) begin
        out = {OUT_W{1'b0}};
        for (i = W-1; i >= 0; i = i - 1)
            if (in[i]) out = i[OUT_W-1:0];
    end
endmodule