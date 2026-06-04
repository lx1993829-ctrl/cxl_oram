// =============================================================================
// pos_map.v - Position Map (v2 - HBM-backed, full AXI4 master)
// =============================================================================
// Same functional interface as v1 (BRAM version):
//   - rd_en / rd_bucket / rd_status / rd_valid  (read lookup)
//   - wr_en / wr_bucket / wr_status              (write update)
//   - init_mode / init_pm_wr_*                    (bulk init)
//
// Differences from v1:
//   - Data stored in HBM, not BRAM. Scales to millions of entries.
//   - Read latency: ~10-20 cycles (HBM round-trip) instead of 1.
//   - busy output: FSM must gate rd_en/wr_en on !busy.
//   - Full AXI4 master interface for HBM access.
//
// HBM layout: entries packed 16 per AXI beat (each padded to 16 bits).
//   beat_addr = PM_BASE + (slot_index / 16) * 32    (PM_BASE = 0x14000000)
//
// Read:  single-beat AXI4 read  (ARLEN=0), extract 16-bit entry from beat.
// Write: single-beat AXI4 write (AWLEN=0), WSTRB masks the 2 target bytes.
// Init:  same AXI write path, active when init_mode=1.
// =============================================================================
`include "oram_params.vh"

module pos_map #(
    parameter N          = `ORAM_N,
    parameter BUCKET_W   = `BUCKET_ID_W,
    parameter SLOT_W     = `SLOT_ID_W,
    parameter ADDR_W     = `SLOT_ADDR_W,
    parameter AXI_DW     = `AXI_DATA_W,
    parameter AXI_AW     = `AXI_ADDR_W,
    parameter AXI_SW     = `AXI_STRB_W,
    parameter AXI_IDW    = `AXI_ID_W,
    parameter AXI_LENW   = `AXI_LEN_W,
    parameter WORD_W     = BUCKET_W + 2,
    parameter PACK_W     = 16,
    parameter EPB        = AXI_DW / PACK_W,     // 16 entries per beat
    parameter EPB_W      = $clog2(EPB),          // 4
    parameter ENTRY_BYTES = PACK_W / 8,          // 2
    parameter BEAT_BYTES = AXI_DW / 8,           // 32
    parameter PM_BASE    = 34'h0_1400_0000
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Read port (same as v1)
    input  wire [ADDR_W-1:0]    rd_slot_addr,
    input  wire                 rd_en,
    output reg  [BUCKET_W-1:0]  rd_bucket,
    output reg  [1:0]           rd_status,
    output reg                  rd_valid,

    // Write port (same as v1)
    input  wire [ADDR_W-1:0]    wr_slot_addr,
    input  wire [BUCKET_W-1:0]  wr_bucket,
    input  wire [1:0]           wr_status,
    input  wire                 wr_en,

    // Init port (same as v1 — active when init_mode=1)
    input  wire                 init_mode,
    input  wire [ADDR_W-1:0]    init_pm_wr_addr,
    input  wire [BUCKET_W-1:0]  init_pm_wr_bucket,
    input  wire [1:0]           init_pm_wr_status,
    input  wire                 init_pm_wr_en,

    // Busy (new in v2 — not in v1)
    output wire                 busy,
    // Debug
    output wire [2:0]           dbg_state,

    // Full AXI4 Master Interface → HBM
    output reg  [AXI_IDW-1:0]   m_axi_arid,
    output reg  [AXI_AW-1:0]    m_axi_araddr,
    output reg  [AXI_LENW-1:0]  m_axi_arlen,
    output reg  [2:0]            m_axi_arsize,
    output reg  [1:0]            m_axi_arburst,
    output reg                   m_axi_arvalid,
    input  wire                  m_axi_arready,

    input  wire [AXI_IDW-1:0]   m_axi_rid,
    input  wire [AXI_DW-1:0]    m_axi_rdata,
    input  wire [1:0]            m_axi_rresp,
    input  wire                  m_axi_rlast,
    input  wire                  m_axi_rvalid,
    output reg                   m_axi_rready,

    output reg  [AXI_IDW-1:0]   m_axi_awid,
    output reg  [AXI_AW-1:0]    m_axi_awaddr,
    output reg  [AXI_LENW-1:0]  m_axi_awlen,
    output reg  [2:0]            m_axi_awsize,
    output reg  [1:0]            m_axi_awburst,
    output reg                   m_axi_awvalid,
    input  wire                  m_axi_awready,

    output reg  [AXI_DW-1:0]    m_axi_wdata,
    output reg  [AXI_SW-1:0]    m_axi_wstrb,
    output reg                   m_axi_wlast,
    output reg                   m_axi_wvalid,
    input  wire                  m_axi_wready,

    input  wire [AXI_IDW-1:0]   m_axi_bid,
    input  wire [1:0]            m_axi_bresp,
    input  wire                  m_axi_bvalid,
    output reg                   m_axi_bready
);

    // =========================================================================
    // Constants
    // =========================================================================
    localparam IDX_W     = $clog2(N);
    localparam AXI_SIZE  = 3'd5;                 // 32 bytes per beat
    localparam AXI_BURST = 2'b01;                // INCR

    // =========================================================================
    // Address decode
    // =========================================================================
    // Mux between init and normal write ports
    wire [ADDR_W-1:0]    wr_addr_mux    = init_mode ? init_pm_wr_addr   : wr_slot_addr;
    wire [BUCKET_W-1:0]  wr_bucket_mux  = init_mode ? init_pm_wr_bucket : wr_bucket;
    wire [1:0]           wr_status_mux  = init_mode ? init_pm_wr_status : wr_status;
    wire                 wr_en_mux      = init_mode ? init_pm_wr_en     : wr_en;

    // Registered addresses — latched on rd_en / wr_en to avoid TB race
    reg [ADDR_W-1:0]  rd_addr_r;
    reg [ADDR_W-1:0]  wr_addr_r;
    reg [BUCKET_W-1:0] wr_bucket_r;
    reg [1:0]          wr_status_r;

    // Derive idx/off from REGISTERED addresses
    wire [IDX_W-1:0] rd_idx = rd_addr_r[IDX_W+11:12];
    wire [IDX_W-1:0] wr_idx = wr_addr_r[IDX_W+11:12];
    wire [EPB_W-1:0] rd_off = rd_idx[EPB_W-1:0];
    wire [EPB_W-1:0] wr_off = wr_idx[EPB_W-1:0];

    // HBM byte address of the AXI beat containing the entry
    wire [AXI_AW-1:0] rd_hbm_addr = PM_BASE
        + ({{(AXI_AW-IDX_W+EPB_W){1'b0}}, rd_idx[IDX_W-1:EPB_W]} << $clog2(BEAT_BYTES));
    wire [AXI_AW-1:0] wr_hbm_addr = PM_BASE
        + ({{(AXI_AW-IDX_W+EPB_W){1'b0}}, wr_idx[IDX_W-1:EPB_W]} << $clog2(BEAT_BYTES));

    // =========================================================================
    // Combinational pack: build wdata and wstrb in one shot (avoids XSim
    // partial-NBA bug)
    // =========================================================================
    reg [AXI_DW-1:0] wr_packed_data;
    reg [AXI_SW-1:0] wr_packed_strb;
    always @(*) begin
        wr_packed_data = {AXI_DW{1'b0}};
        wr_packed_data[wr_off*PACK_W +: PACK_W] = {{(PACK_W-WORD_W){1'b0}},
                                                     wr_status_r, wr_bucket_r};
        wr_packed_strb = {AXI_SW{1'b0}};
        wr_packed_strb[wr_off*ENTRY_BYTES +: ENTRY_BYTES] = {ENTRY_BYTES{1'b1}};
    end

    // =========================================================================
    // FSM
    // =========================================================================
    localparam S_IDLE    = 3'd0,
               S_RD_AR  = 3'd1,    // read: hold arvalid, deassert when arready
               S_RD_R   = 3'd2,    // read: wait R data
               S_WR_AW  = 3'd3,    // write: hold awvalid+wvalid, deassert when ready
               S_WR_B   = 3'd4;    // write: wait BRESP

    reg [2:0]      pm_state;
    reg [EPB_W-1:0] pend_off;

    assign busy = (pm_state != S_IDLE);
    assign dbg_state = pm_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pm_state       <= S_IDLE;
            rd_bucket      <= {BUCKET_W{1'b0}};
            rd_status      <= `ST_DUMMY;
            rd_valid       <= 1'b0;
            pend_off       <= {EPB_W{1'b0}};
            rd_addr_r      <= {ADDR_W{1'b0}};
            wr_addr_r      <= {ADDR_W{1'b0}};
            wr_bucket_r    <= {BUCKET_W{1'b0}};
            wr_status_r    <= `ST_DUMMY;
            m_axi_arvalid  <= 1'b0;
            m_axi_rready   <= 1'b0;
            m_axi_awvalid  <= 1'b0;
            m_axi_wvalid   <= 1'b0;
            m_axi_wlast    <= 1'b0;
            m_axi_bready   <= 1'b0;
            m_axi_wstrb    <= {AXI_SW{1'b0}};
        end else begin
            rd_valid <= 1'b0;

            case (pm_state)

            // =============================================================
            S_IDLE: begin
                if (rd_en) begin
                    // Latch address and compute AXI params directly from input
                    rd_addr_r     <= rd_slot_addr;
                    pend_off      <= rd_slot_addr[EPB_W+11:12];
                    m_axi_arid    <= {AXI_IDW{1'b0}};
                    m_axi_araddr  <= PM_BASE + ({{(AXI_AW-IDX_W+EPB_W){1'b0}},
                                     rd_slot_addr[IDX_W+11:EPB_W+12]} << $clog2(BEAT_BYTES));
                    m_axi_arlen   <= {AXI_LENW{1'b0}};
                    m_axi_arsize  <= AXI_SIZE;
                    m_axi_arburst <= AXI_BURST;
                    m_axi_arvalid <= 1'b1;
                    m_axi_rready  <= 1'b1;
                    pm_state      <= S_RD_AR;
                    $display("[PM] RD: addr=0x%08h off=%0d @%0t",
                             rd_slot_addr, rd_slot_addr[EPB_W+11:12], $time);
                end else if (wr_en_mux) begin
                    // Latch write inputs and compute AXI params directly
                    wr_addr_r     <= wr_addr_mux;
                    wr_bucket_r   <= wr_bucket_mux;
                    wr_status_r   <= wr_status_mux;
                    m_axi_awid    <= {AXI_IDW{1'b0}};
                    m_axi_awaddr  <= PM_BASE + ({{(AXI_AW-IDX_W+EPB_W){1'b0}},
                                     wr_addr_mux[IDX_W+11:EPB_W+12]} << $clog2(BEAT_BYTES));
                    m_axi_awlen   <= {AXI_LENW{1'b0}};
                    m_axi_awsize  <= AXI_SIZE;
                    m_axi_awburst <= AXI_BURST;
                    m_axi_awvalid <= 1'b1;
                    // Pack wdata/wstrb inline from raw inputs
                    begin : wr_pack_inline
                        reg [AXI_DW-1:0] d;
                        reg [AXI_SW-1:0] s;
                        integer wr_o;
                        wr_o = wr_addr_mux[EPB_W+11:12];
                        d = {AXI_DW{1'b0}};
                        d[wr_o*PACK_W +: PACK_W] = {{(PACK_W-WORD_W){1'b0}},
                                                      wr_status_mux, wr_bucket_mux};
                        s = {AXI_SW{1'b0}};
                        s[wr_o*ENTRY_BYTES +: ENTRY_BYTES] = {ENTRY_BYTES{1'b1}};
                        m_axi_wdata <= d;
                        m_axi_wstrb <= s;
                    end
                    m_axi_wlast   <= 1'b1;
                    m_axi_wvalid  <= 1'b1;
                    m_axi_bready  <= 1'b1;
                    pm_state      <= S_WR_AW;
                    $display("[PM] WR: addr=0x%08h off=%0d bkt=%0d st=%0d @%0t",
                             wr_addr_mux, wr_addr_mux[EPB_W+11:12],
                             wr_bucket_mux, wr_status_mux, $time);
                end
            end

            // =============================================================
            // READ: S_RD_AR is the hold cycle — arvalid has been high since
            // S_IDLE. Deassert now and advance. The SimObject's pre-eval on
            // THIS tick sees arvalid=1 (from S_IDLE's NBA). After this eval,
            // arvalid=0 — no double handshake.
            // =============================================================
            S_RD_AR: begin
                // Hold arvalid=1 (asserted by S_IDLE previous tick).
                // Only deassert when arready=1 (AXI handshake fires).
                // If arready=0 (HBM backpressure), stay and retry.
                if (m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    pm_state <= S_RD_R;
                end
            end

            S_RD_R: begin
                if (m_axi_rvalid) begin
                    $display("[PM] RD done: pend_off=%0d rdata=0x%064h", pend_off, m_axi_rdata);
                    $display("[PM] RD extract: bucket=%0d status=%0d",
                             m_axi_rdata[pend_off*PACK_W +: BUCKET_W],
                             m_axi_rdata[pend_off*PACK_W + BUCKET_W +: 2]);
                    m_axi_rready <= 1'b0;
                    rd_bucket    <= m_axi_rdata[pend_off*PACK_W +: BUCKET_W];
                    rd_status    <= m_axi_rdata[pend_off*PACK_W + BUCKET_W +: 2];
                    rd_valid     <= 1'b1;
                    pm_state     <= S_IDLE;
                end
            end

            // =============================================================
            // WRITE: S_WR_AW is the hold cycle — awvalid+wvalid have been
            // high since S_IDLE. Deassert now and advance. SimObject's
            // pre-eval on THIS tick sees awvalid=1, wvalid=1.
            // =============================================================
            S_WR_AW: begin
                // Hold awvalid+wvalid (asserted by S_IDLE previous tick).
                // Only deassert when ready=1 (AXI handshake fires).
                // If ready=0 (HBM backpressure), stay and retry.
                if (m_axi_awready) m_axi_awvalid <= 1'b0;
                if (m_axi_wready)  m_axi_wvalid  <= 1'b0;
                if (m_axi_wready)  m_axi_wlast   <= 1'b0;
                if ((!m_axi_awvalid || m_axi_awready) &&
                    (!m_axi_wvalid  || m_axi_wready))
                    pm_state <= S_WR_B;
            end

            S_WR_B: begin
                m_axi_awvalid <= 1'b0;
                m_axi_wvalid  <= 1'b0;
                if (m_axi_bvalid) begin
                    m_axi_bready <= 1'b0;
                    pm_state     <= S_IDLE;
                end
            end

            default: pm_state <= S_IDLE;
            endcase
        end
    end

    initial begin
        pm_state = S_IDLE;
        rd_valid = 1'b0;
    end

endmodule