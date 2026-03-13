`timescale 1ns/1ps
`include "oram_params.vh"

module tb_oram_selftest;

    localparam AXI_DW  = `AXI_DATA_W;
    localparam AXI_AW  = `AXI_ADDR_W;
    localparam AXI_SW  = `AXI_STRB_W;
    localparam AXI_IDW = `AXI_ID_W;
    localparam AXI_LW  = `AXI_LEN_W;

    reg clk, rst_n;
    initial clk = 0;
    always #5 clk = ~clk;
    initial begin rst_n = 0; repeat(10) @(posedge clk); rst_n = 1; end

    reg        btn_start;
    wire       busy, done, pass, fail;
    wire [7:0] oram_fail_code;
    wire [3:0] oram_test_num;
    wire [3:0] dbg_state;
    wire       dbg_client_done, dbg_client_req;
    wire [`BUCKET_ID_W-1:0] dbg_req_b, dbg_req_b_new;
    wire       dbg_found_in_bucket, dbg_found_in_stash;
    wire       dbg_same_bucket, dbg_err_stash_ovf;
    wire [`STASH_PTR_W:0] dbg_stash_occ;
    wire       dbg_t6_start;
    wire       dbg_arvalid, dbg_arready;
    wire [AXI_AW-1:0] dbg_araddr;
    wire [AXI_LW-1:0] dbg_arlen;
    wire       dbg_rvalid, dbg_rready, dbg_rlast;
    wire       dbg_awvalid, dbg_awready;
    wire [AXI_AW-1:0] dbg_awaddr;
    wire [AXI_LW-1:0] dbg_awlen;
    wire       dbg_wvalid, dbg_wready, dbg_wlast;
    wire       dbg_bvalid, dbg_bready;

    wire [AXI_IDW-1:0] m_axi_arid, m_axi_awid;
    wire [AXI_AW-1:0]  m_axi_araddr, m_axi_awaddr;
    wire [AXI_LW-1:0]  m_axi_arlen, m_axi_awlen;
    wire [2:0]          m_axi_arsize, m_axi_awsize;
    wire [1:0]          m_axi_arburst, m_axi_awburst;
    wire                m_axi_arvalid, m_axi_awvalid;
    reg                 m_axi_arready, m_axi_awready;
    reg  [AXI_IDW-1:0] m_axi_rid;
    reg  [AXI_DW-1:0]  m_axi_rdata;
    reg  [1:0]          m_axi_rresp;
    reg                 m_axi_rlast, m_axi_rvalid;
    wire                m_axi_rready;
    wire [AXI_DW-1:0]  m_axi_wdata;
    wire [AXI_SW-1:0]  m_axi_wstrb;
    wire                m_axi_wlast, m_axi_wvalid;
    reg                 m_axi_wready;
    reg  [AXI_IDW-1:0] m_axi_bid;
    reg  [1:0]          m_axi_bresp;
    reg                 m_axi_bvalid;
    wire                m_axi_bready;

    oram_selftest_top dut (
        .clk(clk), .rst_n(rst_n), .btn_start(btn_start),
        .busy(busy), .done(done), .pass(pass), .fail(fail),
        .oram_fail_code(oram_fail_code), .oram_test_num(oram_test_num),
        .dbg_state(dbg_state), .dbg_client_done(dbg_client_done),
        .dbg_client_req(dbg_client_req), .dbg_req_b(dbg_req_b),
        .dbg_req_b_new(dbg_req_b_new), .dbg_found_in_bucket(dbg_found_in_bucket),
        .dbg_found_in_stash(dbg_found_in_stash), .dbg_same_bucket(dbg_same_bucket),
        .dbg_err_stash_ovf(dbg_err_stash_ovf), .dbg_stash_occ(dbg_stash_occ),
        .dbg_t6_start(dbg_t6_start),
        .dbg_arvalid(dbg_arvalid), .dbg_arready(dbg_arready),
        .dbg_araddr(dbg_araddr), .dbg_arlen(dbg_arlen),
        .dbg_rvalid(dbg_rvalid), .dbg_rready(dbg_rready), .dbg_rlast(dbg_rlast),
        .dbg_awvalid(dbg_awvalid), .dbg_awready(dbg_awready),
        .dbg_awaddr(dbg_awaddr), .dbg_awlen(dbg_awlen),
        .dbg_wvalid(dbg_wvalid), .dbg_wready(dbg_wready), .dbg_wlast(dbg_wlast),
        .dbg_bvalid(dbg_bvalid), .dbg_bready(dbg_bready),
        .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen), .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst), .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rid(m_axi_rid), .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready),
        .m_axi_awid(m_axi_awid), .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awlen(m_axi_awlen), .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst), .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast), .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready)
    );

    // DDR model
    localparam DDR_MEM_BEATS = `ORAM_B * `BEATS_PER_BKT;
    reg [AXI_DW-1:0] ddr_mem [0:DDR_MEM_BEATS-1];
    integer mi;
    initial for (mi = 0; mi < DDR_MEM_BEATS; mi = mi + 1) ddr_mem[mi] = {8{mi[31:0]}};

    initial begin
        m_axi_arready = 0; m_axi_rvalid = 0; m_axi_rlast = 0;
        m_axi_rdata = 0; m_axi_rresp = 0; m_axi_rid = 0;
        m_axi_awready = 0; m_axi_wready = 0;
        m_axi_bvalid = 0; m_axi_bresp = 0; m_axi_bid = 0;
    end

    // LFSR for random timing
    reg [15:0] mig_lfsr;
    initial mig_lfsr = 16'hBEEF;
    task mig_rand;
        begin
            mig_lfsr = {mig_lfsr[14:0], mig_lfsr[15]^mig_lfsr[13]^mig_lfsr[12]^mig_lfsr[10]};
        end
    endtask

    // Is this the init phase?
    wire is_init = (dut.seq_state == 2'd1);

    // =====================================================================
    // READ RESPONDER - guaranteed forward progress
    // After any stall gap, the NEXT cycle always presents valid data.
    // =====================================================================
    task axi_read_respond;
        integer beat, mem_base, total_beats, gap_cnt;
        reg     must_valid;
        begin
            $display("[TB_RD] t=%0t waiting for arvalid", $time);
            while (!m_axi_arvalid) @(posedge clk);
            $display("[DDR] t=%0t AR araddr=0x%08X arlen=%0d", $time, m_axi_araddr, m_axi_arlen);
            @(posedge clk); // 1 cycle arready delay
            m_axi_arready = 1'b1;
            $display("[TB_RD] t=%0t arready=1", $time);
            @(posedge clk);
            m_axi_arready = 1'b0;
            $display("[TB_RD] t=%0t arready=0, starting data. rready=%b", $time, m_axi_rready);
            mem_base    = (m_axi_araddr - 32'h8000_0000) >> 5;
            total_beats = m_axi_arlen + 1;
            gap_cnt     = 0;
            must_valid  = 1'b1;
            for (beat = 0; beat < total_beats; ) begin
                if (gap_cnt > 0) begin
                    m_axi_rvalid = 1'b0;
                    gap_cnt = gap_cnt - 1;
                    if (gap_cnt == 0) must_valid = 1'b1;
                end else begin
                    m_axi_rvalid = 1'b1;
                    m_axi_rdata  = ddr_mem[(mem_base + beat) % DDR_MEM_BEATS];
                    m_axi_rlast  = (beat == total_beats - 1);
                    m_axi_rresp  = 2'b00;
                end
                @(posedge clk);
                // The AXI master has rready=1 throughout ST_RDATA.
                // We count beats based on our own rvalid - no need to check rready
                // (checking rready creates a race with the master's NBA on rlast).
                if (m_axi_rvalid) begin
                    beat = beat + 1;
                    must_valid = 1'b0;
                    if (!is_init) begin
                        mig_rand;
                        if (!must_valid && mig_lfsr[2:0] == 3'd0)
                            gap_cnt = 1 + mig_lfsr[4:3];
                    end
                    if (beat == 1 || beat == 64 || beat == 128)
                        $display("[TB_RD] t=%0t beat=%0d/%0d", $time, beat, total_beats);
                end
            end
            $display("[TB_RD] t=%0t burst complete %0d beats", $time, beat);
            m_axi_rvalid = 1'b0;
            m_axi_rlast  = 1'b0;
            @(posedge clk);
        end
    endtask

    // =====================================================================
    // WRITE RESPONDER - guaranteed forward progress
    // =====================================================================
    task axi_write_respond;
        integer beat, mem_base, gap_cnt;
        reg     discard;
        begin
            while (!m_axi_awvalid) @(posedge clk);
            discard = is_init;
            if (!discard)
                $display("[DDR] t=%0t AW awaddr=0x%08X awlen=%0d", $time, m_axi_awaddr, m_axi_awlen);
            // awready delay
            if (!discard) begin mig_rand; repeat(1 + mig_lfsr[0]) @(posedge clk); end
            else @(posedge clk);
            m_axi_awready = 1'b1;
            @(posedge clk);
            m_axi_awready = 1'b0;
            mem_base = (m_axi_awaddr - 32'h8000_0000) >> 5;
            beat     = 0;
            gap_cnt  = 0;
            begin : wloop
                integer done_flag;
                done_flag = 0;
                while (!done_flag) begin
                    if (gap_cnt > 0) begin
                        m_axi_wready = 1'b0;
                        gap_cnt = gap_cnt - 1;
                    end else begin
                        m_axi_wready = 1'b1;
                    end
                    @(posedge clk);
                    if (m_axi_wvalid && m_axi_wready) begin
                        if (!discard)
                            ddr_mem[(mem_base + beat) % DDR_MEM_BEATS] = m_axi_wdata;
                        beat = beat + 1;
                        if (m_axi_wlast) done_flag = 1;
                        // Random stall after accept (~12% chance, test phase only)
                        if (!discard && !done_flag) begin
                            mig_rand;
                            if (mig_lfsr[2:0] == 3'd0)
                                gap_cnt = 1 + mig_lfsr[4:3];
                        end
                    end
                end
            end
            @(posedge clk);
            m_axi_wready = 1'b0;
            // bvalid delay
            if (!discard) begin mig_rand; repeat(1 + mig_lfsr[0]) @(posedge clk); end
            else @(posedge clk);
            m_axi_bvalid = 1'b1;
            m_axi_bresp  = 2'b00;
            while (!m_axi_bready) @(posedge clk);
            @(posedge clk);
            m_axi_bvalid = 1'b0;
        end
    endtask

    // DDR responder loop
    initial begin
        wait(rst_n); @(posedge clk);
        forever begin
            while (!m_axi_arvalid && !m_axi_awvalid) @(posedge clk);
            if (m_axi_awvalid) begin
                $display("[DDR_LOOP] t=%0t ? write_respond (awvalid=%b arvalid=%b)", $time, m_axi_awvalid, m_axi_arvalid);
                axi_write_respond;
            end else begin
                $display("[DDR_LOOP] t=%0t ? read_respond (awvalid=%b arvalid=%b)", $time, m_axi_awvalid, m_axi_arvalid);
                axi_read_respond;
            end
            @(posedge clk);
        end
    end

    // DDR init spot-check
    always @(posedge clk) begin
        if (dut.ddr_init_done && dut.seq_state == 2'd1) begin
            $display("[DDR_CHK] ddr_mem[0]=%08X ddr_mem[1]=%08X ddr_mem[127]=%08X ddr_mem[128]=%08X",
                     ddr_mem[0][31:0], ddr_mem[1][31:0], ddr_mem[127][31:0], ddr_mem[128][31:0]);
        end
    end

    // Test sequence
    integer fail_count;
    initial begin
        btn_start  = 0;
        fail_count = 0;
        wait(rst_n); repeat(5) @(posedge clk);
        @(posedge clk); btn_start <= 1'b1;
        @(posedge clk); btn_start <= 1'b0;
        $display("[TB] Start at t=%0t", $time);

        begin : wait_done
            integer timeout; timeout = 0;
            while (!done && timeout < 200_000_000) begin @(posedge clk); timeout = timeout + 1; end
            if (timeout >= 200_000_000) begin
                $display("[TIMEOUT] t=%0t state=%0d test=%0d fail=%08b",
                         $time, dut.seq_state, oram_test_num, oram_fail_code);
                fail_count = fail_count + 1;
            end else
                $display("[TB] Done at t=%0t (%0d cycles)", $time, timeout);
        end

        $display("\n--- Results ---");
        $display("  pass=%b fail=%b fail_code=%08b test_num=%0d", pass, fail, oram_fail_code, oram_test_num);
        if (pass && !fail)
            $display("[PASS] All tests passed");
        else begin
            $display("[FAIL] fail_code breakdown:");
            if (oram_fail_code[1]) $display("  T1 FAIL");
            if (oram_fail_code[2]) $display("  T2 FAIL");
            if (oram_fail_code[3]) $display("  T3 FAIL");
            if (oram_fail_code[4]) $display("  T4 FAIL");
            if (oram_fail_code[5]) $display("  T5 FAIL");
            if (oram_fail_code[6]) $display("  T6 FAIL");
            if (oram_fail_code[7]) $display("  Stash overflow");
            fail_count = fail_count + 1;
        end

        repeat(10) @(posedge clk);
        $display("\n========================================");
        if (fail_count == 0) $display("  SELFTEST TB: ALL PASSED");
        else $display("  SELFTEST TB: %0d FAILURE(S)", fail_count);
        $display("========================================");
        $finish;
    end

    // Monitors
    always @(posedge clk)
        if (dut.seq_state == 2'd2 && dut.oram_test_done)
            $display("[SEQ] t=%0t ORAM tests done", $time);

    reg [3:0] prev_tn; initial prev_tn = 4'hF;
    always @(posedge clk)
        if (oram_test_num !== prev_tn && dut.u_test.busy) begin
            prev_tn <= oram_test_num;
            $display("[TEST] t=%0t T%0d (idx=%0d)", $time, oram_test_num, dut.u_test.access_idx);
        end

    // Watchdog
    initial begin #20_000_000_000; $display("[WATCHDOG] timeout"); $finish; end
    initial begin $dumpfile("tb_oram_selftest.vcd"); $dumpvars(0, tb_oram_selftest); end

endmodule