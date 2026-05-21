`timescale 1ns / 1ps
`include "oram_params.vh"

module tb_onboard_test_top;

    reg clk, rst_n, start;

    wire        done;
    wire        all_pass;
    wire [5:0]  fail_count;
    wire [5:0]  test_number;
    wire [52:0] test_result;
    wire        running;
    wire [2:0]  phase;

    // =========================================================================
    // DUT - simulation uses internal MIG via `define SIMULATION
    // Add +define+SIMULATION to XSim compile options, or
    // `define SIMULATION at the top of this file.
    // =========================================================================
    onboard_test_top #(
        .NUM_CLIENTS(2),
        .TOKEN_WIDTH(32),
        .LEASE_ID_WIDTH(8)
    ) uut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .done(done), .all_pass(all_pass), .fail_count(fail_count),
        .test_number(test_number), .test_result(test_result),
        .running(running), .phase(phase),
        // AXI ports - unused in sim (internal MIG handles traffic)
        .m_axi_arid(), .m_axi_araddr(), .m_axi_arlen(),
        .m_axi_arsize(), .m_axi_arburst(), .m_axi_arvalid(),
        .m_axi_arready(1'b0),
        .m_axi_rid({`AXI_ID_W{1'b0}}),
        .m_axi_rdata({`AXI_DATA_W{1'b0}}),
        .m_axi_rresp(2'b0), .m_axi_rlast(1'b0), .m_axi_rvalid(1'b0),
        .m_axi_rready(),
        .m_axi_awid(), .m_axi_awaddr(), .m_axi_awlen(),
        .m_axi_awsize(), .m_axi_awburst(), .m_axi_awvalid(),
        .m_axi_awready(1'b0),
        .m_axi_wdata(), .m_axi_wstrb(), .m_axi_wlast(), .m_axi_wvalid(),
        .m_axi_wready(1'b0),
        .m_axi_bid({`AXI_ID_W{1'b0}}),
        .m_axi_bresp(2'b0), .m_axi_bvalid(1'b0),
        .m_axi_bready()
    );

    // Clock: 100 MHz
    initial clk = 0;
    always #5 clk = ~clk;

    // =========================================================================
    // WORKAROUND: XSim lease_token_table unpacked array bug
    // =========================================================================
    initial begin
        wait(rst_n);
        @(posedge clk);
        force uut.dut.val_valid = uut.client_req;
    end

    // =========================================================================
    // Test Sequence
    // =========================================================================
    initial begin
        rst_n = 0; start = 0;
        repeat(20) @(posedge clk);
        rst_n = 1;
        $display("[%0t] Reset released", $time);
        repeat(10) @(posedge clk);
        @(posedge clk); start = 1;
        @(posedge clk); start = 0;
        $display("[%0t] Start pulsed", $time);

        wait(done);
        repeat(5) @(posedge clk);

        // Report
        $display("");
        $display("============================================================");
        $display("  ON-BOARD TEST RESULTS");
        $display("============================================================");
        $display("  Phase reached : %0d", phase);
        $display("  Last test     : T%0d", test_number);
        $display("  Fail count    : %0d", fail_count);
        $display("  All pass      : %s", all_pass ? "YES" : "NO");
        $display("============================================================");
        begin : report_block
            integer t;
            for (t = 0; t < 53; t = t + 1) begin
                if (t == 0)  $display("  --- Phase 2: Bootstrap Write ---");
                if (t == 12) $display("  --- Phase 3: Read Back ---");
                if (t == 24) $display("  --- Phase 4: Re-read Shuffled ---");
                if (t == 36) $display("  --- Phase 5: Overwrite + Readback ---");
                if (t == 42) $display("  --- Phase 6: Client 1 ---");
                if (t == 45) $display("  --- Phase 7: Security ---");
                if (t == 48) $display("  --- Phase 8: Perf Tests ---");
                if (test_result[t])
                    $display("  T%0d: PASS", t + 1);
                else
                    $display("  T%0d: *** FAIL ***", t + 1);
            end
        end
        $display("============================================================");
        if (all_pass)
            $display("  *** ALL 53 TESTS PASSED ***");
        else
            $display("  *** %0d FAILURE(S) out of 53 tests ***", fail_count);
        $display("============================================================");
        $display("");
        $display("=== PERFORMANCE COUNTERS (avg cycles/op) ===");
        if (uut.perf_rb_ops > 0) begin
            $display("--- READ/BUCKET (%0d ops) ---", uut.perf_rb_ops);
            $display("  DDR Read:    %0d", uut.perf_rb_ddr_rd  / uut.perf_rb_ops);
            $display("  Scan/Lookup: %0d", uut.perf_rb_scan    / uut.perf_rb_ops);
            $display("  Decrypt:     %0d", uut.perf_rb_decrypt / uut.perf_rb_ops);
            $display("  Stash:       %0d", uut.perf_rb_stash   / uut.perf_rb_ops);
            $display("  Encrypt:     %0d", uut.perf_rb_encrypt / uut.perf_rb_ops);
            $display("  Compact:     %0d", uut.perf_rb_compact / uut.perf_rb_ops);
            $display("  Evict:       %0d", uut.perf_rb_evict   / uut.perf_rb_ops);
            $display("  DDR Write:   %0d", uut.perf_rb_ddr_wr  / uut.perf_rb_ops);
            $display("  TOTAL:       %0d", uut.perf_rb_total   / uut.perf_rb_ops);
        end
        if (uut.perf_rs_ops > 0) begin
            $display("--- READ/STASH (%0d ops) ---", uut.perf_rs_ops);
            $display("  DDR Read:    %0d", uut.perf_rs_ddr_rd  / uut.perf_rs_ops);
            $display("  Scan/Lookup: %0d", uut.perf_rs_scan    / uut.perf_rs_ops);
            $display("  Decrypt:     %0d", uut.perf_rs_decrypt / uut.perf_rs_ops);
            $display("  Stash:       %0d", uut.perf_rs_stash   / uut.perf_rs_ops);
            $display("  Encrypt:     %0d", uut.perf_rs_encrypt / uut.perf_rs_ops);
            $display("  Compact:     %0d", uut.perf_rs_compact / uut.perf_rs_ops);
            $display("  Evict:       %0d", uut.perf_rs_evict   / uut.perf_rs_ops);
            $display("  DDR Write:   %0d", uut.perf_rs_ddr_wr  / uut.perf_rs_ops);
            $display("  TOTAL:       %0d", uut.perf_rs_total   / uut.perf_rs_ops);
        end
        if (uut.perf_wb_ops > 0) begin
            $display("--- WRITE/BUCKET (%0d ops) ---", uut.perf_wb_ops);
            $display("  DDR Read:    %0d", uut.perf_wb_ddr_rd  / uut.perf_wb_ops);
            $display("  Scan/Lookup: %0d", uut.perf_wb_scan    / uut.perf_wb_ops);
            $display("  Decrypt:     %0d", uut.perf_wb_decrypt / uut.perf_wb_ops);
            $display("  Stash:       %0d", uut.perf_wb_stash   / uut.perf_wb_ops);
            $display("  Encrypt:     %0d", uut.perf_wb_encrypt / uut.perf_wb_ops);
            $display("  Compact:     %0d", uut.perf_wb_compact / uut.perf_wb_ops);
            $display("  Evict:       %0d", uut.perf_wb_evict   / uut.perf_wb_ops);
            $display("  DDR Write:   %0d", uut.perf_wb_ddr_wr  / uut.perf_wb_ops);
            $display("  TOTAL:       %0d", uut.perf_wb_total   / uut.perf_wb_ops);
        end
        if (uut.perf_ws_ops > 0) begin
            $display("--- WRITE/STASH (%0d ops) ---", uut.perf_ws_ops);
            $display("  DDR Read:    %0d", uut.perf_ws_ddr_rd  / uut.perf_ws_ops);
            $display("  Scan/Lookup: %0d", uut.perf_ws_scan    / uut.perf_ws_ops);
            $display("  Decrypt:     %0d", uut.perf_ws_decrypt / uut.perf_ws_ops);
            $display("  Stash:       %0d", uut.perf_ws_stash   / uut.perf_ws_ops);
            $display("  Encrypt:     %0d", uut.perf_ws_encrypt / uut.perf_ws_ops);
            $display("  Compact:     %0d", uut.perf_ws_compact / uut.perf_ws_ops);
            $display("  Evict:       %0d", uut.perf_ws_evict   / uut.perf_ws_ops);
            $display("  DDR Write:   %0d", uut.perf_ws_ddr_wr  / uut.perf_ws_ops);
            $display("  TOTAL:       %0d", uut.perf_ws_total   / uut.perf_ws_ops);
        end
        $display("================================================");
        $display("  rb=%0d rs=%0d wb=%0d ws=%0d  (total=%0d)",
                 uut.perf_rb_ops, uut.perf_rs_ops,
                 uut.perf_wb_ops, uut.perf_ws_ops,
                 uut.perf_rb_ops + uut.perf_rs_ops +
                 uut.perf_wb_ops + uut.perf_ws_ops);
        repeat(20) @(posedge clk); $finish;
    end

    // =========================================================================
    // Progress & Debug Monitor
    // =========================================================================
    reg [2:0]  prev_phase;
    reg [5:0]  prev_test;
    reg [5:0]  prev_fail_count;
    reg [4:0]  prev_oram_state;
    reg        prev_tag_mismatch;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_phase      <= 3'd0;
            prev_test       <= 6'd0;
            prev_fail_count <= 6'd0;
            prev_oram_state <= 5'd0;
            prev_tag_mismatch <= 1'b0;
        end else if (running) begin
            // Tag mismatch monitor
            prev_tag_mismatch <= uut.err_tag_mismatch;
            if (uut.err_tag_mismatch && !prev_tag_mismatch)
                $display("[%0t] *** TAG MISMATCH at T%0d oram_state=%0d ***",
                         $time, test_number, uut.dut_dbg_oram_state);
            // Phase transitions
            if (phase != prev_phase) begin
                case (phase)
                    3'd0: $display("[%0t] Phase 0: BRAM Init", $time);
                    3'd1: begin
                        $display("[%0t] Phase 1: Lease Grants", $time);
                        // Dump pos_map entries to verify init worked
                        $display("  pos_map[1] (slot 0x1000): bucket=%0d status=%0d",
                            uut.dut.u_oram.u_pos_map.mem[1][5:0],
                            uut.dut.u_oram.u_pos_map.mem[1][7:6]);
                        $display("  pos_map[2] (slot 0x2000): bucket=%0d status=%0d",
                            uut.dut.u_oram.u_pos_map.mem[2][5:0],
                            uut.dut.u_oram.u_pos_map.mem[2][7:6]);
                        $display("  pos_map[3] (slot 0x3000): bucket=%0d status=%0d",
                            uut.dut.u_oram.u_pos_map.mem[3][5:0],
                            uut.dut.u_oram.u_pos_map.mem[3][7:6]);
                    end
                    3'd2: $display("[%0t] Phase 2: Bootstrap Write (T1-T12)", $time);
                    3'd3: $display("[%0t] Phase 3: Read Back (T13-T24)", $time);
                    3'd4: $display("[%0t] Phase 4: Re-read Shuffled (T25-T36)", $time);
                    3'd5: $display("[%0t] Phase 5: Overwrite + Readback (T37-T42)", $time);
                    3'd6: $display("[%0t] Phase 6: Client 1 (T43-T45)", $time);
                    3'd7: $display("[%0t] Phase 7: Security (T46-T48)", $time);
                endcase
                prev_phase <= phase;
            end

            // Test start
            if (test_number != prev_test && test_number != 0) begin
                $display("[%0t] Starting T%0d (read=%b verify=%b client=%b slot=0x%05h)",
                    $time, test_number,
                    uut.cur_is_read, uut.cur_needs_verify,
                    uut.cur_client, uut.cur_slot);
                prev_test <= test_number;
            end

            // MIG: AR handshake - show cmd_bucket too
            if (uut.dut_axi_arvalid && uut.dut_arready)
                $display("[%0t] MIG AR: addr=0x%08h len=%0d cmd_bucket=%0d (T%0d)",
                    $time, uut.dut_axi_araddr, uut.dut_axi_arlen,
                    uut.dut.u_oram.axim_cmd_bucket, test_number);

            // MIG: AW handshake
            if (uut.dut_axi_awvalid && uut.dut_awready)
                $display("[%0t] MIG AW: addr=0x%08h len=%0d cmd_bucket=%0d (T%0d)",
                    $time, uut.dut_axi_awaddr, uut.dut_axi_awlen,
                    uut.dut.u_oram.axim_cmd_bucket, test_number);

            // MIG: B response
            if (uut.dut_bvalid && uut.dut_axi_bready)
                $display("[%0t] MIG B: resp (T%0d)", $time, test_number);

            // ORAM FSM state transitions
            if (uut.dut.u_oram.state != prev_oram_state) begin
                $display("[%0t] ORAM: %0d -> %0d (T%0d)",
                    $time, prev_oram_state, uut.dut.u_oram.state, test_number);
                // On pos_lookup -> DDR_READ transition, show what pos_map returned
                if (prev_oram_state == 5'd1 && uut.dut.u_oram.state == 5'd2) begin
                    $display("    pm_rd_bucket=%0d pm_rd_status=%0d req_b=%0d req_b_new=%0d",
                        uut.dut.u_oram.u_pos_map.rd_bucket,
                        uut.dut.u_oram.u_pos_map.rd_status,
                        uut.dut.u_oram.req_b,
                        uut.dut.u_oram.req_b_new);
                    $display("    axim_cmd_bucket=%0d bucket_base=0x%08h",
                        uut.dut.u_oram.axim_cmd_bucket,
                        uut.dut.u_oram.u_axi_master.bucket_base);
                end
                prev_oram_state <= uut.dut.u_oram.state;
            end

            // client_done
            if (|uut.client_done)
                $display("[%0t] client_done=%b (T%0d)", $time, uut.client_done, test_number);

            // Read beat capture: first 3 beats
            if (uut.cur_is_read && uut.tst_state == 5'd7) begin
                if (|uut.dut.client_rdata_valid) begin
                    if (uut.rd_beat_count < 3)
                        $display("[%0t] RDATA[%0d][31:0]=0x%08h (T%0d)",
                            $time, uut.rd_beat_count,
                            uut.dut.client_rdata[uut.cur_client * 256 +: 32],
                            test_number);
                end
            end

            // Verify: first 3 beats and any mismatch
            if (uut.tst_state == 5'd9 && uut.verify_beat > 0) begin
                if (uut.verify_beat <= 4) begin
                    $display("[%0t] VERIFY[%0d]: got=0x%08h exp=0x%08h (T%0d)",
                        $time, uut.verify_beat - 7'd1,
                        uut.verify_rdata[31:0],
                        (uut.verify_beat - 7'd1),
                        test_number);
                end
            end

            // Fail events
            if (fail_count != prev_fail_count) begin
                $display("[%0t] *** FAIL T%0d ***", $time, test_number);
                $display("    tst_state=%0d timeout=%0d mismatch=%b beats=%0d",
                    uut.tst_state, uut.timeout_cnt,
                    uut.rd_mismatch, uut.rd_beat_count);
                $display("    oram_state=%0d busy=%b done=%b violation=%b",
                    uut.dut.u_oram.state, uut.oram_busy,
                    uut.client_done, uut.access_violation);
                $display("    mig_fsm=%0d arvalid=%b awvalid=%b",
                    0, uut.dut_axi_arvalid, uut.dut_axi_awvalid);
                $display("    captured[0]=0x%08h [1]=0x%08h [2]=0x%08h",
                    uut.captured_rdata[0][31:0],
                    uut.captured_rdata[1][31:0],
                    uut.captured_rdata[2][31:0]);
                prev_fail_count <= fail_count;
            end
        end
    end

    // =========================================================================
    // Watchdog
    // =========================================================================
    initial begin
        #5_000_000_000;
        $display("[WATCHDOG] 5s timeout at phase=%0d T%0d tst=%0d oram=%0d",
            phase, test_number, uut.tst_state,
            uut.dut.u_oram.state);
        $finish;
    end

    initial begin
        $dumpfile("tb_onboard_test.vcd");
        $dumpvars(0, tb_onboard_test_top);
    end

endmodule


/*
`timescale 1ns / 1ps
`include "oram_params.vh"

module tb_onboard_test_top;

    reg clk, rst_n, start;

    wire        done;
    wire        all_pass;
    wire [5:0]  fail_count;
    wire [5:0]  test_number;
    wire [47:0] test_result;
    wire        running;
    wire [2:0]  phase;

    // =========================================================================
    // DUT - simulation uses internal MIG via `define SIMULATION
    // Add +define+SIMULATION to XSim compile options, or
    // `define SIMULATION at the top of this file.
    // =========================================================================
    onboard_test_top #(
        .NUM_CLIENTS(2),
        .TOKEN_WIDTH(32),
        .LEASE_ID_WIDTH(8)
    ) uut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .done(done), .all_pass(all_pass), .fail_count(fail_count),
        .test_number(test_number), .test_result(test_result),
        .running(running), .phase(phase),
        // AXI ports - unused in sim (internal MIG handles traffic)
        .m_axi_arid(), .m_axi_araddr(), .m_axi_arlen(),
        .m_axi_arsize(), .m_axi_arburst(), .m_axi_arvalid(),
        .m_axi_arready(1'b0),
        .m_axi_rid({`AXI_ID_W{1'b0}}),
        .m_axi_rdata({`AXI_DATA_W{1'b0}}),
        .m_axi_rresp(2'b0), .m_axi_rlast(1'b0), .m_axi_rvalid(1'b0),
        .m_axi_rready(),
        .m_axi_awid(), .m_axi_awaddr(), .m_axi_awlen(),
        .m_axi_awsize(), .m_axi_awburst(), .m_axi_awvalid(),
        .m_axi_awready(1'b0),
        .m_axi_wdata(), .m_axi_wstrb(), .m_axi_wlast(), .m_axi_wvalid(),
        .m_axi_wready(1'b0),
        .m_axi_bid({`AXI_ID_W{1'b0}}),
        .m_axi_bresp(2'b0), .m_axi_bvalid(1'b0),
        .m_axi_bready()
    );

    // Clock: 100 MHz
    initial clk = 0;
    always #5 clk = ~clk;

    // =========================================================================
    // WORKAROUND: XSim lease_token_table unpacked array bug
    // =========================================================================
    initial begin
        wait(rst_n);
        @(posedge clk);
        force uut.dut.val_valid = uut.client_req;
    end

    // =========================================================================
    // Test Sequence
    // =========================================================================
    initial begin
        rst_n = 0; start = 0;
        repeat(20) @(posedge clk);
        rst_n = 1;
        $display("[%0t] Reset released", $time);
        repeat(10) @(posedge clk);
        @(posedge clk); start = 1;
        @(posedge clk); start = 0;
        $display("[%0t] Start pulsed", $time);

        wait(done);
        repeat(5) @(posedge clk);

        // Report
        $display("");
        $display("============================================================");
        $display("  ON-BOARD TEST RESULTS");
        $display("============================================================");
        $display("  Phase reached : %0d", phase);
        $display("  Last test     : T%0d", test_number);
        $display("  Fail count    : %0d", fail_count);
        $display("  All pass      : %s", all_pass ? "YES" : "NO");
        $display("============================================================");
        begin : report_block
            integer t;
            for (t = 0; t < 48; t = t + 1) begin
                if (t == 0)  $display("  --- Phase 2: Bootstrap Write ---");
                if (t == 12) $display("  --- Phase 3: Read Back ---");
                if (t == 24) $display("  --- Phase 4: Re-read Shuffled ---");
                if (t == 36) $display("  --- Phase 5: Overwrite + Readback ---");
                if (t == 42) $display("  --- Phase 6: Client 1 ---");
                if (t == 45) $display("  --- Phase 7: Security ---");
                if (test_result[t])
                    $display("  T%0d: PASS", t + 1);
                else
                    $display("  T%0d: *** FAIL ***", t + 1);
            end
        end
        $display("============================================================");
        if (all_pass)
            $display("  *** ALL 53 TESTS PASSED ***");
        else
            $display("  *** %0d FAILURE(S) out of 53 tests ***", fail_count);
        $display("============================================================");
        $display("");
        $display("=== PERFORMANCE COUNTERS (avg cycles/op) ===");
        if (uut.perf_rd_ops > 0) begin
            $display("--- READ (%0d ops) ---", uut.perf_rd_ops);
            $display("  DDR Read:    %0d", uut.perf_rd_ddr_rd  / uut.perf_rd_ops);
            $display("  Scan/Lookup: %0d", uut.perf_rd_scan    / uut.perf_rd_ops);
            $display("  Decrypt:     %0d", uut.perf_rd_decrypt / uut.perf_rd_ops);
            $display("  Stash:       %0d", uut.perf_rd_stash   / uut.perf_rd_ops);
            $display("  Encrypt:     %0d", uut.perf_rd_encrypt / uut.perf_rd_ops);
            $display("  Compact:     %0d", uut.perf_rd_compact / uut.perf_rd_ops);
            $display("  Evict:       %0d", uut.perf_rd_evict   / uut.perf_rd_ops);
            $display("  DDR Write:   %0d", uut.perf_rd_ddr_wr  / uut.perf_rd_ops);
            $display("  TOTAL:       %0d", uut.perf_rd_total   / uut.perf_rd_ops);
        end
        if (uut.perf_wr_ops > 0) begin
            $display("--- WRITE (%0d ops) ---", uut.perf_wr_ops);
            $display("  DDR Read:    %0d", uut.perf_wr_ddr_rd  / uut.perf_wr_ops);
            $display("  Scan/Lookup: %0d", uut.perf_wr_scan    / uut.perf_wr_ops);
            $display("  Decrypt:     %0d", uut.perf_wr_decrypt / uut.perf_wr_ops);
            $display("  Stash:       %0d", uut.perf_wr_stash   / uut.perf_wr_ops);
            $display("  Encrypt:     %0d", uut.perf_wr_encrypt / uut.perf_wr_ops);
            $display("  Compact:     %0d", uut.perf_wr_compact / uut.perf_wr_ops);
            $display("  Evict:       %0d", uut.perf_wr_evict   / uut.perf_wr_ops);
            $display("  DDR Write:   %0d", uut.perf_wr_ddr_wr  / uut.perf_wr_ops);
            $display("  TOTAL:       %0d", uut.perf_wr_total   / uut.perf_wr_ops);
        end
        $display("================================================");
        repeat(20) @(posedge clk); $finish;
    end

    // =========================================================================
    // Progress & Debug Monitor
    // =========================================================================
    reg [2:0]  prev_phase;
    reg [5:0]  prev_test;
    reg [5:0]  prev_fail_count;
    reg [4:0]  prev_oram_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_phase      <= 3'd0;
            prev_test       <= 6'd0;
            prev_fail_count <= 6'd0;
            prev_oram_state <= 5'd0;
        end else if (running) begin
            // Phase transitions
            if (phase != prev_phase) begin
                case (phase)
                    3'd0: $display("[%0t] Phase 0: BRAM Init", $time);
                    3'd1: begin
                        $display("[%0t] Phase 1: Lease Grants", $time);
                        // Dump pos_map entries to verify init worked
                        $display("  pos_map[1] (slot 0x1000): bucket=%0d status=%0d",
                            uut.dut.u_oram.u_pos_map.mem[1][5:0],
                            uut.dut.u_oram.u_pos_map.mem[1][7:6]);
                        $display("  pos_map[2] (slot 0x2000): bucket=%0d status=%0d",
                            uut.dut.u_oram.u_pos_map.mem[2][5:0],
                            uut.dut.u_oram.u_pos_map.mem[2][7:6]);
                        $display("  pos_map[3] (slot 0x3000): bucket=%0d status=%0d",
                            uut.dut.u_oram.u_pos_map.mem[3][5:0],
                            uut.dut.u_oram.u_pos_map.mem[3][7:6]);
                    end
                    3'd2: $display("[%0t] Phase 2: Bootstrap Write (T1-T12)", $time);
                    3'd3: $display("[%0t] Phase 3: Read Back (T13-T24)", $time);
                    3'd4: $display("[%0t] Phase 4: Re-read Shuffled (T25-T36)", $time);
                    3'd5: $display("[%0t] Phase 5: Overwrite + Readback (T37-T42)", $time);
                    3'd6: $display("[%0t] Phase 6: Client 1 (T43-T45)", $time);
                    3'd7: $display("[%0t] Phase 7: Security (T46-T48)", $time);
                endcase
                prev_phase <= phase;
            end

            // Test start
            if (test_number != prev_test && test_number != 0) begin
                $display("[%0t] Starting T%0d (read=%b verify=%b client=%b slot=0x%05h)",
                    $time, test_number,
                    uut.cur_is_read, uut.cur_needs_verify,
                    uut.cur_client, uut.cur_slot);
                prev_test <= test_number;
            end

            // MIG: AR handshake - show cmd_bucket too
            if (uut.dut_axi_arvalid && uut.dut_arready)
                $display("[%0t] MIG AR: addr=0x%08h len=%0d cmd_bucket=%0d (T%0d)",
                    $time, uut.dut_axi_araddr, uut.dut_axi_arlen,
                    uut.dut.u_oram.axim_cmd_bucket, test_number);

            // MIG: AW handshake
            if (uut.dut_axi_awvalid && uut.dut_awready)
                $display("[%0t] MIG AW: addr=0x%08h len=%0d cmd_bucket=%0d (T%0d)",
                    $time, uut.dut_axi_awaddr, uut.dut_axi_awlen,
                    uut.dut.u_oram.axim_cmd_bucket, test_number);

            // MIG: B response
            if (uut.dut_bvalid && uut.dut_axi_bready)
                $display("[%0t] MIG B: resp (T%0d)", $time, test_number);

            // ORAM FSM state transitions
            if (uut.dut.u_oram.state != prev_oram_state) begin
                $display("[%0t] ORAM: %0d -> %0d (T%0d)",
                    $time, prev_oram_state, uut.dut.u_oram.state, test_number);
                // On pos_lookup -> DDR_READ transition, show what pos_map returned
                if (prev_oram_state == 5'd1 && uut.dut.u_oram.state == 5'd2) begin
                    $display("    pm_rd_bucket=%0d pm_rd_status=%0d req_b=%0d req_b_new=%0d",
                        uut.dut.u_oram.u_pos_map.rd_bucket,
                        uut.dut.u_oram.u_pos_map.rd_status,
                        uut.dut.u_oram.req_b,
                        uut.dut.u_oram.req_b_new);
                    $display("    axim_cmd_bucket=%0d bucket_base=0x%08h",
                        uut.dut.u_oram.axim_cmd_bucket,
                        uut.dut.u_oram.u_axi_master.bucket_base);
                end
                prev_oram_state <= uut.dut.u_oram.state;
            end

            // client_done
            if (|uut.client_done)
                $display("[%0t] client_done=%b (T%0d)", $time, uut.client_done, test_number);

            // Read beat capture: first 3 beats
            if (uut.cur_is_read && uut.tst_state == 5'd7) begin
                if (|uut.dut.client_rdata_valid) begin
                    if (uut.rd_beat_count < 3)
                        $display("[%0t] RDATA[%0d][31:0]=0x%08h (T%0d)",
                            $time, uut.rd_beat_count,
                            uut.dut.client_rdata[uut.cur_client * 256 +: 32],
                            test_number);
                end
            end

            // Verify: first 3 beats and any mismatch
            if (uut.tst_state == 5'd9 && uut.verify_beat > 0) begin
                if (uut.verify_beat <= 4) begin
                    $display("[%0t] VERIFY[%0d]: got=0x%08h exp=0x%08h (T%0d)",
                        $time, uut.verify_beat - 7'd1,
                        uut.verify_rdata[31:0],
                        (uut.verify_beat - 7'd1),
                        test_number);
                end
            end

            // Fail events
            if (fail_count != prev_fail_count) begin
                $display("[%0t] *** FAIL T%0d ***", $time, test_number);
                $display("    tst_state=%0d timeout=%0d mismatch=%b beats=%0d",
                    uut.tst_state, uut.timeout_cnt,
                    uut.rd_mismatch, uut.rd_beat_count);
                $display("    oram_state=%0d busy=%b done=%b violation=%b",
                    uut.dut.u_oram.state, uut.oram_busy,
                    uut.client_done, uut.access_violation);
                $display("    mig_fsm=%0d arvalid=%b awvalid=%b",
                    0, uut.dut_axi_arvalid, uut.dut_axi_awvalid);
                $display("    captured[0]=0x%08h [1]=0x%08h [2]=0x%08h",
                    uut.captured_rdata[0][31:0],
                    uut.captured_rdata[1][31:0],
                    uut.captured_rdata[2][31:0]);
                prev_fail_count <= fail_count;
            end
        end
    end

    // =========================================================================
    // Watchdog
    // =========================================================================
    initial begin
        #5_000_000_000;
        $display("[WATCHDOG] 5s timeout at phase=%0d T%0d tst=%0d oram=%0d",
            phase, test_number, uut.tst_state,
            uut.dut.u_oram.state);
        $finish;
    end

    initial begin
        $dumpfile("tb_onboard_test.vcd");
        $dumpvars(0, tb_onboard_test_top);
    end
 endmodule
 */   
    
    
    
  