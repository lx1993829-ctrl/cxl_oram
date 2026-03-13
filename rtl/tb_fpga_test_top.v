`timescale 1ns / 1ps
//============================================================================
// Testbench for fpga_test_top - Realistic DDR Backpressure
//
// DDR model uses task-based responders with LFSR-driven random timing:
//   - AR/AW ready: 1-2 cycle random latency before acceptance
//   - Write data: ~12.5% chance of 1-4 cycle wready stall after each beat
//   - Read data: ~12.5% chance of 1-4 cycle rvalid gap after each beat
//   - B response: 1-2 cycle random latency after last write beat
//   - Guaranteed forward progress: after any stall, next cycle always valid
//   - Write priority when both arvalid and awvalid asserted
//============================================================================

module tb_fpga_test_top;

    localparam ADDR_WIDTH  = 32;
    localparam DATA_WIDTH  = 128;
    localparam CLK_PERIOD  = 10;
    localparam MEM_DEPTH   = 4096;  // 256-bit words

    //=========================================================================
    // Clock and Stimulus
    //=========================================================================
    reg clk;
    reg rst_n;
    reg btn_start;

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    //=========================================================================
    // DUT Status
    //=========================================================================
    wire        led_running;
    wire        led_pass;
    wire        led_fail;
    wire [3:0]  led_test_id;
    wire [3:0]  led_fail_id;
    wire [7:0]  led_error_count;

    //=========================================================================
    // AXI Signals
    //=========================================================================
    wire [ADDR_WIDTH-1:0] m_axi_awaddr;
    wire [7:0]            m_axi_awlen;
    wire [2:0]            m_axi_awsize;
    wire [1:0]            m_axi_awburst;
    wire                  m_axi_awvalid;
    reg                   m_axi_awready;
    wire [255:0]          m_axi_wdata;
    wire [31:0]           m_axi_wstrb;
    wire                  m_axi_wlast;
    wire                  m_axi_wvalid;
    reg                   m_axi_wready;
    reg  [1:0]            m_axi_bresp;
    reg                   m_axi_bvalid;
    wire                  m_axi_bready;
    wire [ADDR_WIDTH-1:0] m_axi_araddr;
    wire [7:0]            m_axi_arlen;
    wire [2:0]            m_axi_arsize;
    wire [1:0]            m_axi_arburst;
    wire                  m_axi_arvalid;
    reg                   m_axi_arready;
    reg  [255:0]          m_axi_rdata;
    reg  [1:0]            m_axi_rresp;
    reg                   m_axi_rlast;
    reg                   m_axi_rvalid;
    wire                  m_axi_rready;

    //=========================================================================
    // DUT
    //=========================================================================
    fpga_test_top #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .btn_start      (btn_start),
        .led_running    (led_running),
        .led_pass       (led_pass),
        .led_fail       (led_fail),
        .led_test_id    (led_test_id),
        .led_fail_id    (led_fail_id),
        .led_error_count(led_error_count),
        .m_axi_awaddr   (m_axi_awaddr),
        .m_axi_awlen    (m_axi_awlen),
        .m_axi_awsize   (m_axi_awsize),
        .m_axi_awburst  (m_axi_awburst),
        .m_axi_awvalid  (m_axi_awvalid),
        .m_axi_awready  (m_axi_awready),
        .m_axi_wdata    (m_axi_wdata),
        .m_axi_wstrb    (m_axi_wstrb),
        .m_axi_wlast    (m_axi_wlast),
        .m_axi_wvalid   (m_axi_wvalid),
        .m_axi_wready   (m_axi_wready),
        .m_axi_bresp    (m_axi_bresp),
        .m_axi_bvalid   (m_axi_bvalid),
        .m_axi_bready   (m_axi_bready),
        .m_axi_araddr   (m_axi_araddr),
        .m_axi_arlen    (m_axi_arlen),
        .m_axi_arsize   (m_axi_arsize),
        .m_axi_arburst  (m_axi_arburst),
        .m_axi_arvalid  (m_axi_arvalid),
        .m_axi_arready  (m_axi_arready),
        .m_axi_rdata    (m_axi_rdata),
        .m_axi_rresp    (m_axi_rresp),
        .m_axi_rlast    (m_axi_rlast),
        .m_axi_rvalid   (m_axi_rvalid),
        .m_axi_rready   (m_axi_rready)
    );

    //=========================================================================
    // DDR Memory
    //=========================================================================
    reg [255:0] ddr_mem [0:MEM_DEPTH-1];
    integer mi;

    function [31:0] addr_to_idx;
        input [ADDR_WIDTH-1:0] addr;
        begin
            addr_to_idx = addr[ADDR_WIDTH-1:5];  // >> 5 for 32-byte words
        end
    endfunction

    //=========================================================================
    // LFSR for random timing (16-bit, same polynomial as reference)
    //=========================================================================
    reg [15:0] mig_lfsr;

    task mig_rand;
        begin
            mig_lfsr = {mig_lfsr[14:0],
                        mig_lfsr[15] ^ mig_lfsr[13] ^ mig_lfsr[12] ^ mig_lfsr[10]};
        end
    endtask

    //=========================================================================
    // Initialize DDR signals
    //=========================================================================
    initial begin
        m_axi_arready = 1'b0;
        m_axi_rvalid  = 1'b0;
        m_axi_rlast   = 1'b0;
        m_axi_rdata   = 256'b0;
        m_axi_rresp   = 2'b00;
        m_axi_awready = 1'b0;
        m_axi_wready  = 1'b0;
        m_axi_bvalid  = 1'b0;
        m_axi_bresp   = 2'b00;
        mig_lfsr      = 16'hBEEF;
        for (mi = 0; mi < MEM_DEPTH; mi = mi + 1)
            ddr_mem[mi] = 256'b0;
    end

    //=========================================================================
    // READ RESPONDER - task-based, guaranteed forward progress
    //
    // After any stall gap, the NEXT cycle always presents valid data.
    // ~12.5% chance of 1-4 cycle rvalid gap after each accepted beat.
    //=========================================================================
    task axi_read_respond;
        integer beat, mem_base, total_beats, gap_cnt;
        reg     must_valid;
        begin
            // Wait for arvalid
            while (!m_axi_arvalid) @(posedge clk);
            $display("[DDR-RD] t=%0t AR addr=0x%08h len=%0d",
                     $time, m_axi_araddr, m_axi_arlen + 1);

            // Random arready latency: 1-2 cycles
            mig_rand;
            repeat (1 + mig_lfsr[0]) @(posedge clk);
            m_axi_arready = 1'b1;
            @(posedge clk);
            m_axi_arready = 1'b0;

            // Capture burst parameters
            mem_base    = addr_to_idx(m_axi_araddr);
            total_beats = m_axi_arlen + 1;
            gap_cnt     = 0;
            must_valid  = 1'b1;

            // Stream read data with random gaps
            for (beat = 0; beat < total_beats; ) begin
                if (gap_cnt > 0) begin
                    m_axi_rvalid = 1'b0;
                    gap_cnt = gap_cnt - 1;
                    if (gap_cnt == 0) must_valid = 1'b1;
                end else begin
                    m_axi_rvalid = 1'b1;
                    m_axi_rdata  = ddr_mem[(mem_base + beat) % MEM_DEPTH];
                    m_axi_rlast  = (beat == total_beats - 1);
                    m_axi_rresp  = 2'b00;
                end

                @(posedge clk);

                if (m_axi_rvalid) begin
                    beat = beat + 1;
                    must_valid = 1'b0;
                    // ~12.5% chance of stall, 1-4 cycles
                    mig_rand;
                    if (!must_valid && mig_lfsr[2:0] == 3'd0) begin
                        gap_cnt = 1 + mig_lfsr[4:3];
                    end
                end
            end

            m_axi_rvalid = 1'b0;
            m_axi_rlast  = 1'b0;
            @(posedge clk);
        end
    endtask

    //=========================================================================
    // WRITE RESPONDER - task-based, guaranteed forward progress
    //
    // ~12.5% chance of 1-4 cycle wready stall after each accepted beat.
    // Random AW ready and B response latency.
    //=========================================================================
    task axi_write_respond;
        integer beat, mem_base, gap_cnt, done_flag;
        begin
            // Wait for awvalid
            while (!m_axi_awvalid) @(posedge clk);
            $display("[DDR-WR] t=%0t AW addr=0x%08h len=%0d",
                     $time, m_axi_awaddr, m_axi_awlen + 1);

            // Random awready latency: 1-2 cycles
            mig_rand;
            repeat (1 + mig_lfsr[0]) @(posedge clk);
            m_axi_awready = 1'b1;
            @(posedge clk);
            m_axi_awready = 1'b0;

            // Capture burst parameters
            mem_base  = addr_to_idx(m_axi_awaddr);
            beat      = 0;
            gap_cnt   = 0;
            done_flag = 0;

            // Accept write data beats with random wready stalls
            while (!done_flag) begin
                if (gap_cnt > 0) begin
                    m_axi_wready = 1'b0;
                    gap_cnt = gap_cnt - 1;
                end else begin
                    m_axi_wready = 1'b1;
                end

                @(posedge clk);

                if (m_axi_wvalid && m_axi_wready) begin
                    ddr_mem[(mem_base + beat) % MEM_DEPTH] = m_axi_wdata;
                    beat = beat + 1;
                    if (m_axi_wlast) done_flag = 1;

                    // ~12.5% chance of stall after accept, 1-4 cycles
                    if (!done_flag) begin
                        mig_rand;
                        if (mig_lfsr[2:0] == 3'd0) begin
                            gap_cnt = 1 + mig_lfsr[4:3];
                        end
                    end
                end
            end

            @(posedge clk);
            m_axi_wready = 1'b0;

            // Random bvalid latency: 1-2 cycles
            mig_rand;
            repeat (1 + mig_lfsr[0]) @(posedge clk);
            m_axi_bvalid = 1'b1;
            m_axi_bresp  = 2'b00;
            while (!m_axi_bready) @(posedge clk);
            @(posedge clk);
            m_axi_bvalid = 1'b0;
        end
    endtask

    //=========================================================================
    // DDR Responder Loop - write priority when both valid
    //=========================================================================
    initial begin
        wait (rst_n);
        @(posedge clk);
        forever begin
            while (!m_axi_arvalid && !m_axi_awvalid) @(posedge clk);
            if (m_axi_awvalid)
                axi_write_respond;
            else
                axi_read_respond;
            @(posedge clk);
        end
    end

    //=========================================================================
    // Test Stimulus
    //=========================================================================
    initial begin
        $display("============================================================");
        $display("  tb_fpga_test_top - Realistic DDR Backpressure");
        $display("============================================================");

        rst_n     = 1'b0;
        btn_start = 1'b0;

        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        $display("[%0t] [TB] Reset released", $time);

        repeat (50) @(posedge clk);

        $display("[%0t] [TB] Pressing btn_start", $time);
        @(posedge clk);
        btn_start = 1'b1;
        @(posedge clk);
        btn_start = 1'b0;

        $display("[%0t] [TB] Waiting for tests to complete...", $time);
        wait (led_running == 1'b0 && (led_pass == 1'b1 || led_fail == 1'b1));
        repeat (10) @(posedge clk);

        $display("");
        $display("============================================================");
        $display("  RESULTS (realistic DDR backpressure)");
        $display("============================================================");
        $display("  led_running     = %b", led_running);
        $display("  led_pass        = %b", led_pass);
        $display("  led_fail        = %b", led_fail);
        $display("  led_test_id     = %0d", led_test_id);
        $display("  led_fail_id     = %0d", led_fail_id);
        $display("  led_error_count = %0d", led_error_count);
        $display("============================================================");

        if (led_pass && !led_fail)
            $display("  *** ALL TESTS PASSED (realistic backpressure) ***");
        else
            $display("  *** FAILED at test %0d, %0d error(s) ***",
                     led_fail_id, led_error_count);
        $display("============================================================");

        repeat (20) @(posedge clk);
        $finish;
    end

    //=========================================================================
    // Live Monitor
    //=========================================================================
    reg [3:0] prev_test_id;
    always @(posedge clk) begin
        if (rst_n) begin
            if (led_test_id != prev_test_id && led_running)
                $display("[%0t] [TB] === Starting Test %0d ===", $time, led_test_id);
            prev_test_id <= led_test_id;
        end else
            prev_test_id <= 4'd0;
    end

    //=========================================================================
    // Timeout
    //=========================================================================
    initial begin
        #50_000_000;
        $display("\n[TB] *** TIMEOUT ***");
        $display("  led_test_id = %0d  led_fail_id = %0d", led_test_id, led_fail_id);
        $finish;
    end

    //=========================================================================
    // Waveform
    //=========================================================================
    initial begin
        $dumpfile("tb_fpga_test_top.vcd");
        $dumpvars(0, tb_fpga_test_top);
    end

endmodule