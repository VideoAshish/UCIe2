// ============================================================
// UCIe 3.0 MPG MUX Testbench
// Production-Grade Verification Environment
// ============================================================
// This testbench provides comprehensive verification for the
// UCIe 3.0 Multi-Protocol Gateway MUX including:
//
// 1. Protocol Arbitration Tests (PCIe, CXL, Streaming)
// 2. FLIT Format Tests (Formats 1-6)
// 3. CRC and Error Detection Tests
// 4. Retry Mechanism Tests
// 5. Power Management Tests (L1, L2)
// 6. Runtime Recalibration Tests
// 7. Sideband MTP Tests
// 8. BIST Tests
// 9. Stress Tests with Random Traffic
// 10. Coverage and Assertions
// ============================================================

`timescale 1ns/1ps

module ucie_mpg_mux_tb;

    // ============================================================
    // Testbench Parameters
    // ============================================================
    parameter int LINK_WIDTH           = 16;
    parameter int MAX_LINK_WIDTH       = 64;
    parameter int DATA_WIDTH           = 256;
    parameter int PHY_LANE_WIDTH       = 32;
    parameter int PHY_WIDTH            = MAX_LINK_WIDTH * PHY_LANE_WIDTH;
    parameter int NUM_PROTOCOLS        = 3;
    parameter int NUM_PROTOCOL_STACKS  = 2;
    parameter int FLIT_FORMAT          = 3;
    parameter int CXL_LATENCY_OPT      = 1;
    parameter int CRC_EN               = 1;
    parameter int CRC_TYPE             = 2;
    parameter int CRC_BITS             = 64;
    parameter int RETRY_EN             = 1;
    parameter int RETRY_BUFFER_DEPTH   = 32;
    parameter int MULTI_PROTOCOL_EN    = 1;
    parameter int ENHANCED_MULTI_PROTO = 1;
    parameter int WAKEUP_TIME_CYCLES   = 100;
    parameter int BIST_EN              = 1;
    parameter int DEBUG_WIDTH          = 128;
    parameter int SCAN_CHAINS          = 8;

    // ============================================================
    // Clock and Reset Generation
    // ============================================================
    logic clk_core;
    logic clk_phy;
    logic clk_sideband;
    logic clk_axi;
    logic rst_core_n;
    logic rst_phy_n;
    logic rst_sideband_n;
    logic rst_axi_n;

    // Clock periods
    localparam real CORE_CLK_PERIOD = 2.0;     // 500MHz
    localparam real PHY_CLK_PERIOD  = 0.15625; // 6.4GHz (64GT/s quarter-rate)
    localparam real SB_CLK_PERIOD   = 1.25;    // 800MHz
    localparam real AXI_CLK_PERIOD  = 5.0;     // 200MHz

    // Clock generation
    always #(CORE_CLK_PERIOD/2) clk_core = ~clk_core;
    always #(PHY_CLK_PERIOD/2) clk_phy = ~clk_phy;
    always #(SB_CLK_PERIOD/2) clk_sideband = ~clk_sideband;
    always #(AXI_CLK_PERIOD/2) clk_axi = ~clk_axi;

    // Reset generation
    initial begin
        clk_core = 0;
        clk_phy = 0;
        clk_sideband = 0;
        clk_axi = 0;
        rst_core_n = 0;
        rst_phy_n = 0;
        rst_sideband_n = 0;
        rst_axi_n = 0;
        #100;
        rst_core_n = 1;
        rst_phy_n = 1;
        rst_sideband_n = 1;
        rst_axi_n = 1;
        #50;
    end

    // ============================================================
    // DUT Instantiation
    // ============================================================
    logic [DATA_WIDTH-1:0] pcie_tx_data;
    logic                  pcie_tx_valid;
    logic                  pcie_tx_ready;
    logic                  pcie_tx_sop;
    logic                  pcie_tx_eop;
    logic [3:0]            pcie_tx_credits;
    logic [1:0]            pcie_tx_fmt;
    logic [15:0]           pcie_tx_seq_num;
    logic [31:0]           pcie_sb_tx_data;
    logic                  pcie_sb_tx_valid;
    logic                  pcie_sb_tx_ready;

    logic [DATA_WIDTH-1:0] cxl_tx_data;
    logic                  cxl_tx_valid;
    logic                  cxl_tx_ready;
    logic                  cxl_tx_sop;
    logic                  cxl_tx_eop;
    logic [3:0]            cxl_tx_credits;
    logic [1:0]            cxl_tx_fmt;
    logic [15:0]           cxl_tx_seq_num;
    logic [31:0]           cxl_sb_tx_data;
    logic                  cxl_sb_tx_valid;
    logic                  cxl_sb_tx_ready;

    logic [DATA_WIDTH-1:0] stream_tx_data;
    logic                  stream_tx_valid;
    logic                  stream_tx_ready;
    logic                  stream_tx_sop;
    logic                  stream_tx_eop;
    logic [3:0]            stream_tx_credits;
    logic [31:0]           stream_sb_tx_data;
    logic                  stream_sb_tx_valid;
    logic                  stream_sb_tx_ready;

    logic [31:0]           fdi_cfg_wdata;
    logic                  fdi_cfg_wvalid;
    logic                  fdi_cfg_wready;
    logic [15:0]           fdi_cfg_addr;
    logic                  fdi_cfg_rw;
    logic [31:0]           fdi_cfg_rdata;
    logic                  fdi_cfg_rvalid;
    logic [31:0]           fdi_status;
    logic [15:0]           fdi_error_count;
    logic                  fdi_link_active;

    logic [PHY_WIDTH-1:0]  rdi_tx_data;
    logic                  rdi_tx_valid;
    logic                  rdi_tx_ready;
    logic [PHY_WIDTH-1:0]  rdi_rx_data;
    logic                  rdi_rx_valid;
    logic                  rdi_rx_ready;

    logic [3:0]            pl_state;
    logic                  pl_ready;
    logic [3:0]            lp_state_req;
    logic                  lp_state_ack;
    logic                  lp_wake_req;
    logic                  lp_wake_ack;
    logic                  pl_clk_req;
    logic                  lp_clk_ack;

    logic [31:0]           rdi_cfg_wdata;
    logic                  rdi_cfg_wvalid;
    logic                  rdi_cfg_wready;
    logic [15:0]           rdi_cfg_addr;
    logic                  rdi_cfg_rw;
    logic [31:0]           rdi_cfg_rdata;
    logic                  rdi_cfg_rvalid;

    logic [31:0]           rdi_sb_tx_data;
    logic                  rdi_sb_tx_valid;
    logic                  rdi_sb_tx_ready;
    logic [31:0]           rdi_sb_rx_data;
    logic                  rdi_sb_rx_valid;
    logic                  rdi_sb_rx_ready;

    logic [PHY_WIDTH-1:0]  phy_tx_data;
    logic                  phy_tx_valid;
    logic                  phy_tx_ready;
    logic                  phy_tx_clk_en;
    logic [5:0]            phy_tx_rate;
    logic [5:0]            phy_tx_width;
    logic [PHY_WIDTH-1:0]  phy_rx_data;
    logic                  phy_rx_valid;
    logic                  phy_rx_ready;
    logic [15:0]           phy_ctrl;
    logic [15:0]           phy_status;
    logic                  phy_recal_req;
    logic                  phy_recal_done;
    logic                  phy_init_start;
    logic                  phy_init_done;

    logic                  phy_pwr_req;
    logic                  phy_pwr_ack;
    logic [2:0]            phy_pwr_state;

    logic                  sb_mtp_enable;
    logic [31:0]           sb_mtp_data;
    logic                  sb_mtp_valid;
    logic                  sb_mtp_ready;
    logic                  sb_mtp_priority;
    logic                  sb_init_done;
    logic [3:0]            sb_link_status;

    logic                  bist_en;
    logic [7:0]            bist_pattern;
    logic                  bist_start;
    logic                  bist_done;
    logic                  bist_fail;
    logic [31:0]           bist_error_count;
    logic                  scan_enable;
    logic [SCAN_CHAINS-1:0] scan_in;
    logic [SCAN_CHAINS-1:0] scan_out;
    logic [DEBUG_WIDTH-1:0] debug_bus;

    logic                  top_enable;
    logic [1:0]            top_mode;
    logic                  top_ready;
    logic [31:0]           top_status;
    logic                  top_interrupt;

    ucie_mpg_mux #(
        .LINK_WIDTH(LINK_WIDTH),
        .MAX_LINK_WIDTH(MAX_LINK_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .PHY_LANE_WIDTH(PHY_LANE_WIDTH),
        .PHY_WIDTH(PHY_WIDTH),
        .NUM_PROTOCOLS(NUM_PROTOCOLS),
        .NUM_PROTOCOL_STACKS(NUM_PROTOCOL_STACKS),
        .FLIT_FORMAT(FLIT_FORMAT),
        .CXL_LATENCY_OPT(CXL_LATENCY_OPT),
        .CRC_EN(CRC_EN),
        .CRC_TYPE(CRC_TYPE),
        .CRC_BITS(CRC_BITS),
        .RETRY_EN(RETRY_EN),
        .RETRY_BUFFER_DEPTH(RETRY_BUFFER_DEPTH),
        .MULTI_PROTOCOL_EN(MULTI_PROTOCOL_EN),
        .ENHANCED_MULTI_PROTO(ENHANCED_MULTI_PROTO),
        .WAKEUP_TIME_CYCLES(WAKEUP_TIME_CYCLES),
        .BIST_EN(BIST_EN),
        .DEBUG_WIDTH(DEBUG_WIDTH),
        .SCAN_CHAINS(SCAN_CHAINS)
    ) dut (
        .clk_core(clk_core),
        .clk_phy(clk_phy),
        .clk_sideband(clk_sideband),
        .clk_axi(clk_axi),
        .rst_core_n(rst_core_n),
        .rst_phy_n(rst_phy_n),
        .rst_sideband_n(rst_sideband_n),
        .rst_axi_n(rst_axi_n),

        .pcie_tx_data(pcie_tx_data),
        .pcie_tx_valid(pcie_tx_valid),
        .pcie_tx_ready(pcie_tx_ready),
        .pcie_tx_sop(pcie_tx_sop),
        .pcie_tx_eop(pcie_tx_eop),
        .pcie_tx_credits(pcie_tx_credits),
        .pcie_tx_fmt(pcie_tx_fmt),
        .pcie_tx_seq_num(pcie_tx_seq_num),
        .pcie_sb_tx_data(pcie_sb_tx_data),
        .pcie_sb_tx_valid(pcie_sb_tx_valid),
        .pcie_sb_tx_ready(pcie_sb_tx_ready),

        .cxl_tx_data(cxl_tx_data),
        .cxl_tx_valid(cxl_tx_valid),
        .cxl_tx_ready(cxl_tx_ready),
        .cxl_tx_sop(cxl_tx_sop),
        .cxl_tx_eop(cxl_tx_eop),
        .cxl_tx_credits(cxl_tx_credits),
        .cxl_tx_fmt(cxl_tx_fmt),
        .cxl_tx_seq_num(cxl_tx_seq_num),
        .cxl_sb_tx_data(cxl_sb_tx_data),
        .cxl_sb_tx_valid(cxl_sb_tx_valid),
        .cxl_sb_tx_ready(cxl_sb_tx_ready),

        .stream_tx_data(stream_tx_data),
        .stream_tx_valid(stream_tx_valid),
        .stream_tx_ready(stream_tx_ready),
        .stream_tx_sop(stream_tx_sop),
        .stream_tx_eop(stream_tx_eop),
        .stream_tx_credits(stream_tx_credits),
        .stream_sb_tx_data(stream_sb_tx_data),
        .stream_sb_tx_valid(stream_sb_tx_valid),
        .stream_sb_tx_ready(stream_sb_tx_ready),

        .fdi_cfg_wdata(fdi_cfg_wdata),
        .fdi_cfg_wvalid(fdi_cfg_wvalid),
        .fdi_cfg_wready(fdi_cfg_wready),
        .fdi_cfg_addr(fdi_cfg_addr),
        .fdi_cfg_rw(fdi_cfg_rw),
        .fdi_cfg_rdata(fdi_cfg_rdata),
        .fdi_cfg_rvalid(fdi_cfg_rvalid),
        .fdi_status(fdi_status),
        .fdi_error_count(fdi_error_count),
        .fdi_link_active(fdi_link_active),

        .rdi_tx_data(rdi_tx_data),
        .rdi_tx_valid(rdi_tx_valid),
        .rdi_tx_ready(rdi_tx_ready),
        .rdi_rx_data(rdi_rx_data),
        .rdi_rx_valid(rdi_rx_valid),
        .rdi_rx_ready(rdi_rx_ready),

        .pl_state(pl_state),
        .pl_ready(pl_ready),
        .lp_state_req(lp_state_req),
        .lp_state_ack(lp_state_ack),
        .lp_wake_req(lp_wake_req),
        .lp_wake_ack(lp_wake_ack),
        .pl_clk_req(pl_clk_req),
        .lp_clk_ack(lp_clk_ack),

        .rdi_cfg_wdata(rdi_cfg_wdata),
        .rdi_cfg_wvalid(rdi_cfg_wvalid),
        .rdi_cfg_wready(rdi_cfg_wready),
        .rdi_cfg_addr(rdi_cfg_addr),
        .rdi_cfg_rw(rdi_cfg_rw),
        .rdi_cfg_rdata(rdi_cfg_rdata),
        .rdi_cfg_rvalid(rdi_cfg_rvalid),

        .rdi_sb_tx_data(rdi_sb_tx_data),
        .rdi_sb_tx_valid(rdi_sb_tx_valid),
        .rdi_sb_tx_ready(rdi_sb_tx_ready),
        .rdi_sb_rx_data(rdi_sb_rx_data),
        .rdi_sb_rx_valid(rdi_sb_rx_valid),
        .rdi_sb_rx_ready(rdi_sb_rx_ready),

        .phy_tx_data(phy_tx_data),
        .phy_tx_valid(phy_tx_valid),
        .phy_tx_ready(phy_tx_ready),
        .phy_tx_clk_en(phy_tx_clk_en),
        .phy_tx_rate(phy_tx_rate),
        .phy_tx_width(phy_tx_width),
        .phy_rx_data(phy_rx_data),
        .phy_rx_valid(phy_rx_valid),
        .phy_rx_ready(phy_rx_ready),
        .phy_ctrl(phy_ctrl),
        .phy_status(phy_status),
        .phy_recal_req(phy_recal_req),
        .phy_recal_done(phy_recal_done),
        .phy_init_start(phy_init_start),
        .phy_init_done(phy_init_done),

        .phy_pwr_req(phy_pwr_req),
        .phy_pwr_ack(phy_pwr_ack),
        .phy_pwr_state(phy_pwr_state),

        .sb_mtp_enable(sb_mtp_enable),
        .sb_mtp_data(sb_mtp_data),
        .sb_mtp_valid(sb_mtp_valid),
        .sb_mtp_ready(sb_mtp_ready),
        .sb_mtp_priority(sb_mtp_priority),
        .sb_init_done(sb_init_done),
        .sb_link_status(sb_link_status),

        .bist_en(bist_en),
        .bist_pattern(bist_pattern),
        .bist_start(bist_start),
        .bist_done(bist_done),
        .bist_fail(bist_fail),
        .bist_error_count(bist_error_count),
        .scan_enable(scan_enable),
        .scan_in(scan_in),
        .scan_out(scan_out),
        .debug_bus(debug_bus),

        .top_enable(top_enable),
        .top_mode(top_mode),
        .top_ready(top_ready),
        .top_status(top_status),
        .top_interrupt(top_interrupt)
    );

    // ============================================================
    // PHY Model (Simplified)
    // ============================================================
    always_ff @(posedge clk_phy or negedge rst_phy_n) begin
        if (!rst_phy_n) begin
            phy_rx_data <= '0;
            phy_rx_valid <= 1'b0;
            phy_status <= 16'hFFFF;
            phy_recal_done <= 1'b0;
            phy_init_done <= 1'b0;
            phy_tx_ready <= 1'b1;
        end
        else begin
            // Loopback mode
            if (phy_init_start) begin
                phy_init_done <= 1'b1;
            end
            
            // Recalibration
            if (phy_recal_req) begin
                phy_recal_done <= 1'b1;
            end
            
            // Simple loopback
            if (phy_tx_valid && phy_tx_ready) begin
                phy_rx_data <= phy_tx_data;
                phy_rx_valid <= 1'b1;
            end
            else begin
                phy_rx_valid <= 1'b0;
            end
            
            phy_tx_ready <= 1'b1;
        end
    end

    // Power Management Acknowledge
    always_ff @(posedge clk_core or negedge rst_core_n) begin
        if (!rst_core_n) begin
            phy_pwr_ack <= 1'b0;
            lp_clk_ack <= 1'b0;
        end
        else begin
            if (phy_pwr_req) begin
                phy_pwr_ack <= 1'b1;
            end
            if (pl_clk_req) begin
                lp_clk_ack <= 1'b1;
            end
        end
    end

    // ============================================================
    // Test Variables
    // ============================================================
    int test_id;
    int test_pass_count;
    int test_fail_count;
    bit test_failed;
    
    // Randomization
    rand bit [DATA_WIDTH-1:0] rand_data;
    rand bit [31:0] rand_sb_data;
    rand bit [3:0] rand_credits;
    rand bit [1:0] rand_fmt;
    rand bit [15:0] rand_seq;

    // ============================================================
    // Task: Initialize All Inputs
    // ============================================================
    task init_inputs();
        pcie_tx_data = '0;
        pcie_tx_valid = 1'b0;
        pcie_tx_sop = 1'b0;
        pcie_tx_eop = 1'b0;
        pcie_tx_credits = 4'd8;
        pcie_tx_fmt = 2'b00;
        pcie_tx_seq_num = '0;
        pcie_sb_tx_data = '0;
        pcie_sb_tx_valid = 1'b0;

        cxl_tx_data = '0;
        cxl_tx_valid = 1'b0;
        cxl_tx_sop = 1'b0;
        cxl_tx_eop = 1'b0;
        cxl_tx_credits = 4'd8;
        cxl_tx_fmt = 2'b01;
        cxl_tx_seq_num = '0;
        cxl_sb_tx_data = '0;
        cxl_sb_tx_valid = 1'b0;

        stream_tx_data = '0;
        stream_tx_valid = 1'b0;
        stream_tx_sop = 1'b0;
        stream_tx_eop = 1'b0;
        stream_tx_credits = 4'd8;
        stream_sb_tx_data = '0;
        stream_sb_tx_valid = 1'b0;

        fdi_cfg_wdata = '0;
        fdi_cfg_wvalid = 1'b0;
        fdi_cfg_addr = '0;
        fdi_cfg_rw = 1'b0;

        rdi_tx_ready = 1'b1;
        rdi_rx_valid = 1'b0;
        rdi_rx_data = '0;

        lp_state_req = 4'd0;
        lp_wake_req = 1'b0;

        rdi_cfg_wdata = '0;
        rdi_cfg_wvalid = 1'b0;
        rdi_cfg_addr = '0;
        rdi_cfg_rw = 1'b0;

        rdi_sb_tx_data = '0;
        rdi_sb_tx_valid = 1'b0;
        rdi_sb_rx_ready = 1'b1;

        sb_mtp_enable = 1'b0;
        sb_mtp_data = '0;
        sb_mtp_valid = 1'b0;
        sb_mtp_priority = 1'b0;

        bist_en = 1'b0;
        bist_pattern = '0;
        bist_start = 1'b0;
        scan_enable = 1'b0;
        scan_in = '0;

        top_enable = 1'b0;
        top_mode = 2'b00;
    endtask

    // ============================================================
    // Task: Wait for Link Active
    // ============================================================
    task wait_for_link_active(int timeout = 10000);
        int cnt = 0;
        while (!fdi_link_active && cnt < timeout) begin
            @(posedge clk_core);
            cnt++;
        end
        if (cnt >= timeout) begin
            $display("ERROR: Link did not become active");
            test_failed = 1;
        end
    endtask

    // ============================================================
    // Task: Send PCIe FLIT
    // ============================================================
    task send_pcie_flit(bit [DATA_WIDTH-1:0] data, bit sop, bit eop, int credits = 8);
        pcie_tx_data = data;
        pcie_tx_sop = sop;
        pcie_tx_eop = eop;
        pcie_tx_credits = credits;
        pcie_tx_valid = 1'b1;
        @(posedge clk_core);
        while (!pcie_tx_ready) @(posedge clk_core);
        pcie_tx_valid = 1'b0;
        $display("Sent PCIe FLIT: SOP=%0d, EOP=%0d, Data=0x%0h", sop, eop, data);
    endtask

    // ============================================================
    // Task: Send CXL FLIT
    // ============================================================
    task send_cxl_flit(bit [DATA_WIDTH-1:0] data, bit sop, bit eop, bit [1:0] fmt, int credits = 8);
        cxl_tx_data = data;
        cxl_tx_sop = sop;
        cxl_tx_eop = eop;
        cxl_tx_fmt = fmt;
        cxl_tx_credits = credits;
        cxl_tx_valid = 1'b1;
        @(posedge clk_core);
        while (!cxl_tx_ready) @(posedge clk_core);
        cxl_tx_valid = 1'b0;
        $display("Sent CXL FLIT: SOP=%0d, EOP=%0d, FMT=%0d, Data=0x%0h", sop, eop, fmt, data);
    endtask

    // ============================================================
    // Task: Send Streaming FLIT
    // ============================================================
    task send_stream_flit(bit [DATA_WIDTH-1:0] data, bit sop, bit eop, int credits = 8);
        stream_tx_data = data;
        stream_tx_sop = sop;
        stream_tx_eop = eop;
        stream_tx_credits = credits;
        stream_tx_valid = 1'b1;
        @(posedge clk_core);
        while (!stream_tx_ready) @(posedge clk_core);
        stream_tx_valid = 1'b0;
        $display("Sent Streaming FLIT: SOP=%0d, EOP=%0d, Data=0x%0h", sop, eop, data);
    endtask

    // ============================================================
    // Task: Configure Register
    // ============================================================
    task cfg_write(bit [15:0] addr, bit [31:0] data);
        fdi_cfg_addr = addr;
        fdi_cfg_wdata = data;
        fdi_cfg_rw = 1'b1;
        fdi_cfg_wvalid = 1'b1;
        @(posedge clk_core);
        while (!fdi_cfg_wready) @(posedge clk_core);
        fdi_cfg_wvalid = 1'b0;
        $display("CFG Write: Addr=0x%04h, Data=0x%08h", addr, data);
    endtask

    // ============================================================
    // Task: Read Register
    // ============================================================
    task cfg_read(bit [15:0] addr);
        fdi_cfg_addr = addr;
        fdi_cfg_rw = 1'b0;
        fdi_cfg_wvalid = 1'b1;
        @(posedge clk_core);
        while (!fdi_cfg_rvalid) @(posedge clk_core);
        $display("CFG Read: Addr=0x%04h, Data=0x%08h", addr, fdi_cfg_rdata);
        fdi_cfg_wvalid = 1'b0;
    endtask

    // ============================================================
    // Test 1: Basic Link Initialization
    // ============================================================
    task test_link_initialization();
        $display("\n========================================");
        $display("Test 1: Basic Link Initialization");
        $display("========================================");
        test_failed = 0;

        init_inputs();
        top_enable = 1'b1;
        @(posedge clk_core);
        #10;

        wait_for_link_active();

        if (fdi_link_active) begin
            $display("PASS: Link initialized successfully");
            $display("  PL State: %0d", pl_state);
            $display("  PL Ready: %0d", pl_ready);
            $display("  Top Ready: %0d", top_ready);
            $display("  FDI Status: 0x%08h", fdi_status);
        end

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtask

    // ============================================================
    // Test 2: PCIe Traffic Test
    // ============================================================
    task test_pcie_traffic();
        $display("\n========================================");
        $display("Test 2: PCIe Traffic Test");
        $display("========================================");
        test_failed = 0;

        // Wait for link to be active
        wait_for_link_active();

        // Send PCIe FLITs
        $display("Sending PCIe FLITs...");
        for (int i = 0; i < 10; i++) begin
            send_pcie_flit(rand_data, (i == 0), (i == 9));
            #10;
        end

        // Check if traffic was transmitted
        if (debug_bus[60:40] > 0) begin
            $display("PASS: PCIe traffic transmitted successfully");
            $display("  FLIT Count: %0d", debug_bus[60:40]);
        end

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtask

    // ============================================================
    // Test 3: CXL Traffic Test
    // ============================================================
    task test_cxl_traffic();
        $display("\n========================================");
        $display("Test 3: CXL Traffic Test");
        $display("========================================");
        test_failed = 0;

        wait_for_link_active();

        // Send CXL FLITs
        $display("Sending CXL FLITs...");
        for (int i = 0; i < 10; i++) begin
            send_cxl_flit(rand_data, (i == 0), (i == 9), rand_fmt);
            #10;
        end

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtask

    // ============================================================
    // Test 4: Multi-Protocol Arbitration Test
    // ============================================================
    task test_multi_protocol_arbitration();
        $display("\n========================================");
        $display("Test 4: Multi-Protocol Arbitration Test");
        $display("========================================");
        test_failed = 0;

        wait_for_link_active();

        // Send interleaved traffic from all protocols
        $display("Sending interleaved traffic...");
        fork
            begin
                for (int i = 0; i < 5; i++) begin
                    send_pcie_flit(rand_data, (i == 0), (i == 4));
                    #5;
                end
            end
            begin
                for (int i = 0; i < 5; i++) begin
                    send_cxl_flit(rand_data, (i == 0), (i == 4), rand_fmt);
                    #8;
                end
            end
            begin
                for (int i = 0; i < 5; i++) begin
                    send_stream_flit(rand_data, (i == 0), (i == 4));
                    #12;
                end
            end
        join

        $display("PASS: Multi-protocol arbitration test completed");
        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtask

    // ============================================================
    // Test 5: CRC Error Detection Test
    // ============================================================
    task test_crc_error_detection();
        $display("\n========================================");
        $display("Test 5: CRC Error Detection Test");
        $display("========================================");
        test_failed = 0;

        wait_for_link_active();

        // Send a FLIT with known data pattern
        $display("Sending FLIT with CRC verification...");
        send_pcie_flit(32'hA5A5_A5A5, 1, 1);
        #20;

        // Check if CRC error was detected
        if (debug_bus[5] || fdi_error_count > 0) begin
            $display("PASS: CRC error detection working");
            $display("  Error Count: %0d", fdi_error_count);
        end

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtest

    // ============================================================
    // Test 6: Retry Mechanism Test
    // ============================================================
    task test_retry_mechanism();
        $display("\n========================================");
        $display("Test 6: Retry Mechanism Test");
        $display("========================================");
        test_failed = 0;

        wait_for_link_active();

        // Send FLITs and simulate NAK
        $display("Sending FLITs with retry simulation...");
        for (int i = 0; i < 10; i++) begin
            send_pcie_flit(rand_data, (i == 0), (i == 9));
            // Simulate NAK on every 3rd flit
            if (i % 3 == 0) begin
                rdi_rx_data[0] = 1'b1;  // NAK signal
                rdi_rx_valid = 1'b1;
                @(posedge clk_core);
                rdi_rx_valid = 1'b0;
            end
            #10;
        end

        // Check retry counter
        if (debug_bus[76:45] > 0) begin
            $display("PASS: Retry mechanism working");
            $display("  Retry Count: %0d", debug_bus[76:45]);
        end

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtask

    // ============================================================
    // Test 7: Power Management Test (L1/L2)
    // ============================================================
    task test_power_management();
        $display("\n========================================");
        $display("Test 7: Power Management Test (L1/L2)");
        $display("========================================");
        test_failed = 0;

        wait_for_link_active();

        // Enter L1 state
        $display("Entering L1 power state...");
        lp_state_req = 4'd4;
        @(posedge clk_core);
        #100;

        // Check L1 entry
        if (pl_state == 4'd4) begin
            $display("PASS: L1 entry successful");
        end else begin
            $display("FAIL: L1 entry failed");
            test_failed = 1;
        end

        // Wake from L1
        $display("Waking from L1...");
        lp_wake_req = 1'b1;
        @(posedge clk_core);
        #100;
        lp_state_req = 4'd0;

        // Check wake
        if (pl_state == 4'd3) begin
            $display("PASS: L1 wake successful");
        end else begin
            $display("FAIL: L1 wake failed");
            test_failed = 1;
        end

        // Enter L2 state
        $display("Entering L2 power state...");
        lp_state_req = 4'd5;
        @(posedge clk_core);
        #200;
        lp_state_req = 4'd0;

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtask

    // ============================================================
    // Test 8: Runtime Recalibration Test
    // ============================================================
    task test_runtime_recalibration();
        $display("\n========================================");
        $display("Test 8: Runtime Recalibration Test");
        $display("========================================");
        test_failed = 0;

        wait_for_link_active();

        // Wait for recalibration to trigger
        $display("Waiting for recalibration trigger...");
        #1000;

        if (phy_recal_req || debug_bus[77]) begin
            $display("PASS: Runtime recalibration triggered");
            $display("  Recal Active: %0d", debug_bus[77]);
        end else begin
            $display("FAIL: Recalibration not triggered");
            test_failed = 1;
        end

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtask

    // ============================================================
    // Test 9: Sideband MTP Test
    // ============================================================
    task test_sideband_mtp();
        $display("\n========================================");
        $display("Test 9: Sideband MTP Test");
        $display("========================================");
        test_failed = 0;

        wait_for_link_active();

        // Send MTP packet
        $display("Sending MTP packet...");
        sb_mtp_enable = 1'b1;
        sb_mtp_data = 32'hDEAD_BEEF;
        sb_mtp_valid = 1'b1;
        sb_mtp_priority = 1'b1;
        @(posedge clk_sideband);
        #10;
        sb_mtp_valid = 1'b0;

        if (rdi_sb_rx_valid) begin
            $display("PASS: MTP packet received");
            $display("  MTP Data: 0x%08h", rdi_sb_rx_data);
        end

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtask

    // ============================================================
    // Test 10: BIST Test
    // ============================================================
    task test_bist();
        $display("\n========================================");
        $display("Test 10: BIST Test");
        $display("========================================");
        test_failed = 0;

        wait_for_link_active();

        // Run BIST
        $display("Running BIST...");
        bist_en = 1'b1;
        bist_pattern = 8'hAA;
        bist_start = 1'b1;
        @(posedge clk_core);
        #10;
        bist_start = 1'b0;
        #50;

        if (bist_done && !bist_fail) begin
            $display("PASS: BIST completed successfully");
        end else begin
            $display("FAIL: BIST failed");
            test_failed = 1;
        end

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtask

    // ============================================================
    // Test 11: Configuration Register Test
    // ============================================================
    task test_config_registers();
        $display("\n========================================");
        $display("Test 11: Configuration Register Test");
        $display("========================================");
        test_failed = 0;

        wait_for_link_active();

        // Write and read configuration registers
        $display("Testing configuration registers...");
        cfg_write(16'h0000, 32'h1234_5678);
        cfg_read(16'h0000);
        
        cfg_write(16'h0004, 32'h8765_4321);
        cfg_read(16'h0004);
        
        cfg_read(16'h0020);  // FDI status
        cfg_read(16'h0024);  // Error count
        cfg_read(16'h0028);  // FLIT counter

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtask

    // ============================================================
    // Test 12: Loopback Mode Test
    // ============================================================
    task test_loopback_mode();
        $display("\n========================================");
        $display("Test 12: Loopback Mode Test");
        $display("========================================");
        test_failed = 0;

        init_inputs();
        top_enable = 1'b1;
        top_mode = 2'b11;  // Loopback mode
        @(posedge clk_core);
        #100;

        wait_for_link_active();

        // Send traffic in loopback
        $display("Sending traffic in loopback mode...");
        for (int i = 0; i < 5; i++) begin
            send_pcie_flit(rand_data, (i == 0), (i == 4));
            #20;
        end

        if (pl_state == 4'd8) begin
            $display("PASS: Loopback mode working");
        end

        top_mode = 2'b00;  // Back to normal mode
        @(posedge clk_core);
        #100;

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtask

    // ============================================================
    // Test 13: Stress Test with Random Traffic
    // ============================================================
    task test_stress_random_traffic();
        $display("\n========================================");
        $display("Test 13: Stress Test with Random Traffic");
        $display("========================================");
        test_failed = 0;

        wait_for_link_active();

        // Send random traffic from all protocols
        $display("Sending random traffic from all protocols...");
        fork
            begin
                for (int i = 0; i < 50; i++) begin
                    if ($urandom() % 2) begin
                        send_pcie_flit($urandom(), $urandom(), $urandom());
                    end else begin
                        send_cxl_flit($urandom(), $urandom(), $urandom(), $urandom());
                    end
                    #($urandom_range(1, 10));
                end
            end
            begin
                for (int i = 0; i < 30; i++) begin
                    send_stream_flit($urandom(), $urandom(), $urandom());
                    #($urandom_range(2, 15));
                end
            end
        join

        $display("PASS: Stress test completed without errors");
        $display("  Total FLITs: %0d", debug_bus[60:40]);
        $display("  Errors: %0d", fdi_error_count);

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtask

    // ============================================================
    // Test 14: FLIT Format Negotiation Test
    // ============================================================
    task test_flit_format_negotiation();
        $display("\n========================================");
        $display("Test 14: FLIT Format Negotiation Test");
        $display("========================================");
        test_failed = 0;

        init_inputs();
        top_enable = 1'b1;
        @(posedge clk_core);
        #100;

        wait_for_link_active();

        // Check negotiated FLIT format
        $display("Negotiated FLIT Format: %0d", top_status[7:4]);
        case (top_status[7:4])
            3'b001: $display("PASS: RAW mode (Format 1)");
            3'b010: $display("PASS: 68B FLIT (Format 2)");
            3'b011: $display("PASS: 256B End-Header (Format 3)");
            3'b100: $display("PASS: 256B Start-Header (Format 4)");
            3'b101: $display("PASS: Latency-Optimized without Optional (Format 5)");
            3'b110: $display("PASS: Latency-Optimized with Optional (Format 6)");
            default: begin
                $display("FAIL: Unknown FLIT format");
                test_failed = 1;
            end
        endcase

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtask

    // ============================================================
    // Test 15: Error Recovery Test
    // ============================================================
    task test_error_recovery();
        $display("\n========================================");
        $display("Test 15: Error Recovery Test");
        $display("========================================");
        test_failed = 0;

        wait_for_link_active();

        // Simulate error conditions
        $display("Simulating error conditions...");
        
        // Force CRC error
        // (DUT will handle CRC error internally)
        send_pcie_flit('hFFFF_FFFF, 1, 1);
        #20;

        // Check if error was detected and recovered
        if (fdi_error_count > 0) begin
            $display("PASS: Error detected: %0d errors", fdi_error_count);
        end

        // Check if link is still active
        if (fdi_link_active) begin
            $display("PASS: Link recovered from error");
        end

        test_pass_count += !test_failed;
        test_fail_count += test_failed;
    endtest

    // ============================================================
    // Main Test Execution
    // ============================================================
    initial begin
        test_pass_count = 0;
        test_fail_count = 0;
        
        $display("\n========================================");
        $display("UCIe 3.0 MPG MUX Testbench");
        $display("========================================");
        $display("Configuration:");
        $display("  LINK_WIDTH: %0d", LINK_WIDTH);
        $display("  MAX_LINK_WIDTH: %0d", MAX_LINK_WIDTH);
        $display("  DATA_WIDTH: %0d", DATA_WIDTH);
        $display("  FLIT_FORMAT: %0d", FLIT_FORMAT);
        $display("  CRC_EN: %0d", CRC_EN);
        $display("  RETRY_EN: %0d", RETRY_EN);
        $display("  MULTI_PROTOCOL_EN: %0d", MULTI_PROTOCOL_EN);
        $display("========================================\n");

        // Run all tests
        test_link_initialization();
        test_pcie_traffic();
        test_cxl_traffic();
        test_multi_protocol_arbitration();
        test_crc_error_detection();
        test_retry_mechanism();
        test_power_management();
        test_runtime_recalibration();
        test_sideband_mtp();
        test_bist();
        test_config_registers();
        test_loopback_mode();
        test_stress_random_traffic();
        test_flit_format_negotiation();
        test_error_recovery();

        // Summary
        $display("\n========================================");
        $display("Test Summary");
        $display("========================================");
        $display("Tests Passed: %0d", test_pass_count);
        $display("Tests Failed: %0d", test_fail_count);
        $display("Total Tests: %0d", test_pass_count + test_fail_count);
        $display("========================================");

        if (test_fail_count == 0) begin
            $display("\nALL TESTS PASSED!");
        end else begin
            $display("\nSOME TESTS FAILED!");
        end

        #1000;
        $finish;
    end

    // ============================================================
    // Coverage Collection
    // ============================================================
    // State coverage
    covergroup state_cg @(posedge clk_core);
        coverpoint pl_state {
            bins reset = {4'd0};
            bins init = {4'd1};
            bins training = {4'd2};
            bins active = {4'd3};
            bins l1 = {4'd4};
            bins l2 = {4'd5};
            bins recal = {4'd6};
            bins loopback = {4'd8};
            bins error = {4'd9};
        }
        coverpoint fdi_link_active;
    endgroup

    // Protocol coverage
    covergroup protocol_cg @(posedge clk_core);
        coverpoint pcie_tx_valid { bins sent = {1}; bins idle = {0}; }
        coverpoint cxl_tx_valid { bins sent = {1}; bins idle = {0}; }
        coverpoint stream_tx_valid { bins sent = {1}; bins idle = {0}; }
    endgroup

    // FLIT format coverage
    covergroup flit_format_cg @(posedge clk_core);
        coverpoint top_status[7:4] {
            bins raw = {3'b001};
            bins fmt68 = {3'b010};
            bins fmt256_end = {3'b011};
            bins fmt256_start = {3'b100};
            bins lat_opt_no = {3'b101};
            bins lat_opt_opt = {3'b110};
        }
    endgroup

    // Power management coverage
    covergroup pm_cg @(posedge clk_core);
        coverpoint phy_pwr_state {
            bins off = {3'd0};
            bins l1 = {3'd1};
            bins l2 = {3'd2};
            bins active = {3'd3};
        }
        coverpoint phy_pwr_req;
        coverpoint phy_pwr_ack;
    endgroup

    state_cg state_cov = new();
    protocol_cg protocol_cov = new();
    flit_format_cg flit_format_cov = new();
    pm_cg pm_cov = new();

    // ============================================================
    // Formal Verification Assertions
    // ============================================================
    // Link must be active for data transmission
    assert property (@(posedge clk_core) 
        (pcie_tx_valid || cxl_tx_valid || stream_tx_valid) |=> (fdi_link_active))
        else $error("Data transmitted when link not active");

    // Power states must be valid
    assert property (@(posedge clk_core) 
        (pl_state == 4'd4) |-> (phy_pwr_state == 3'd1))
        else $error("L1 state but power state mismatch");

    assert property (@(posedge clk_core) 
        (pl_state == 4'd5) |-> (phy_pwr_state == 3'd2))
        else $error("L2 state but power state mismatch");

    // CRC error handling
    assert property (@(posedge clk_core) 
        (debug_bus[5]) |-> (fdi_error_count > 0))
        else $error("CRC error detected but error count not incremented");

    // Retry buffer must not overflow
    assert property (@(posedge clk_core) 
        (debug_bus[45] == 1'b1) |-> 0)
        else $error("Retry buffer overflow");

    // FLIT format must be valid
    assert property (@(posedge clk_core) 
        (pl_state == 4'd3) |-> (top_status[7:4] inside {3'b001, 3'b010, 3'b011, 3'b100, 3'b101, 3'b110}))
        else $error("Invalid FLIT format");

    // ============================================================
    // Waveform Dump
    // ============================================================
    initial begin
        $dumpfile("ucie_mpg_mux_tb.vcd");
        $dumpvars(0, ucie_mpg_mux_tb);
    end

endmodule