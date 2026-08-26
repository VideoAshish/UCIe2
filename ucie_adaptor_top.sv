// ============================================================
// UCIe 3.0 D2D Adapter - Top-Level Module
// ============================================================
// This module implements the Die-to-Die (D2D) Adapter layer for UCIe 3.0
// Reference: UCIe 3.0 Specification, wiowiz.com reference implementation [6]
// 
// Key Features:
// 1. Link Training and Status State Machine (LTSSM) [8†L7]
// 2. Sideband parameter negotiation [3†L30-L31]
// 3. Mainband CRC-32 protection [6†L33]
// 4. FLIT packing for multiple formats (Raw, 68B, 256B)
// 5. Protocol arbitration and multiplexing
// 6. FDI (Flit-aware D2D Interface) and RDI (Raw D2D Interface) [3†L5-L6]
// 7. Power management (L1, L2 states)
// ============================================================

`include "ucie_3.0_defines.sv"

module ucie_d2d_adapter_top #(
    // Core Configuration Parameters
    parameter int LINK_WIDTH          = 16,    // Active lanes: 8, 16, 32, 64
    parameter int FDI_DATA_WIDTH      = 256,   // FDI mainband width
    parameter int FDI_SB_WIDTH        = 32,    // FDI sideband width
    parameter int RDI_WIDTH           = 512,   // RDI width = LINK_WIDTH * 32
    parameter int NUM_PROTOCOLS       = 3,     // PCIe, CXL, Streaming
    parameter int FLIT_FORMAT         = 3,     // 1:Raw, 2:68B, 3:256B End-Header
    parameter int CRC_EN              = 1,
    parameter int RETRY_EN            = 1,
    parameter int RETRY_BUFFER_DEPTH  = 32,
    parameter int MULTI_PROTOCOL_EN   = 1
) (
    // Clock and Reset
    input  logic clk_core,
    input  logic clk_phy,
    input  logic clk_sideband,
    input  logic rst_core_n,
    input  logic rst_phy_n,
    input  logic rst_sideband_n,

    // FDI Interface (Protocol to Adapter) [9†L31-L33]
    // Protocol 0: PCIe
    input  logic [FDI_DATA_WIDTH-1:0] pcie_tx_data,
    input  logic                      pcie_tx_valid,
    output logic                      pcie_tx_ready,
    input  logic                      pcie_tx_sop,
    input  logic                      pcie_tx_eop,
    input  logic [3:0]                pcie_tx_credits,
    input  logic [1:0]                pcie_tx_fmt,

    // Protocol 1: CXL
    input  logic [FDI_DATA_WIDTH-1:0] cxl_tx_data,
    input  logic                      cxl_tx_valid,
    output logic                      cxl_tx_ready,
    input  logic                      cxl_tx_sop,
    input  logic                      cxl_tx_eop,
    input  logic [3:0]                cxl_tx_credits,
    input  logic [1:0]                cxl_tx_fmt,

    // Protocol 2: Streaming (Raw)
    input  logic [FDI_DATA_WIDTH-1:0] stream_tx_data,
    input  logic                      stream_tx_valid,
    output logic                      stream_tx_ready,
    input  logic                      stream_tx_sop,
    input  logic                      stream_tx_eop,
    input  logic [3:0]                stream_tx_credits,

    // FDI Sideband
    input  logic [FDI_SB_WIDTH-1:0]   fdi_sb_tx_data,
    input  logic                      fdi_sb_tx_valid,
    output logic                      fdi_sb_tx_ready,
    output logic [FDI_SB_WIDTH-1:0]   fdi_sb_rx_data,
    output logic                      fdi_sb_rx_valid,

    // FDI Configuration
    input  logic [31:0]               fdi_cfg_wdata,
    input  logic                      fdi_cfg_wvalid,
    output logic                      fdi_cfg_wready,
    input  logic [15:0]               fdi_cfg_addr,
    input  logic                      fdi_cfg_rw,
    output logic [31:0]               fdi_cfg_rdata,
    output logic                      fdi_cfg_rvalid,

    // FDI Status
    output logic                      fdi_link_active,
    output logic [31:0]               fdi_status,
    output logic [15:0]               fdi_error_count,

    // RDI Interface (Adapter to PHY) [9†L32-L33]
    output logic [RDI_WIDTH-1:0]      rdi_tx_data,
    output logic                      rdi_tx_valid,
    input  logic                      rdi_tx_ready,
    input  logic [RDI_WIDTH-1:0]      rdi_rx_data,
    input  logic                      rdi_rx_valid,
    output logic                      rdi_rx_ready,

    // RDI Link State Management
    output logic [3:0]                pl_state,
    output logic                      pl_ready,
    input  logic [3:0]                lp_state_req,
    output logic                      lp_state_ack,
    input  logic                      lp_wake_req,
    output logic                      lp_wake_ack,
    output logic                      pl_clk_req,
    input  logic                      lp_clk_ack,

    // PHY Interface
    output logic [RDI_WIDTH-1:0]      phy_tx_data,
    output logic                      phy_tx_valid,
    input  logic                      phy_tx_ready,
    output logic [5:0]                phy_tx_rate,
    output logic [5:0]                phy_tx_width,
    input  logic [RDI_WIDTH-1:0]      phy_rx_data,
    input  logic                      phy_rx_valid,
    output logic                      phy_rx_ready,
    output logic                      phy_init_start,
    input  logic                      phy_init_done,

    // Power Management
    output logic                      phy_pwr_req,
    input  logic                      phy_pwr_ack,
    output logic [2:0]                phy_pwr_state,

    // Test and Debug
    input  logic                      top_enable,
    input  logic [1:0]                top_mode,
    output logic                      top_ready,
    output logic [31:0]               top_status,
    output logic [127:0]              debug_bus
);

    // ============================================================
    // Internal Signals
    // ============================================================
    logic [FDI_DATA_WIDTH-1:0] arb_data;
    logic                      arb_valid;
    logic                      arb_ready;
    logic                      arb_sop;
    logic                      arb_eop;
    logic [1:0]                arb_fmt;
    logic [3:0]                arb_credits;
    logic [FDI_SB_WIDTH-1:0]   arb_sb_data;
    logic                      arb_sb_valid;

    logic [31:0]               crc_calc;
    logic                      crc_error;
    logic                      crc_valid;

    logic [255:0]              flit_data;
    logic                      flit_valid;
    logic                      flit_ready;

    logic                      param_exchange_complete;
    logic                      fdi_bringup_complete;
    logic [2:0]                negotiated_format;

    // ============================================================
    // Sub-module Instantiation
    // ============================================================

    // 1. Link Training State Machine (LTSSM) [6†L32][8†L7]
    ucie_ltssm #(
        .LINK_WIDTH(LINK_WIDTH)
    ) u_ltssm (
        .clk                (clk_core),
        .rst_n              (rst_core_n),
        .clk_phy            (clk_phy),
        .rst_phy_n          (rst_phy_n),
        .top_enable         (top_enable),
        .top_mode           (top_mode),
        .phy_init_done      (phy_init_done),
        .param_exchange_done(param_exchange_complete),
        .fdi_bringup_done   (fdi_bringup_complete),
        .lp_state_req       (lp_state_req),
        .lp_wake_req        (lp_wake_req),
        .lp_clk_ack         (lp_clk_ack),
        .pl_state           (pl_state),
        .pl_ready           (pl_ready),
        .lp_state_ack       (lp_state_ack),
        .lp_wake_ack        (lp_wake_ack),
        .pl_clk_req         (pl_clk_req),
        .phy_init_start     (phy_init_start),
        .phy_tx_rate        (phy_tx_rate),
        .phy_tx_width       (phy_tx_width),
        .top_ready          (top_ready)
    );

    // 2. Sideband Parameter Negotiation [6†L33][3†L30-L31]
    ucie_sideband #(
        .SB_WIDTH(FDI_SB_WIDTH)
    ) u_sideband (
        .clk                (clk_sideband),
        .rst_n              (rst_sideband_n),
        .clk_core           (clk_core),
        .rst_core_n         (rst_core_n),
        .sb_tx_data         (fdi_sb_tx_data),
        .sb_tx_valid        (fdi_sb_tx_valid),
        .sb_tx_ready        (fdi_sb_tx_ready),
        .sb_rx_data         (fdi_sb_rx_data),
        .sb_rx_valid        (fdi_sb_rx_valid),
        .pl_ready           (pl_ready),
        .flit_format_req    (FLIT_FORMAT),
        .crc_en_req         (CRC_EN),
        .retry_en_req       (RETRY_EN),
        .multi_proto_req    (MULTI_PROTOCOL_EN),
        .negotiated_format  (negotiated_format),
        .param_exchange_done(param_exchange_complete)
    );

    // 3. Protocol Arbitration [9†L44-L45]
    ucie_protocol_arbiter #(
        .NUM_PROTOCOLS(NUM_PROTOCOLS),
        .DATA_WIDTH(FDI_DATA_WIDTH)
    ) u_arbiter (
        .clk                (clk_core),
        .rst_n              (rst_core_n),
        .pl_ready           (pl_ready),
        // PCIe inputs
        .pcie_data          (pcie_tx_data),
        .pcie_valid         (pcie_tx_valid),
        .pcie_ready         (pcie_tx_ready),
        .pcie_sop           (pcie_tx_sop),
        .pcie_eop           (pcie_tx_eop),
        .pcie_credits       (pcie_tx_credits),
        .pcie_fmt           (pcie_tx_fmt),
        // CXL inputs
        .cxl_data           (cxl_tx_data),
        .cxl_valid          (cxl_tx_valid),
        .cxl_ready          (cxl_tx_ready),
        .cxl_sop            (cxl_tx_sop),
        .cxl_eop            (cxl_tx_eop),
        .cxl_credits        (cxl_tx_credits),
        .cxl_fmt            (cxl_tx_fmt),
        // Streaming inputs
        .stream_data        (stream_tx_data),
        .stream_valid       (stream_tx_valid),
        .stream_ready       (stream_tx_ready),
        .stream_sop         (stream_tx_sop),
        .stream_eop         (stream_tx_eop),
        .stream_credits     (stream_tx_credits),
        // Arbiter outputs
        .arb_data           (arb_data),
        .arb_valid          (arb_valid),
        .arb_ready          (arb_ready),
        .arb_sop            (arb_sop),
        .arb_eop            (arb_eop),
        .arb_fmt            (arb_fmt),
        .arb_credits        (arb_credits)
    );

    // 4. CRC-32 Generator [6†L33]
    ucie_mainband_crc #(
        .DATA_WIDTH(FDI_DATA_WIDTH)
    ) u_crc (
        .clk                (clk_core),
        .rst_n              (rst_core_n),
        .data_in            (arb_data),
        .data_valid         (arb_valid),
        .data_ready         (arb_ready),
        .crc_out            (crc_calc),
        .crc_valid          (crc_valid),
        .crc_error          (crc_error)
    );

    // 5. FLIT Packing Engine
    ucie_flit_packer #(
        .DATA_WIDTH(FDI_DATA_WIDTH),
        .RDI_WIDTH(RDI_WIDTH),
        .FLIT_FORMAT(FLIT_FORMAT)
    ) u_flit_packer (
        .clk                (clk_core),
        .rst_n              (rst_core_n),
        .pl_ready           (pl_ready),
        .arb_data           (arb_data),
        .arb_valid          (arb_valid),
        .arb_ready          (arb_ready),
        .arb_sop            (arb_sop),
        .arb_eop            (arb_eop),
        .arb_fmt            (arb_fmt),
        .arb_credits        (arb_credits),
        .crc_in             (crc_calc),
        .crc_valid          (crc_valid),
        .negotiated_format  (negotiated_format),
        .flit_data          (flit_data),
        .flit_valid         (flit_valid),
        .flit_ready         (flit_ready)
    );

    // 6. RDI Interface (Adapter to PHY)
    ucie_rdi_interface #(
        .RDI_WIDTH(RDI_WIDTH)
    ) u_rdi (
        .clk_phy            (clk_phy),
        .rst_phy_n          (rst_phy_n),
        .clk_core           (clk_core),
        .rst_core_n         (rst_core_n),
        .pl_ready           (pl_ready),
        .flit_data          (flit_data),
        .flit_valid         (flit_valid),
        .flit_ready         (flit_ready),
        .phy_tx_ready       (phy_tx_ready),
        .phy_rx_data        (phy_rx_data),
        .phy_rx_valid       (phy_rx_valid),
        .rdi_tx_data        (rdi_tx_data),
        .rdi_tx_valid       (rdi_tx_valid),
        .rdi_tx_ready       (rdi_tx_ready),
        .rdi_rx_data        (rdi_rx_data),
        .rdi_rx_valid       (rdi_rx_valid),
        .rdi_rx_ready       (rdi_rx_ready),
        .phy_tx_data        (phy_tx_data),
        .phy_tx_valid       (phy_tx_valid),
        .phy_rx_ready       (phy_rx_ready)
    );

    // 7. Power Management
    ucie_power_manager u_pm (
        .clk                (clk_core),
        .rst_n              (rst_core_n),
        .pl_state           (pl_state),
        .lp_state_req       (lp_state_req),
        .lp_state_ack       (lp_state_ack),
        .phy_pwr_req        (phy_pwr_req),
        .phy_pwr_ack        (phy_pwr_ack),
        .phy_pwr_state      (phy_pwr_state)
    );

    // ============================================================
    // Status and Debug Outputs
    // ============================================================
    assign fdi_link_active = pl_ready;
    assign fdi_status = {pl_state, pl_ready, fdi_link_active, 26'b0};

    assign debug_bus = {
        pl_state,
        pl_ready,
        flit_valid,
        crc_error,
        param_exchange_complete,
        fdi_bringup_complete,
        negotiated_format,
        112'b0
    };

endmodule