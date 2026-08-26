//==============================================================================
// File : ucie_tx_top.sv
// Description : UCIe TX datapath top
//==============================================================================

module ucie_tx_top
#(
    parameter int DATA_WIDTH = 256,
    parameter int FLIT_WIDTH = 256
)
(
    input  logic clk,
    input  logic rst_n,

    //------------------------------------------------------------
    // Application Interface
    //------------------------------------------------------------
    input  logic                    app_valid,
    output logic                    app_ready,
    input  logic [DATA_WIDTH-1:0]   app_data,
    input  logic                    app_sop,
    input  logic                    app_eop,

    //------------------------------------------------------------
    // ACK from RX
    //------------------------------------------------------------
    input  logic                    ack_valid,
    input  logic [9:0]              ack_seq,

    //------------------------------------------------------------
    // Credit Return
    //------------------------------------------------------------
    input  logic [7:0]              credit_return,

    //------------------------------------------------------------
    // Link Interface
    //------------------------------------------------------------
    output logic                    tx_valid,
    input  logic                    tx_ready,
    output logic [FLIT_WIDTH-1:0]   tx_flit
);

    //-----------------------------------------------------------------
    // Internal Pipeline Signals
    //-----------------------------------------------------------------

    logic                    seg_valid;
    logic                    seg_ready;
    logic [DATA_WIDTH-1:0]   seg_data;
    logic                    seg_sop;
    logic                    seg_eop;

    logic                    vc_valid;
    logic                    vc_ready;
    logic [DATA_WIDTH-1:0]   vc_data;
    logic                    vc_sop;
    logic                    vc_eop;

    logic                    fmt_valid;
    logic                    fmt_ready;
    logic [FLIT_WIDTH-1:0]   fmt_flit;
    logic [9:0]              fmt_seq;

    logic                    rb_valid;
    logic                    rb_ready;
    logic [FLIT_WIDTH-1:0]   rb_flit;

    logic                    replay_valid;
    logic [FLIT_WIDTH-1:0]   replay_flit;

    logic                    credit_ok;

    //-----------------------------------------------------------------
    // Packet Segmenter
    //-----------------------------------------------------------------

    ucie_packet_segmenter u_segmenter
    (
        .clk      (clk),
        .rst_n    (rst_n),

        .in_valid (app_valid),
        .in_ready (app_ready),
        .in_data  (app_data),
        .in_sop   (app_sop),
        .in_eop   (app_eop),

        .out_valid(seg_valid),
        .out_ready(seg_ready),
        .out_data (seg_data),
        .out_sop  (seg_sop),
        .out_eop  (seg_eop)
    );

    //-----------------------------------------------------------------
    // VC Scheduler
    //-----------------------------------------------------------------

    ucie_vc_scheduler u_vc
    (
        .clk(clk),
        .rst_n(rst_n),

        .in_valid(seg_valid),
        .in_ready(seg_ready),
        .in_data(seg_data),
        .in_sop(seg_sop),
        .in_eop(seg_eop),

        .out_valid(vc_valid),
        .out_ready(vc_ready),
        .out_data(vc_data),
        .out_sop(vc_sop),
        .out_eop(vc_eop)
    );

    //-----------------------------------------------------------------
    // Flit Formatter
    //-----------------------------------------------------------------

    ucie_flit_formatter u_formatter
    (
        .clk(clk),
        .rst_n(rst_n),

        .in_valid(vc_valid),
        .in_ready(vc_ready),
        .in_data(vc_data),
        .in_sop(vc_sop),
        .in_eop(vc_eop),

        .out_valid(fmt_valid),
        .out_ready(fmt_ready),
        .out_flit(fmt_flit),

        .out_seq(fmt_seq)
    );

    //-----------------------------------------------------------------
    // Replay Buffer
    //-----------------------------------------------------------------

    ucie_replay_buffer u_replay_buffer
    (
        .clk(clk),
        .rst_n(rst_n),

        .wr_en(fmt_valid & fmt_ready),
        .wr_seq(fmt_seq),
        .wr_data(fmt_flit),

        .ack_valid(ack_valid),
        .ack_seq(ack_seq),

        .replay_req(1'b0),          // connected to replay engine
        .replay_seq('0),

        .replay_data(replay_flit),
        .replay_valid(replay_valid)
    );

    //-----------------------------------------------------------------
    // Credit Manager
    //-----------------------------------------------------------------

    ucie_credit_manager u_credit
    (
        .clk(clk),
        .rst_n(rst_n),

        .credit_return(credit_return),

        .consume(fmt_valid & fmt_ready),

        .credit_ok(credit_ok)
    );

    //-----------------------------------------------------------------
    // Output Skid Buffer
    //-----------------------------------------------------------------

    skid_buffer
    #(
        .DATA_WIDTH(FLIT_WIDTH)
    )
    u_skid
    (
        .clk(clk),
        .rst_n(rst_n),

        .s_valid(fmt_valid & credit_ok),
        .s_ready(fmt_ready),
        .s_data(fmt_flit),

        .m_valid(tx_valid),
        .m_ready(tx_ready),
        .m_data(tx_flit)
    );

endmodule