module ucie3_mpg #(
    parameter int FLIT_W       = 2048,
    parameter int NUM_STACKS   = 2,

    parameter int STACK_ID_LSB = 5,

    parameter bit ENABLE_SVA   = 1'b1
) (
    input logic                         clk,
    input logic                         rst_n,

    // ============================================================
    // Protocol-stack interfaces
    // ============================================================

    input  logic [NUM_STACKS-1:0]       stack_valid,

    output logic [NUM_STACKS-1:0]       stack_ready,

    input logic [FLIT_W-1:0]            stack_flit
                                             [NUM_STACKS],

    // ============================================================
    // Negotiated receiver bandwidth capability
    //
    // 0 = 100%
    // 1 = 50%
    // ============================================================

    input logic [NUM_STACKS-1:0]        stack_bw_50,

    // ============================================================
    // Replay / retry candidate
    //
    // This interface is intentionally separate from FDI.
    // The retry manager decides when replay traffic is eligible.
    // ============================================================

    input logic                         replay_valid,
    output logic                         replay_ready,

    input logic [FLIT_W-1:0]             replay_flit,

    input logic [$clog2(NUM_STACKS)-1:0] replay_stack,

    // ============================================================
    // Final Link TX
    // ============================================================

    output logic                         tx_valid,
    input logic                          tx_ready,

    output logic [FLIT_W-1:0]             tx_flit,

    output logic                         tx_is_nop,

    output logic [$clog2(NUM_STACKS)-1:0] tx_stack,

    // ============================================================
    // Negotiated NOP
    // ============================================================

    input logic [FLIT_W-1:0]             nop_flit
);

    localparam int SEL_W =
        (NUM_STACKS <= 1) ? 1 : $clog2(NUM_STACKS);

    // ------------------------------------------------------------
    // FDI MUX
    // ------------------------------------------------------------

    logic                    mux_valid;
    logic                    mux_ready;

    logic [FLIT_W-1:0]       mux_flit;
    logic [SEL_W-1:0]       mux_stack;

    // ------------------------------------------------------------
    // Candidate arbitration between:
    //
    //   FDI-generated traffic
    //   Replay traffic
    //
    // ------------------------------------------------------------

    logic                    candidate_valid;
    logic                    candidate_ready;

    logic [FLIT_W-1:0]       candidate_flit;
    logic [SEL_W-1:0]        candidate_stack;

    logic                    fdi_candidate_ready;
    logic                    replay_candidate_ready;

    // ============================================================
    // FDI MPG MUX
    // ============================================================

    ucie3_mpg_mux #(
        .FLIT_W       (FLIT_W),
        .NUM_STACKS   (NUM_STACKS),
        .STACK_ID_LSB (STACK_ID_LSB)
    ) u_mpg_mux (
        .clk       (clk),
        .rst_n     (rst_n),

        .s_valid   (stack_valid),
        .s_ready   (stack_ready),
        .s_flit    (stack_flit),

        .m_valid   (mux_valid),
        .m_ready   (mux_ready),

        .m_flit    (mux_flit),
        .m_stack   (mux_stack)
    );

    // ============================================================
    // Replay / FDI candidate mux
    //
    // Replay is given priority when valid. The retry controller
    // should normally manage replay ordering.
    //
    // For a design requiring strict fairness between replay and
    // new traffic, replace this with a dedicated 2-way arbiter.
    // ============================================================

    always_comb begin

        candidate_valid = 1'b0;
        candidate_flit  = '0;
        candidate_stack = '0;

        mux_ready       = 1'b0;
        replay_ready    = 1'b0;

        if (replay_valid) begin

            candidate_valid = 1'b1;
            candidate_flit  = replay_flit;
            candidate_stack = replay_stack;

            replay_ready = candidate_ready;

        end
        else if (mux_valid) begin

            candidate_valid = 1'b1;
            candidate_flit  = mux_flit;
            candidate_stack = mux_stack;

            mux_ready = candidate_ready;

        end
    end

    // ============================================================
    // 50% bandwidth enforcement
    // ============================================================

    ucie3_mpg_rate_enforcer #(
        .FLIT_W     (FLIT_W),
        .NUM_STACKS (NUM_STACKS)
    ) u_rate (
        .clk        (clk),
        .rst_n      (rst_n),

        .in_valid   (candidate_valid),
        .in_ready   (candidate_ready),

        .in_flit    (candidate_flit),
        .in_stack   (candidate_stack),

        .bw_50      (stack_bw_50),

        .nop_flit   (nop_flit),

        .out_valid  (tx_valid),
        .out_ready  (tx_ready),

        .out_flit   (tx_flit),
        .out_is_nop (tx_is_nop),
        .out_stack  (tx_stack)
    );

endmodule