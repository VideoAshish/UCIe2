module ucie3_mpg_mux #(
    parameter int FLIT_W = 2048,
    parameter int NUM_STACKS = 2,

    parameter int STACK_ID_LSB = 5
) (
    input  logic                         clk,
    input  logic                         rst_n,

    input  logic [NUM_STACKS-1:0]        s_valid,
    output logic [NUM_STACKS-1:0]        s_ready,

    input  logic [FLIT_W-1:0]            s_flit
                                               [NUM_STACKS],

    output logic                         m_valid,
    input  logic                         m_ready,

    output logic [FLIT_W-1:0]            m_flit,

    output logic [$clog2(NUM_STACKS)-1:0] m_stack
);

    localparam int SEL_W =
        (NUM_STACKS <= 1) ? 1 : $clog2(NUM_STACKS);

    logic [SEL_W-1:0] grant_id;

    // ============================================================
    // Arbiter
    // ============================================================

    ucie3_mpg_rr_arbiter #(
        .NUM_STACKS(NUM_STACKS)
    ) u_rr (
        .clk         (clk),
        .rst_n       (rst_n),

        .req_valid   (s_valid),
        .req_ready   (s_ready),

        .grant_valid (m_valid),
        .grant_id    (grant_id),

        .grant_ready (m_ready)
    );

    // ============================================================
    // Flit MUX
    // ============================================================

    always_comb begin

        m_flit = '0;

        if (m_valid) begin

            m_flit = s_flit[grant_id];

            // ----------------------------------------------------
            // Stack identifier.
            //
            // The exact field location should be bound to the
            // negotiated UCIe 3.0 flit format.
            // ----------------------------------------------------

            m_flit[STACK_ID_LSB] =
                (grant_id != '0);

        end
    end

    assign m_stack = grant_id;

endmodule