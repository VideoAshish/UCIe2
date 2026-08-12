module ucie3_mpg_rate_enforcer #(
    parameter int FLIT_W     = 2048,
    parameter int NUM_STACKS = 2,
    parameter int SEL_W =
        (NUM_STACKS <= 1) ? 1 : $clog2(NUM_STACKS)
) (
    input  logic                       clk,
    input  logic                       rst_n,

    // ------------------------------------------------------------
    // Candidate TX flit.
    //
    // This candidate may originate from:
    //   FDI
    //   Retry Buffer
    //   Replay
    // ------------------------------------------------------------

    input  logic                       in_valid,
    output logic                       in_ready,

    input  logic [FLIT_W-1:0]          in_flit,

    input  logic [SEL_W-1:0]           in_stack,

    // 1 => receiver permits maximum 50%
    // 0 => receiver permits 100%
    input  logic [NUM_STACKS-1:0]      bw_50,

    // Format-specific NOP flit.
    input  logic [FLIT_W-1:0]          nop_flit,

    // ------------------------------------------------------------
    // Link TX
    // ------------------------------------------------------------

    output logic                       out_valid,
    input  logic                       out_ready,

    output logic [FLIT_W-1:0]          out_flit,

    output logic                       out_is_nop,

    output logic [SEL_W-1:0]           out_stack
);

    logic               need_nop;

    logic               last_data_valid_q;
    logic [SEL_W-1:0]   last_stack_q;
    logic               last_stack_50_q;

    // ============================================================
    // Determine whether a NOP must be inserted.
    // ============================================================

    always_comb begin

        need_nop = 1'b0;

        if (in_valid &&
            last_data_valid_q &&
            last_stack_50_q &&
            bw_50[in_stack] &&
            (last_stack_q == in_stack)) begin

            need_nop = 1'b1;

        end
    end

    // ============================================================
    // Output mux
    // ============================================================

    always_comb begin

        if (need_nop) begin

            out_valid  = 1'b1;
            out_flit   = nop_flit;
            out_is_nop = 1'b1;
            out_stack  = '0;

            // The candidate remains pending.
            in_ready   = 1'b0;

        end
        else begin

            out_valid  = in_valid;
            out_flit   = in_flit;
            out_is_nop = 1'b0;
            out_stack  = in_stack;

            in_ready   = out_ready;

        end
    end

    // ============================================================
    // State
    // ============================================================

    always_ff @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            last_data_valid_q <= 1'b0;
            last_stack_q      <= '0;
            last_stack_50_q   <= 1'b0;

        end
        else if (out_valid && out_ready) begin

            if (out_is_nop) begin

                // NOP separates protocol flits.
                last_data_valid_q <= 1'b0;

            end
            else begin

                last_data_valid_q <= 1'b1;
                last_stack_q      <= out_stack;
                last_stack_50_q   <= bw_50[out_stack];

            end
        end
    end

endmodule