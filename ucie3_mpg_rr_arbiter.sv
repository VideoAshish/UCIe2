module ucie3_mpg_rr_arbiter #(
    parameter int NUM_STACKS = 2,
    parameter int SEL_W =
        (NUM_STACKS <= 1) ? 1 : $clog2(NUM_STACKS)
) (
    input  logic                       clk,
    input  logic                       rst_n,

    input  logic [NUM_STACKS-1:0]      req_valid,
    output logic [NUM_STACKS-1:0]      req_ready,

    output logic                       grant_valid,
    output logic [SEL_W-1:0]           grant_id,

    input  logic                       grant_ready
);

    logic [SEL_W-1:0] rr_ptr_q;

    logic             hold_q;
    logic [SEL_W-1:0] hold_id_q;

    logic             grant_valid_c;
    logic [SEL_W-1:0] grant_id_c;

    integer i;
    integer idx;

    // ============================================================
    // Combinational arbitration
    // ============================================================

    always_comb begin

        grant_valid_c = 1'b0;
        grant_id_c    = '0;

        if (hold_q) begin

            grant_valid_c = req_valid[hold_id_q];
            grant_id_c    = hold_id_q;

        end
        else begin

            for (i = 0; i < NUM_STACKS; i++) begin

                idx = rr_ptr_q + i;

                if (idx >= NUM_STACKS)
                    idx = idx - NUM_STACKS;

                if (!grant_valid_c &&
                    req_valid[idx]) begin

                    grant_valid_c = 1'b1;
                    grant_id_c    = idx[SEL_W-1:0];

                end
            end
        end
    end

    // ============================================================
    // Handshake
    // ============================================================

    always_comb begin

        grant_valid = grant_valid_c;
        grant_id    = grant_id_c;

        req_ready = '0;

        if (grant_valid_c)
            req_ready[grant_id_c] = grant_ready;

    end

    // ============================================================
    // State
    // ============================================================

    always_ff @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            rr_ptr_q <= '0;

            hold_q   <= 1'b0;
            hold_id_q <= '0;

        end
        else begin

            // ----------------------------------------------------
            // Hold selected request while output is stalled.
            // ----------------------------------------------------

            if (!hold_q) begin

                if (grant_valid_c && !grant_ready) begin

                    hold_q    <= 1'b1;
                    hold_id_q <= grant_id_c;

                end
            end
            else begin

                if (grant_ready)
                    hold_q <= 1'b0;

            end

            // ----------------------------------------------------
            // Round-robin pointer updates only on accepted flit.
            // ----------------------------------------------------

            if (grant_valid && grant_ready) begin

                if (grant_id == NUM_STACKS-1)
                    rr_ptr_q <= '0;
                else
                    rr_ptr_q <= grant_id + 1'b1;

            end
        end
    end

endmodule