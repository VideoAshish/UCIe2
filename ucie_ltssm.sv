// ============================================================
// UCIe LTSSM - Link Training and Status State Machine [8†L15-L16]
// ============================================================
// States: RESET -> SBINIT -> MBINIT -> TRAINING -> LINKINIT -> ACTIVE
// Power states: L1, L2
// ============================================================

module ucie_ltssm #(
    parameter int LINK_WIDTH = 16
) (
    input  logic clk,
    input  logic rst_n,
    input  logic clk_phy,
    input  logic rst_phy_n,
    input  logic top_enable,
    input  logic [1:0] top_mode,
    input  logic phy_init_done,
    input  logic param_exchange_done,
    input  logic fdi_bringup_done,
    input  logic [3:0] lp_state_req,
    input  logic lp_wake_req,
    input  logic lp_clk_ack,
    output logic [3:0] pl_state,
    output logic pl_ready,
    output logic lp_state_ack,
    output logic lp_wake_ack,
    output logic pl_clk_req,
    output logic phy_init_start,
    output logic [5:0] phy_tx_rate,
    output logic [5:0] phy_tx_width,
    output logic top_ready
);

    typedef enum logic [3:0] {
        RESET     = 4'h0,
        SBINIT    = 4'h1,
        MBINIT    = 4'h2,
        TRAINING  = 4'h3,
        LINKINIT  = 4'h4,
        ACTIVE    = 4'h5,
        L1        = 4'h6,
        L2        = 4'h7,
        ERROR     = 4'h8
    } state_t;

    state_t state, next_state;
    logic [31:0] timer;
    logic [31:0] timeout;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= RESET;
            timer <= 32'd0;
            pl_state <= 4'd0;
            pl_ready <= 1'b0;
            phy_init_start <= 1'b0;
            phy_tx_rate <= 6'd4;
            phy_tx_width <= LINK_WIDTH;
            top_ready <= 1'b0;
            lp_state_ack <= 1'b0;
            lp_wake_ack <= 1'b0;
            pl_clk_req <= 1'b0;
        end
        else begin
            state <= next_state;
            timer <= timer + 1;

            case (state)
                RESET: begin
                    pl_state <= 4'd0;
                    pl_ready <= 1'b0;
                    top_ready <= 1'b0;
                    if (top_enable) begin
                        phy_init_start <= 1'b1;
                    end
                end

                SBINIT: begin
                    pl_state <= 4'd1;
                    // Sideband initialization complete
                end

                MBINIT: begin
                    pl_state <= 4'd2;
                    if (param_exchange_done) begin
                        next_state <= TRAINING;
                    end
                end

                TRAINING: begin
                    pl_state <= 4'd3;
                    if (phy_init_done) begin
                        next_state <= LINKINIT;
                    end
                end

                LINKINIT: begin
                    pl_state <= 4'd4;
                    if (fdi_bringup_done) begin
                        next_state <= ACTIVE;
                    end
                end

                ACTIVE: begin
                    pl_state <= 4'd5;
                    pl_ready <= 1'b1;
                    top_ready <= 1'b1;
                    if (lp_state_req == 4'd4) next_state <= L1;
                    else if (lp_state_req == 4'd5) next_state <= L2;
                end

                L1: begin
                    pl_state <= 4'd6;
                    pl_ready <= 1'b0;
                    pl_clk_req <= 1'b1;
                    lp_state_ack <= 1'b1;
                    if (lp_wake_req && lp_clk_ack) begin
                        next_state <= MBINIT;
                    end
                end

                L2: begin
                    pl_state <= 4'd7;
                    pl_ready <= 1'b0;
                    pl_clk_req <= 1'b1;
                    lp_state_ack <= 1'b1;
                    if (lp_wake_req && lp_clk_ack) begin
                        next_state <= MBINIT;
                    end
                end

                ERROR: begin
                    pl_state <= 4'd8;
                    pl_ready <= 1'b0;
                    if (top_enable) next_state <= RESET;
                end

                default: next_state <= RESET;
            endcase
        end
    end

endmodule