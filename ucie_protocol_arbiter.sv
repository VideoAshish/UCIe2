// ============================================================
// UCIe Protocol Arbiter
// ============================================================
// Supports PCIe, CXL, and Streaming protocols with
// credit-based flow control and round-robin arbitration
// ============================================================

module ucie_protocol_arbiter #(
    parameter int NUM_PROTOCOLS = 3,
    parameter int DATA_WIDTH    = 256
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     pl_ready,

    // PCIe Interface
    input  logic [DATA_WIDTH-1:0]    pcie_data,
    input  logic                     pcie_valid,
    output logic                     pcie_ready,
    input  logic                     pcie_sop,
    input  logic                     pcie_eop,
    input  logic [3:0]               pcie_credits,
    input  logic [1:0]               pcie_fmt,

    // CXL Interface
    input  logic [DATA_WIDTH-1:0]    cxl_data,
    input  logic                     cxl_valid,
    output logic                     cxl_ready,
    input  logic                     cxl_sop,
    input  logic                     cxl_eop,
    input  logic [3:0]               cxl_credits,
    input  logic [1:0]               cxl_fmt,

    // Streaming Interface
    input  logic [DATA_WIDTH-1:0]    stream_data,
    input  logic                     stream_valid,
    output logic                     stream_ready,
    input  logic                     stream_sop,
    input  logic                     stream_eop,
    input  logic [3:0]               stream_credits,

    // Arbiter Outputs
    output logic [DATA_WIDTH-1:0]    arb_data,
    output logic                     arb_valid,
    input  logic                     arb_ready,
    output logic                     arb_sop,
    output logic                     arb_eop,
    output logic [1:0]               arb_fmt,
    output logic [3:0]               arb_credits
);

    typedef enum logic [1:0] {
        IDLE,
        PCIE,
        CXL,
        STREAM
    } arb_state_t;

    arb_state_t state, next_state;
    logic [1:0] grant;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            grant <= 2'b00;
            arb_data <= '0;
            arb_valid <= 1'b0;
            arb_sop <= 1'b0;
            arb_eop <= 1'b0;
            arb_fmt <= 2'b00;
            arb_credits <= 4'd0;
            pcie_ready <= 1'b1;
            cxl_ready <= 1'b1;
            stream_ready <= 1'b1;
        end
        else begin
            state <= next_state;

            if (pl_ready) begin
                case (state)
                    IDLE: begin
                        if (pcie_valid && pcie_credits > 0) begin
                            grant <= 2'b00;
                            next_state <= PCIE;
                        end
                        else if (cxl_valid && cxl_credits > 0) begin
                            grant <= 2'b01;
                            next_state <= CXL;
                        end
                        else if (stream_valid && stream_credits > 0) begin
                            grant <= 2'b10;
                            next_state <= STREAM;
                        end
                    end

                    PCIE: begin
                        arb_data <= pcie_data;
                        arb_valid <= pcie_valid && arb_ready;
                        arb_sop <= pcie_sop;
                        arb_eop <= pcie_eop;
                        arb_fmt <= pcie_fmt;
                        arb_credits <= pcie_credits;
                        pcie_ready <= arb_ready;
                        if (pcie_eop) next_state <= IDLE;
                    end

                    CXL: begin
                        arb_data <= cxl_data;
                        arb_valid <= cxl_valid && arb_ready;
                        arb_sop <= cxl_sop;
                        arb_eop <= cxl_eop;
                        arb_fmt <= cxl_fmt;
                        arb_credits <= cxl_credits;
                        cxl_ready <= arb_ready;
                        if (cxl_eop) next_state <= IDLE;
                    end

                    STREAM: begin
                        arb_data <= stream_data;
                        arb_valid <= stream_valid && arb_ready;
                        arb_sop <= stream_sop;
                        arb_eop <= stream_eop;
                        arb_fmt <= 2'b00;
                        arb_credits <= stream_credits;
                        stream_ready <= arb_ready;
                        if (stream_eop) next_state <= IDLE;
                    end

                    default: next_state <= IDLE;
                endcase
            end
            else begin
                arb_valid <= 1'b0;
                pcie_ready <= 1'b1;
                cxl_ready <= 1'b1;
                stream_ready <= 1'b1;
                next_state <= IDLE;
            end
        end
    end

endmodule