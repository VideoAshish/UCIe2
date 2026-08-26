// ============================================================
// TL-UH Round-Robin Arbiter
// ============================================================

module tluh_arbiter #(
    parameter int NUM_MASTERS = 4,
    parameter int TLAddressWidth = 32,
    parameter int TLDataWidth   = 64,
    parameter int TLIDWidth     = 8
) (
    input  logic clk,
    input  logic arst,

    // Master interfaces (initiator)
    tlu_interface.initiator master_if [NUM_MASTERS-1:0],

    // Target interface (shared bus)
    tlu_interface.target    target_if
);

    typedef enum logic [1:0] {
        IDLE,
        WAIT_GRANT,
        WAIT_RESPONSE
    } arb_state_t;

    arb_state_t state;
    logic [$clog2(NUM_MASTERS)-1:0] current_master;
    logic [$clog2(NUM_MASTERS)-1:0] next_master;
    logic [NUM_MASTERS-1:0]         req_mask;

    // Request generation
    always_comb begin
        req_mask = '0;
        for (int i = 0; i < NUM_MASTERS; i++) begin
            req_mask[i] = master_if[i].av;
        end
    end

    // Round-robin priority encoder
    always_comb begin
        next_master = current_master;
        for (int i = 1; i <= NUM_MASTERS; i++) begin
            int idx = (current_master + i) % NUM_MASTERS;
            if (req_mask[idx]) begin
                next_master = idx;
                break;
            end
        end
    end

    // Arbitration FSM
    always_ff @(posedge clk or posedge arst) begin
        if (arst) begin
            state <= IDLE;
            current_master <= '0;
        end else begin
            case (state)
                IDLE: begin
                    if (|req_mask) begin
                        current_master <= next_master;
                        state <= WAIT_GRANT;
                    end
                end

                WAIT_GRANT: begin
                    // Connect selected master to target
                    target_if.ac <= master_if[current_master].ac;
                    target_if.ad <= master_if[current_master].ad;
                    target_if.av <= master_if[current_master].av;
                    master_if[current_master].ar <= target_if.ar;

                    if (master_if[current_master].av && target_if.ar) begin
                        state <= WAIT_RESPONSE;
                    end
                end

                WAIT_RESPONSE: begin
                    // Forward D-channel response
                    master_if[current_master].dc <= target_if.dc;
                    master_if[current_master].dd <= target_if.dd;
                    master_if[current_master].dv <= target_if.dv;
                    target_if.dr <= master_if[current_master].dr;

                    if (target_if.dv && master_if[current_master].dr) begin
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule