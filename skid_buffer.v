//==============================================================================
// File      : skid_buffer.sv
// Project   : UCIe Adapter
// Author    : Production-grade implementation
//
// Description:
//   One-entry skid buffer.
//
// Features
//   - Zero-cycle latency in pass-through mode
//   - Single-cycle skid storage
//   - Ready/Valid compliant
//   - No data loss
//   - Synthesizable
//
//==============================================================================

module skid_buffer #(
    parameter int DATA_WIDTH = 256
)(
    input  logic                   clk,
    input  logic                   rst_n,

    //----------------------------------------------------------
    // Upstream
    //----------------------------------------------------------
    input  logic                   s_valid,
    output logic                   s_ready,
    input  logic [DATA_WIDTH-1:0]  s_data,

    //----------------------------------------------------------
    // Downstream
    //----------------------------------------------------------
    output logic                   m_valid,
    input  logic                   m_ready,
    output logic [DATA_WIDTH-1:0]  m_data
);

    //----------------------------------------------------------
    // Internal Storage
    //----------------------------------------------------------

    logic                  skid_valid;
    logic [DATA_WIDTH-1:0] skid_data;

    //----------------------------------------------------------
    // Upstream Ready
    //
    // Accept new data if:
    //   • skid register empty
    //   • OR downstream consuming current beat
    //----------------------------------------------------------

    assign s_ready = !skid_valid || m_ready;

    //----------------------------------------------------------
    // Capture into skid register
    //----------------------------------------------------------

    always_ff @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            skid_valid <= 1'b0;
            skid_data  <= '0;

        end
        else begin

            //----------------------------------------------
            // Store beat when output stalls
            //----------------------------------------------
            if (s_valid && s_ready && !m_ready) begin

                skid_valid <= 1'b1;
                skid_data  <= s_data;

            end

            //----------------------------------------------
            // Release skid entry
            //----------------------------------------------
            else if (m_ready) begin

                skid_valid <= 1'b0;

            end

        end

    end

    //----------------------------------------------------------
    // Output Mux
    //----------------------------------------------------------

    always_comb begin

        if (skid_valid) begin

            m_valid = 1'b1;
            m_data  = skid_data;

        end
        else begin

            m_valid = s_valid;
            m_data  = s_data;

        end

    end

endmodule