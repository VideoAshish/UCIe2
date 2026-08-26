//==============================================================================
// File      : flit_formatter.sv
// Project   : Production UCIe Adapter
// Description:
//      UCIe TX Flit Formatter
//
// Notes
//  • Synthesizable
//  • Ready/Valid compliant
//  • One flit per cycle
//  • CRC generated in next stage
//==============================================================================

module ucie_flit_formatter
#(
    parameter int DATA_WIDTH    = 256,
    parameter int FLIT_WIDTH    = 256,
    parameter int HEADER_WIDTH  = 32,
    parameter int VC_WIDTH      = 3,
    parameter int SEQ_WIDTH     = 10,
    parameter int CTRL_WIDTH    = 8
)
(
    input  logic                     clk,
    input  logic                     rst_n,

    //----------------------------------------------------------
    // Packet Input
    //----------------------------------------------------------

    input  logic                     in_valid,
    output logic                     in_ready,

    input  logic [DATA_WIDTH-1:0]    in_data,
    input  logic                     in_sop,
    input  logic                     in_eop,

    input  logic [VC_WIDTH-1:0]      in_vc,

    //----------------------------------------------------------
    // Output to CRC stage
    //----------------------------------------------------------

    output logic                     out_valid,
    input  logic                     out_ready,

    output logic [FLIT_WIDTH-1:0]    out_flit,

    output logic [SEQ_WIDTH-1:0]     out_seq
);

    //----------------------------------------------------------------------
    // Derived
    //----------------------------------------------------------------------

    localparam int PAYLOAD_WIDTH =
        FLIT_WIDTH -
        HEADER_WIDTH -
        VC_WIDTH -
        SEQ_WIDTH -
        CTRL_WIDTH;

    //----------------------------------------------------------------------
    // Sequence Counter
    //----------------------------------------------------------------------

    logic [SEQ_WIDTH-1:0] seq_counter;

    always_ff @(posedge clk or negedge rst_n)
    begin
        if(!rst_n)
            seq_counter <= '0;
        else if(in_valid && in_ready)
            seq_counter <= seq_counter + 1'b1;
    end

    assign out_seq = seq_counter;

    //----------------------------------------------------------------------
    // Header
    //----------------------------------------------------------------------

    logic [HEADER_WIDTH-1:0] header;

    always_comb
    begin

        header = '0;

        header[31:24] = 8'hAA;     // Protocol ID
        header[23:16] = 8'h01;     // Packet Type
        header[15:8]  = 8'h00;     // Source
        header[7:0]   = 8'h01;     // Destination

    end

    //----------------------------------------------------------------------
    // Control Field
    //----------------------------------------------------------------------

    logic [CTRL_WIDTH-1:0] ctrl;

    always_comb
    begin

        ctrl = '0;

        ctrl[7] = in_sop;
        ctrl[6] = in_eop;

        ctrl[5] = 1'b0;    // poison

        ctrl[4] = 1'b0;    // replay

        ctrl[3:0] = 4'b0000;

    end

    //----------------------------------------------------------------------
    // Payload
    //----------------------------------------------------------------------

    logic [PAYLOAD_WIDTH-1:0] payload;

    always_comb
    begin

        payload = '0;

        if(DATA_WIDTH >= PAYLOAD_WIDTH)
            payload = in_data[PAYLOAD_WIDTH-1:0];
        else
            payload[DATA_WIDTH-1:0] = in_data;

    end

    //----------------------------------------------------------------------
    // Assemble Flit
    //----------------------------------------------------------------------

    logic [FLIT_WIDTH-1:0] flit_next;

    always_comb
    begin

        flit_next = '0;

        flit_next = {

            header,

            seq_counter,

            in_vc,

            ctrl,

            payload

        };

    end

    //----------------------------------------------------------------------
    // Output Register
    //----------------------------------------------------------------------

    logic                     valid_r;
    logic [FLIT_WIDTH-1:0]    flit_r;

    assign in_ready = out_ready | !valid_r;

    always_ff @(posedge clk or negedge rst_n)
    begin

        if(!rst_n)
        begin

            valid_r <= 1'b0;
            flit_r  <= '0;

        end
        else
        begin

            if(in_ready)
            begin

                valid_r <= in_valid;

                if(in_valid)
                    flit_r <= flit_next;

            end

        end

    end

    assign out_valid = valid_r;

    assign out_flit  = flit_r;

endmodule