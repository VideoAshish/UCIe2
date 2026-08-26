// ============================================================
// UCIe FLIT Packing Engine
// ============================================================
// Formats:
//   1: Raw (64B data, no header, no CRC)
//   2: 68B (64B data + 2B header + 2B CRC)
//   3: 256B End-Header (256B data + 16B header at end)
//   4: 256B Start-Header (256B data + 16B header at start)
// ============================================================

module ucie_flit_packer #(
    parameter int DATA_WIDTH = 256,
    parameter int RDI_WIDTH  = 512,
    parameter int FLIT_FORMAT = 3
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     pl_ready,
    input  logic [DATA_WIDTH-1:0]    arb_data,
    input  logic                     arb_valid,
    output logic                     arb_ready,
    input  logic                     arb_sop,
    input  logic                     arb_eop,
    input  logic [1:0]               arb_fmt,
    input  logic [3:0]               arb_credits,
    input  logic [31:0]              crc_in,
    input  logic                     crc_valid,
    input  logic [2:0]               negotiated_format,
    output logic [255:0]             flit_data,
    output logic                     flit_valid,
    input  logic                     flit_ready
);

    typedef enum logic [2:0] {
        FMT_RAW         = 3'b001,
        FMT_68B         = 3'b010,
        FMT_256B_END    = 3'b011,
        FMT_256B_START  = 3'b100
    } flit_format_t;

    logic [255:0] flit_buffer;
    logic        packing_done;

    always_comb begin
        flit_data = '0;
        flit_valid = 1'b0;
        arb_ready = 1'b1;

        if (arb_valid && pl_ready && crc_valid) begin
            case (negotiated_format)
                FMT_RAW: begin
                    // Format 1: Raw - 64B data only
                    flit_data[63:0] = arb_data[63:0];
                    flit_valid = 1'b1;
                end

                FMT_68B: begin
                    // Format 2: 68B - 64B data + 2B header + 2B CRC
                    flit_data[15:0]  = {4'b0, arb_fmt, 6'b0, arb_eop, arb_sop};
                    flit_data[79:16] = arb_data[63:0];
                    flit_data[95:80] = crc_in[15:0];
                    flit_valid = 1'b1;
                end

                FMT_256B_END: begin
                    // Format 3: 256B End-Header
                    flit_data[239:0] = arb_data[239:0];
                    flit_data[255:240] = {4'b0, arb_fmt, 6'b0, arb_eop, arb_sop};
                    flit_valid = 1'b1;
                end

                FMT_256B_START: begin
                    // Format 4: 256B Start-Header
                    flit_data[15:0]  = {4'b0, arb_fmt, 6'b0, arb_eop, arb_sop};
                    flit_data[255:16] = arb_data[239:0];
                    flit_valid = 1'b1;
                end

                default: begin
                    flit_valid = 1'b0;
                end
            endcase
        end
    end

endmodule