// ============================================================
// TL-UH Access Decoder and Burst Generator
// Reference: OpenTitan tluh_mem_bridge_decoder.sv[reference:39]
// ============================================================

module tluh_mem_bridge_decoder #(
    parameter int BusAddressWidth = 32,
    parameter int DataWidth       = 64,
    parameter int SizeWidth       = 3,
    localparam int MemSubwordAddressWidth = $clog2(DataWidth / 8),
    localparam int MemAddressWidth = BusAddressWidth - MemSubwordAddressWidth,
    localparam int BurstCounterWidth = ((2 ** SizeWidth) - 1) - MemSubwordAddressWidth
) (
    input  logic clk,
    input  logic arst,

    input  logic [2:0]                 r_in_opcode,
    input  logic [BusAddressWidth-1:0] r_in_address,
    input  logic [SizeWidth-1:0]       r_in_size,
    input  logic                       n_in_valid,
    output logic                       n_in_ready,

    output logic [MemAddressWidth-1:0] n_out_address,
    output logic                       n_out_read,
    output logic                       n_out_write,
    output logic                       n_out_rmw,
    output logic                       n_out_hint,
    output logic                       n_out_last,
    input  logic                       n_out_ready
);

    // Burst counter[reference:40]
    logic [BurstCounterWidth-1:0] r_burst_counter;
    logic [BurstCounterWidth-1:0] n_burst_counter;
    logic [BurstCounterWidth-1:0] c_burst_term;

    // Opcode decoding[reference:41]
    wire c_read  = n_out_ready && n_in_valid && (r_in_opcode == AGet);
    wire c_write = n_out_ready && n_in_valid && 
                   ((r_in_opcode == APutFullData) || (r_in_opcode == APutPartialData));
    wire c_rmw   = n_out_ready && n_in_valid && 
                   ((r_in_opcode == AArithmeticData) || (r_in_opcode == ALogicalData));
    wire c_hint  = n_in_valid && (r_in_opcode == AIntent);
    wire c_burstablein = c_rmw || c_write;
    wire c_skipburst   = c_hint;

    // Burst termination calculation[reference:42]
    assign c_burst_term = 
        (BurstCounterWidth)'(({((2**SizeWidth)-1){1'd1}} >> (~r_in_size)) >> 
                             MemSubwordAddressWidth);

    wire c_count_enable = n_in_valid & n_out_ready;
    wire c_last = c_skipburst || (c_burst_term == r_burst_counter);

    // Next burst counter[reference:43]
    assign n_burst_counter = c_last ? 
        (BurstCounterWidth)'(0) : 
        (r_burst_counter + (BurstCounterWidth)'(1));

    // Ready and address generation[reference:44]
    assign n_in_ready = (c_last | c_burstablein) & n_out_ready;
    assign n_out_address = (MemAddressWidth'(r_in_address >> MemSubwordAddressWidth)) | 
                           (MemAddressWidth'(r_burst_counter));
    assign n_out_read  = c_read;
    assign n_out_write = c_write;
    assign n_out_rmw   = c_rmw;
    assign n_out_hint  = c_hint;
    assign n_out_last  = c_last;

    // Burst counter register
    always_ff @(posedge clk or posedge arst) begin
        if (arst) begin
            r_burst_counter <= '0;
        end else if (c_count_enable) begin
            r_burst_counter <= n_burst_counter;
        end
    end

endmodule