// ============================================================
// UCIe Mainband CRC-32 Generator
// ============================================================
// Polynomial: 0x04C11DB7 (IEEE 802.3)
// Initial value: 0xFFFFFFFF
// ============================================================

module ucie_mainband_crc #(
    parameter int DATA_WIDTH = 256
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic [DATA_WIDTH-1:0]    data_in,
    input  logic                     data_valid,
    output logic                     data_ready,
    output logic [31:0]              crc_out,
    output logic                     crc_valid,
    output logic                     crc_error
);

    logic [31:0] crc_reg;
    logic [31:0] crc_next;
    logic [7:0]  byte_idx;

    // CRC-32 polynomial: x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1
    function automatic [31:0] crc32_byte(input [31:0] crc, input [7:0] data);
        logic [31:0] new_crc;
        new_crc = crc ^ {24'h0, data};
        for (int i = 0; i < 8; i++) begin
            new_crc = (new_crc[31]) ? (new_crc << 1) ^ 32'h04C11DB7 : (new_crc << 1);
        end
        return new_crc;
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc_reg <= 32'hFFFFFFFF;
            crc_out <= 32'hFFFFFFFF;
            crc_valid <= 1'b0;
            crc_error <= 1'b0;
            byte_idx <= 8'd0;
            data_ready <= 1'b1;
        end
        else if (data_valid && data_ready) begin
            // Process DATA_WIDTH bits as bytes
            for (int i = 0; i < DATA_WIDTH/8; i++) begin
                crc_reg <= crc32_byte(crc_reg, data_in[i*8 +: 8]);
            end
            crc_out <= ~crc_reg;  // Final XOR
            crc_valid <= 1'b1;
            crc_error <= 1'b0;
            data_ready <= 1'b0;
        end
        else begin
            crc_valid <= 1'b0;
            data_ready <= 1'b1;
        end
    end

endmodule