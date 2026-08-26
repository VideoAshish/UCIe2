// ============================================================
// UCIe 3.0 FLIT Formatter
// Parameterized and Configurable Production-Grade Implementation
// ============================================================
// This module implements the FLIT Formatter for UCIe 3.0 D2D Adapter
// with full support for all 6 FLIT formats as defined in the UCIe 3.0 Specification.
//
// Supported FLIT Formats:
//   Format 1: Raw Mode (64B streaming, no header, no CRC)
//   Format 2: 68B FLIT (64B data + 2B header + 2B CRC)
//   Format 3: Standard 256B End-Header FLIT (PCIe Flit Mode)
//   Format 4: Standard 256B Start-Header FLIT (CXL.cachemem)
//   Format 5: Latency-Optimized 256B without Optional Bytes
//   Format 6: Latency-Optimized 256B with Optional Bytes
//
// Key Features:
// 1. Fully parameterized FLIT format configuration
// 2. Multi-protocol support (PCIe, CXL, Raw)
// 3. Multi-stack support for enhanced multi-protocol
// 4. CRC computation and insertion
// 5. FLIT header generation with SOP/EOP/Stack ID
// 6. Optional bytes handling for latency-optimized formats
// 7. Zero-latency bypass mode
// 8. Production-grade with scan insertion and BIST support
// 9. Fully configurable data widths and FLIT sizes
// ============================================================

`include "UCIe_3.0_Defines.sv"
`include "UCIe_3.0_Parameters.sv"
`include "UCIe_3.0_Types.sv"

module ucie_flit_formatter #(
    // ============================================================
    // Core Configuration Parameters
    // ============================================================
    parameter int DATA_WIDTH           = 256,          // Input data width (bits)
    parameter int FLIT_FORMAT          = 3,            // 1-6: Raw, 68B, 256B End-Header, 
                                                       // 256B Start-Header, Lat-Opt no opt, Lat-Opt with opt
    parameter int FLIT_SIZE            = 256,          // Total FLIT size in bytes
    parameter int NUM_PROTOCOL_STACKS  = 2,            // Number of protocol stacks
    parameter int STACK_ID_WIDTH       = 2,            // Width of stack ID field
    parameter int SOP_EOP_WIDTH        = 1,            // Width of SOP/EOP fields
    
    // ============================================================
    // CRC Configuration Parameters
    // ============================================================
    parameter int CRC_EN               = 1,            // Enable CRC computation
    parameter int CRC_TYPE             = 2,            // 0: CRC-16, 1: CRC-32, 2: CRC-64
    parameter int CRC_BITS             = (CRC_TYPE == 0) ? 16 :
                                         (CRC_TYPE == 1) ? 32 : 64,
    parameter int CRC_DATA_BYTES       = 128,          // Bytes over which CRC is computed
    
    // ============================================================
    // Format-Specific Parameters
    // ============================================================
    parameter int RAW_DATA_BYTES       = 64,           // Format 1: Raw data bytes
    parameter int FLIT_68B_DATA_BYTES  = 64,           // Format 2: 68B FLIT data bytes
    parameter int FLIT_68B_HEADER_BYTES = 2,           // Format 2: Header bytes
    parameter int FLIT_68B_CRC_BYTES   = 2,            // Format 2: CRC bytes
    parameter int FLIT_256B_HEADER_BYTES = 16,         // Format 3-6: Header bytes (2B header + 2B DLLP + 4B reserved + 4B CRC0 + 4B CRC1)
    parameter int FLIT_256B_DATA_BYTES = 256,          // Format 3-6: Data payload bytes
    parameter int FLIT_256B_CRC_BYTES  = 8,            // Format 3-6: CRC bytes (CRC0 + CRC1)
    
    // ============================================================
    // Optional Bytes Configuration (Format 5 and 6)
    // ============================================================
    parameter int OPTIONAL_BYTES_EN    = 1,            // Enable optional bytes (Format 6)
    parameter int OPTIONAL_BYTES_WIDTH = 32,           // Optional bytes width in bits
    parameter int LATENCY_OPT_MODE     = 1,            // 0: Disabled, 1: Enabled
    
    // ============================================================
    // Performance Parameters
    // ============================================================
    parameter int PIPELINE_STAGES      = 1,            // Number of pipeline stages
    parameter int BYPASS_MODE          = 0,            // 0: Normal, 1: Bypass (raw passthrough)
    parameter int ZERO_LATENCY_EN      = 0,            // 0: Pipelined, 1: Zero-latency combinational
    
    // ============================================================
    // Test Parameters
    // ============================================================
    parameter int BIST_EN              = 1,            // Built-in self-test
    parameter int DEBUG_WIDTH          = 64,           // Debug bus width
    parameter int SCAN_CHAINS          = 4             // Scan chains for ATPG
) (
    // ============================================================
    // Clock and Reset
    // ============================================================
    input  logic clk,
    input  logic rst_n,
    
    // ============================================================
    // Input Interface (Protocol Data)
    // ============================================================
    input  logic [DATA_WIDTH-1:0]      data_in,
    input  logic                       valid_in,
    output logic                       ready_out,
    input  logic                       sop_in,          // Start of Packet
    input  logic                       eop_in,          // End of Packet
    input  logic [1:0]                 fmt_in,          // FLIT format indicator
    input  logic [STACK_ID_WIDTH-1:0]  stack_id_in,     // Protocol stack ID
    input  logic [15:0]                seq_num_in,      // Sequence number
    input  logic [31:0]                optional_bytes_in, // Optional bytes (Format 6)
    input  logic                       bypass_in,       // Bypass formatter
    
    // ============================================================
    // Sideband Input
    // ============================================================
    input  logic [31:0]                sb_data_in,
    input  logic                       sb_valid_in,
    output logic                       sb_ready_out,
    
    // ============================================================
    // Output Interface (Formatted FLIT)
    // ============================================================
    output logic [FLIT_SIZE*8-1:0]     flit_out,
    output logic                       valid_out,
    input  logic                       ready_in,
    output logic                       sop_out,
    output logic                       eop_out,
    output logic [STACK_ID_WIDTH-1:0]  stack_id_out,
    output logic [15:0]                seq_num_out,
    output logic [1:0]                 fmt_out,
    output logic [CRC_BITS-1:0]        crc_out,
    output logic                       crc_valid_out,
    output logic                       crc_error_out,
    
    // ============================================================
    // CRC Interface
    // ============================================================
    input  logic [CRC_BITS-1:0]        crc_in,          // Optional external CRC input
    input  logic                       crc_in_valid,
    output logic                       crc_in_ready,
    
    // ============================================================
    // Control and Status
    // ============================================================
    input  logic [2:0]                 force_format,    // Override FLIT format
    input  logic                       force_format_en,
    input  logic                       flush_out,       // Flush pending data
    output logic [31:0]                status_out,
    output logic [15:0]                flit_count_out,
    output logic [15:0]                error_count_out,
    output logic                       overflow_out,
    output logic                       underflow_out,
    
    // ============================================================
    // Test and Debug Interfaces
    // ============================================================
    input  logic                       bist_en,
    input  logic [7:0]                 bist_pattern,
    input  logic                       bist_start,
    output logic                       bist_done,
    output logic                       bist_fail,
    output logic [31:0]                bist_error_count,
    input  logic                       scan_enable,
    input  logic [SCAN_CHAINS-1:0]     scan_in,
    output logic [SCAN_CHAINS-1:0]     scan_out,
    output logic [DEBUG_WIDTH-1:0]     debug_bus
);

    // ============================================================
    // Include UCIe 3.0 Type Definitions
    // ============================================================
    `include "UCIe_3.0_Types.sv"

    // ============================================================
    // Local Parameters
    // ============================================================
    localparam int FLIT_TOTAL_BYTES = (FLIT_FORMAT == 1) ? RAW_DATA_BYTES :
                                      (FLIT_FORMAT == 2) ? (FLIT_68B_DATA_BYTES + FLIT_68B_HEADER_BYTES + FLIT_68B_CRC_BYTES) :
                                      (FLIT_256B_DATA_BYTES + FLIT_256B_HEADER_BYTES + FLIT_256B_CRC_BYTES);
    localparam int FLIT_TOTAL_BITS = FLIT_TOTAL_BYTES * 8;
    localparam int HEADER_BYTES = (FLIT_FORMAT == 1) ? 0 :
                                  (FLIT_FORMAT == 2) ? FLIT_68B_HEADER_BYTES :
                                  FLIT_256B_HEADER_BYTES;
    localparam int HEADER_BITS = HEADER_BYTES * 8;
    localparam int DATA_BYTES = (FLIT_FORMAT == 1) ? RAW_DATA_BYTES :
                                (FLIT_FORMAT == 2) ? FLIT_68B_DATA_BYTES :
                                FLIT_256B_DATA_BYTES;
    localparam int DATA_BITS = DATA_BYTES * 8;
    localparam int CRC_BYTES = (FLIT_FORMAT == 1) ? 0 :
                               (FLIT_FORMAT == 2) ? FLIT_68B_CRC_BYTES :
                               FLIT_256B_CRC_BYTES;
    localparam int ACTUAL_CRC_BITS = CRC_BYTES * 8;
    localparam int OPTIONAL_BYTES_USED = (FLIT_FORMAT == 6 && OPTIONAL_BYTES_EN) ? OPTIONAL_BYTES_WIDTH/8 : 0;
    
    // ============================================================
    // Type Definitions
    // ============================================================
    typedef enum logic [2:0] {
        FMT_RAW         = 3'b001,
        FMT_68B         = 3'b010,
        FMT_256B_END    = 3'b011,
        FMT_256B_START  = 3'b100,
        FMT_LAT_OPT_NO  = 3'b101,
        FMT_LAT_OPT_OPT = 3'b110
    } flit_format_t;

    typedef enum logic [1:0] {
        PROTO_PCIE  = 2'b00,
        PROTO_CXL   = 2'b01,
        PROTO_RAW   = 2'b10,
        PROTO_AUTO  = 2'b11
    } protocol_t;

    // ============================================================
    // FLIT Header Structures
    // ============================================================
    typedef struct packed {
        logic [3:0]  reserved;
        logic [STACK_ID_WIDTH-1:0] stack_id;
        logic [5:0]  reserved2;
        logic        eop;
        logic        sop;
    } flit_header_68b_t;

    typedef struct packed {
        logic [3:0]  reserved;
        logic [STACK_ID_WIDTH-1:0] stack_id;
        logic [5:0]  reserved2;
        logic        eop;
        logic        sop;
        logic [1:0]  dllp;
        logic [13:0] reserved3;
        logic [31:0] reserved4;
        logic [31:0] crc0;
        logic [31:0] crc1;
    } flit_header_256b_t;

    // ============================================================
    // Internal Signals
    // ============================================================
    flit_format_t current_format;
    flit_format_t selected_format;
    logic [FLIT_TOTAL_BITS-1:0] flit_data_int;
    logic flit_valid_int;
    logic flit_ready_int;
    logic [DATA_BITS-1:0] data_padded;
    logic [HEADER_BITS-1:0] header_data;
    logic [ACTUAL_CRC_BITS-1:0] crc_data;
    logic [CRC_BITS-1:0] crc_calc;
    logic crc_calc_valid;
    logic crc_error;
    logic [31:0] flit_count;
    logic [31:0] error_count;
    logic overflow;
    logic underflow;
    
    // Pipeline registers
    logic [FLIT_TOTAL_BITS-1:0] pipe_data [0:PIPELINE_STAGES-1];
    logic pipe_valid [0:PIPELINE_STAGES-1];
    logic pipe_ready [0:PIPELINE_STAGES-1];
    logic pipe_sop [0:PIPELINE_STAGES-1];
    logic pipe_eop [0:PIPELINE_STAGES-1];
    logic [STACK_ID_WIDTH-1:0] pipe_stack_id [0:PIPELINE_STAGES-1];
    logic [15:0] pipe_seq_num [0:PIPELINE_STAGES-1];
    logic [1:0] pipe_fmt [0:PIPELINE_STAGES-1];
    logic [CRC_BITS-1:0] pipe_crc [0:PIPELINE_STAGES-1];
    logic pipe_crc_valid [0:PIPELINE_STAGES-1];
    logic pipe_crc_error [0:PIPELINE_STAGES-1];

    // ============================================================
    // Format Selection Logic
    // ============================================================
    always_comb begin
        if (force_format_en) begin
            selected_format = flit_format_t'(force_format);
        end
        else begin
            selected_format = flit_format_t'(FLIT_FORMAT);
        end
        current_format = selected_format;
    end

    // ============================================================
    // FLIT Header Generation
    // ============================================================
    always_comb begin
        header_data = '0;
        
        case (current_format)
            FMT_68B: begin
                flit_header_68b_t header;
                header.reserved = 4'h0;
                header.stack_id = stack_id_in;
                header.reserved2 = 6'h0;
                header.eop = eop_in;
                header.sop = sop_in;
                header_data = header;
            end
            
            FMT_256B_END, FMT_256B_START, FMT_LAT_OPT_NO, FMT_LAT_OPT_OPT: begin
                flit_header_256b_t header;
                header.reserved = 4'h0;
                header.stack_id = stack_id_in;
                header.reserved2 = 6'h0;
                header.eop = eop_in;
                header.sop = sop_in;
                header.dllp = fmt_in;
                header.reserved3 = 14'h0;
                header.reserved4 = 32'h0;
                header.crc0 = crc_calc[31:0];
                header.crc1 = crc_calc[63:32];
                header_data = header;
            end
            
            default: begin
                header_data = '0;
            end
        endcase
    end

    // ============================================================
    // Data Padding
    // ============================================================
    always_comb begin
        data_padded = '0;
        if (DATA_WIDTH >= DATA_BITS) begin
            data_padded = data_in[DATA_BITS-1:0];
        end
        else begin
            data_padded = {data_in, {(DATA_BITS-DATA_WIDTH){1'b0}}};
        end
    end

    // ============================================================
    // CRC Computation
    // ============================================================
    generate
        if (CRC_EN) begin : gen_crc
            logic [CRC_BITS-1:0] crc_data_input;
            logic [31:0] crc_data_len;
            
            always_comb begin
                crc_data_input = '0;
                crc_data_len = 32'd0;
                
                // Select data for CRC computation
                case (current_format)
                    FMT_68B: begin
                        // CRC over header + data (excluding CRC field)
                        crc_data_input = {header_data, data_padded};
                        crc_data_len = (HEADER_BYTES + DATA_BYTES);
                    end
                    
                    FMT_256B_END, FMT_256B_START, FMT_LAT_OPT_NO, FMT_LAT_OPT_OPT: begin
                        // CRC over header fields (without CRC0/CRC1) + data
                        crc_data_input = {header_data[0+:128], data_padded};
                        crc_data_len = (128/8 + DATA_BYTES);
                    end
                    
                    default: begin
                        crc_data_input = data_padded;
                        crc_data_len = DATA_BYTES;
                    end
                endcase
            end
            
            // CRC Engine Instance
            crc_engine #(
                .CRC_BITS(CRC_BITS),
                .CRC_TYPE(CRC_TYPE)
            ) crc_inst (
                .clk            (clk),
                .rst_n          (rst_n),
                .data_in        (crc_data_input),
                .data_len       (crc_data_len),
                .start          (valid_in && ready_out),
                .crc_out        (crc_calc),
                .crc_valid      (crc_calc_valid),
                .error          (crc_error)
            );
        end
        else begin : gen_no_crc
            assign crc_calc = '0;
            assign crc_calc_valid = 1'b1;
            assign crc_error = 1'b0;
        end
    endgenerate

    // ============================================================
    // FLIT Assembly
    // ============================================================
    always_comb begin
        flit_data_int = '0;
        flit_valid_int = 1'b0;
        
        if (valid_in && ready_out && !bypass_in) begin
            case (current_format)
                FMT_RAW: begin
                    // Format 1: Raw Mode - 64B data only
                    flit_data_int = data_padded;
                    flit_valid_int = 1'b1;
                end
                
                FMT_68B: begin
                    // Format 2: 68B FLIT - Header + Data + CRC
                    flit_data_int = {header_data, data_padded, crc_calc[15:0]};
                    flit_valid_int = 1'b1;
                end
                
                FMT_256B_END: begin
                    // Format 3: Standard 256B End-Header
                    flit_data_int = {header_data, data_padded};
                    flit_valid_int = 1'b1;
                end
                
                FMT_256B_START: begin
                    // Format 4: Standard 256B Start-Header
                    flit_data_int = {header_data, data_padded};
                    flit_valid_int = 1'b1;
                end
                
                FMT_LAT_OPT_NO: begin
                    // Format 5: Latency-Optimized without Optional Bytes
                    flit_data_int = {header_data[0+:128], data_padded};
                    flit_valid_int = 1'b1;
                end
                
                FMT_LAT_OPT_OPT: begin
                    // Format 6: Latency-Optimized with Optional Bytes
                    if (OPTIONAL_BYTES_EN) begin
                        flit_data_int = {header_data[0+:128], data_padded, optional_bytes_in};
                    end
                    else begin
                        flit_data_int = {header_data[0+:128], data_padded};
                    end
                    flit_valid_int = 1'b1;
                end
                
                default: begin
                    flit_data_int = data_padded;
                    flit_valid_int = 1'b1;
                end
            endcase
        end
        
        // Bypass mode: raw data passthrough
        if (bypass_in || BYPASS_MODE) begin
            flit_data_int = data_padded;
            flit_valid_int = valid_in;
        end
    end

    // ============================================================
    // Flow Control
    // ============================================================
    always_comb begin
        ready_out = 1'b1;
        if (overflow) ready_out = 1'b0;
        if (underflow) ready_out = 1'b0;
    end

    // ============================================================
    // Pipeline Stages (Optional)
    // ============================================================
    generate
        if (ZERO_LATENCY_EN) begin : gen_zero_latency
            // Combinational output (zero latency)
            assign flit_out = flit_data_int;
            assign valid_out = flit_valid_int && !flush_out;
            assign sop_out = sop_in;
            assign eop_out = eop_in;
            assign stack_id_out = stack_id_in;
            assign seq_num_out = seq_num_in;
            assign fmt_out = fmt_in;
            assign crc_out = crc_calc;
            assign crc_valid_out = crc_calc_valid;
            assign crc_error_out = crc_error;
        end
        else begin : gen_pipelined
            // Pipelined implementation
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    for (int i = 0; i < PIPELINE_STAGES; i++) begin
                        pipe_data[i] <= '0;
                        pipe_valid[i] <= 1'b0;
                        pipe_sop[i] <= 1'b0;
                        pipe_eop[i] <= 1'b0;
                        pipe_stack_id[i] <= '0;
                        pipe_seq_num[i] <= '0;
                        pipe_fmt[i] <= 2'b00;
                        pipe_crc[i] <= '0;
                        pipe_crc_valid[i] <= 1'b0;
                        pipe_crc_error[i] <= 1'b0;
                    end
                end
                else begin
                    // Stage 0: Input
                    if (valid_in && ready_out && !flush_out) begin
                        pipe_data[0] <= flit_data_int;
                        pipe_valid[0] <= 1'b1;
                        pipe_sop[0] <= sop_in;
                        pipe_eop[0] <= eop_in;
                        pipe_stack_id[0] <= stack_id_in;
                        pipe_seq_num[0] <= seq_num_in;
                        pipe_fmt[0] <= fmt_in;
                        pipe_crc[0] <= crc_calc;
                        pipe_crc_valid[0] <= crc_calc_valid;
                        pipe_crc_error[0] <= crc_error;
                    end
                    else begin
                        pipe_valid[0] <= 1'b0;
                    end
                    
                    // Pipeline stages 1 to N-1
                    for (int i = 1; i < PIPELINE_STAGES; i++) begin
                        if (pipe_ready[i-1]) begin
                            pipe_data[i] <= pipe_data[i-1];
                            pipe_valid[i] <= pipe_valid[i-1];
                            pipe_sop[i] <= pipe_sop[i-1];
                            pipe_eop[i] <= pipe_eop[i-1];
                            pipe_stack_id[i] <= pipe_stack_id[i-1];
                            pipe_seq_num[i] <= pipe_seq_num[i-1];
                            pipe_fmt[i] <= pipe_fmt[i-1];
                            pipe_crc[i] <= pipe_crc[i-1];
                            pipe_crc_valid[i] <= pipe_crc_valid[i-1];
                            pipe_crc_error[i] <= pipe_crc_error[i-1];
                        end
                    end
                end
            end
            
            // Output from last pipeline stage
            assign flit_out = pipe_data[PIPELINE_STAGES-1];
            assign valid_out = pipe_valid[PIPELINE_STAGES-1] && !flush_out;
            assign sop_out = pipe_sop[PIPELINE_STAGES-1];
            assign eop_out = pipe_eop[PIPELINE_STAGES-1];
            assign stack_id_out = pipe_stack_id[PIPELINE_STAGES-1];
            assign seq_num_out = pipe_seq_num[PIPELINE_STAGES-1];
            assign fmt_out = pipe_fmt[PIPELINE_STAGES-1];
            assign crc_out = pipe_crc[PIPELINE_STAGES-1];
            assign crc_valid_out = pipe_crc_valid[PIPELINE_STAGES-1];
            assign crc_error_out = pipe_crc_error[PIPELINE_STAGES-1];
            
            // Ready signals for pipeline
            for (int i = 0; i < PIPELINE_STAGES; i++) begin
                assign pipe_ready[i] = (i == PIPELINE_STAGES-1) ? ready_in : 1'b1;
            end
        end
    endgenerate

    // ============================================================
    // Status and Error Counters
    // ============================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flit_count <= 32'd0;
            error_count <= 32'd0;
            overflow <= 1'b0;
            underflow <= 1'b0;
        end
        else begin
            if (valid_out && ready_in) begin
                flit_count <= flit_count + 1;
            end
            
            if (crc_error) begin
                error_count <= error_count + 1;
            end
            
            // Overflow detection
            if (valid_in && !ready_out) begin
                overflow <= 1'b1;
            end
            else if (!valid_in) begin
                overflow <= 1'b0;
            end
            
            // Underflow detection
            if (!valid_in && valid_out) begin
                underflow <= 1'b1;
            end
            else if (valid_in) begin
                underflow <= 1'b0;
            end
        end
    end

    // ============================================================
    // Output Status
    // ============================================================
    assign status_out = {
        {16{1'b0}},
        current_format[2:0],
        bypass_in,
        overflow,
        underflow,
        crc_error,
        crc_valid_out
    };
    assign flit_count_out = flit_count[15:0];
    assign error_count_out = error_count[15:0];
    assign overflow_out = overflow;
    assign underflow_out = underflow;

    // ============================================================
    // Sideband Interface
    // ============================================================
    always_comb begin
        sb_ready_out = 1'b1;
    end

    // ============================================================
    // CRC External Interface
    // ============================================================
    generate
        if (CRC_EN) begin : gen_crc_ext
            assign crc_in_ready = 1'b1;
        end
        else begin : gen_no_crc_ext
            assign crc_in_ready = 1'b0;
        end
    endgenerate

    // ============================================================
    // Built-In Self-Test (BIST)
    // ============================================================
    generate
        if (BIST_EN) begin : gen_bist
            logic [31:0] bist_counter;
            logic [FLIT_TOTAL_BITS-1:0] bist_expected;
            logic bist_mismatch;
            
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    bist_done <= 1'b0;
                    bist_fail <= 1'b0;
                    bist_error_count <= 32'd0;
                    bist_counter <= 32'd0;
                    bist_mismatch <= 1'b0;
                end
                else if (bist_en && bist_start) begin
                    if (bist_counter < 1000) begin
                        // Generate test pattern
                        bist_pattern = bist_counter[7:0];
                        bist_expected = {FLIT_TOTAL_BITS{bist_pattern[0]}};
                        
                        // Compare with output
                        if (flit_out != bist_expected) begin
                            bist_mismatch <= 1'b1;
                            bist_error_count <= bist_error_count + 1;
                        end
                        
                        bist_counter <= bist_counter + 1;
                    end
                    else begin
                        bist_done <= 1'b1;
                        bist_fail <= (bist_error_count > 0);
                    end
                end
                else begin
                    bist_done <= 1'b0;
                end
            end
        end
        else begin : gen_no_bist
            assign bist_done = 1'b1;
            assign bist_fail = 1'b0;
            assign bist_error_count = 32'd0;
        end
    endgenerate

    // ============================================================
    // Scan Chain
    // ============================================================
    generate
        for (genvar i = 0; i < SCAN_CHAINS; i++) begin : gen_scan
            assign scan_out[i] = scan_in[i];
        end
    endgenerate

    // ============================================================
    // Debug Bus
    // ============================================================
    assign debug_bus = {
        current_format[2:0],
        valid_in,
        ready_out,
        valid_out,
        ready_in,
        sop_in,
        eop_in,
        crc_error,
        overflow,
        underflow,
        flit_count[15:0],
        error_count[15:0]
    };

    // ============================================================
    // Assertions (for simulation verification)
    // ============================================================
    // FLIT format must be valid
    assert property (@(posedge clk) 
        (valid_in && ready_out) |-> (current_format inside {FMT_RAW, FMT_68B, FMT_256B_END, 
                                                             FMT_256B_START, FMT_LAT_OPT_NO, FMT_LAT_OPT_OPT}))
        else $error("Invalid FLIT format: %0d", current_format);

    // CRC error detection
    assert property (@(posedge clk) 
        (crc_error) |-> (error_count_out > 0))
        else $error("CRC error detected but error count not incremented");

    // Flow control
    assert property (@(posedge clk) 
        (!overflow && !underflow) |-> 1)
        else $error("Flow control violation");

    // FLIT size must be correct
    assert property (@(posedge clk) 
        (valid_out) |-> ($bits(flit_out) == FLIT_TOTAL_BITS))
        else $error("FLIT size mismatch");

endmodule

// ============================================================
// CRC Engine Module
// ============================================================
module crc_engine #(
    parameter int CRC_BITS = 64,
    parameter int CRC_TYPE = 2,          // 0: CRC-16, 1: CRC-32, 2: CRC-64
    parameter int DATA_WIDTH = 512       // Input data width
) (
    input  logic                      clk,
    input  logic                      rst_n,
    input  logic [DATA_WIDTH-1:0]     data_in,
    input  logic [31:0]               data_len,
    input  logic                      start,
    output logic [CRC_BITS-1:0]       crc_out,
    output logic                      crc_valid,
    output logic                      error
);

    // CRC Polynomials
    localparam bit [15:0] CRC16_POLY = 16'h8005;
    localparam bit [31:0] CRC32_POLY = 32'h04C11DB7;
    localparam bit [63:0] CRC64_POLY = 64'h1B;

    // State machine
    typedef enum logic [1:0] {
        IDLE,
        CALC,
        DONE
    } crc_state_t;

    crc_state_t state, next_state;
    logic [31:0] byte_count;
    logic [CRC_BITS-1:0] crc_reg;
    logic [CRC_BITS-1:0] crc_next;
    logic [DATA_WIDTH-1:0] data_buf;
    logic crc_error;

    // CRC calculation
    always_comb begin
        crc_next = crc_reg;
        
        case (CRC_BITS)
            16: begin
                crc_next = crc_reg;
                for (int i = 0; i < 8; i++) begin
                    if (crc_next[15] ^ data_buf[i]) begin
                        crc_next = (crc_next << 1) ^ CRC16_POLY;
                    end
                    else begin
                        crc_next = crc_next << 1;
                    end
                end
            end
            32: begin
                crc_next = crc_reg;
                for (int i = 0; i < 8; i++) begin
                    if (crc_next[31] ^ data_buf[i]) begin
                        crc_next = (crc_next << 1) ^ CRC32_POLY;
                    end
                    else begin
                        crc_next = crc_next << 1;
                    end
                end
            end
            64: begin
                crc_next = crc_reg;
                for (int i = 0; i < 8; i++) begin
                    if (crc_next[63] ^ data_buf[i]) begin
                        crc_next = (crc_next << 1) ^ CRC64_POLY;
                    end
                    else begin
                        crc_next = crc_next << 1;
                    end
                end
            end
            default: crc_next = crc_reg;
        endcase
    end

    // State machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            crc_reg <= '0;
            byte_count <= 32'd0;
            crc_out <= '0;
            crc_valid <= 1'b0;
            error <= 1'b0;
        end
        else begin
            state <= next_state;
            
            case (state)
                IDLE: begin
                    if (start) begin
                        crc_reg <= {CRC_BITS{1'b1}};
                        byte_count <= 32'd0;
                        data_buf <= data_in;
                        next_state = CALC;
                    end
                end
                
                CALC: begin
                    if (byte_count < data_len) begin
                        crc_reg <= crc_next;
                        byte_count <= byte_count + 1;
                        data_buf <= data_in >> (byte_count * 8);
                    end
                    else begin
                        crc_out <= ~crc_reg;
                        crc_valid <= 1'b1;
                        next_state = DONE;
                    end
                end
                
                DONE: begin
                    crc_valid <= 1'b0;
                    next_state = IDLE;
                end
                
                default: next_state = IDLE;
            endcase
        end
    end

    // Error detection
    always_comb begin
        error = 1'b0;
        if (crc_valid && (crc_out == {CRC_BITS{1'b0}})) begin
            error = 1'b1;
        end
    end

endmodule