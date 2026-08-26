// ============================================================
// TileLink TL-C Node - SystemVerilog Implementation
// ============================================================
// Reference: TileLink 1.8.0 Specification
// https://github.com/chipsalliance/tilelink
//
// TL-C implements cache coherence with 5 channels:
//   A - Request (from client to manager)
//   B - Probe (from manager to client)
//   C - Release (from client to manager)
//   D - Response (from manager to client)
//   E - Grant Acknowledgement (from client to manager)
// ============================================================

`include "tl_defines.sv"

module tilelink_tlc_node #(
    // ============================================================
    // Core Configuration Parameters
    // ============================================================
    parameter int TLAddressWidth = 32,
    parameter int TLDataWidth   = 64,
    parameter int TLIDWidth     = 8,
    parameter int TLSourceWidth = 8,
    parameter int TLSinkWidth   = 8,

    // ============================================================
    // Cache Coherence Parameters
    // ============================================================
    parameter int NUM_CLIENTS   = 4,      // Number of caching clients
    parameter int CACHE_LINE_SIZE = 64,   // Bytes per cache line
    parameter int NUM_SETS      = 64,     // Number of cache sets
    parameter int NUM_WAYS      = 4       // Number of ways per set
) (
    // ============================================================
    // Clock and Reset
    // ============================================================
    input  logic clk,
    input  logic rst_n,

    // ============================================================
    // TL-C Client Interface (Initiator)
    // ============================================================
    // Channel A: Request from client to manager
    output logic [TLIDWidth-1:0]       a_id,
    output logic [2:0]                 a_opcode,
    output logic [2:0]                 a_param,
    output logic [2:0]                 a_size,
    output logic [TLSourceWidth-1:0]   a_source,
    output logic [TLAddressWidth-1:0]  a_address,
    output logic [TLDataWidth/8-1:0]   a_mask,
    output logic [TLDataWidth-1:0]     a_data,
    output logic                       a_corrupt,
    output logic                       a_valid,
    input  logic                       a_ready,

    // Channel D: Response from manager to client
    input  logic [TLIDWidth-1:0]       d_id,
    input  logic [2:0]                 d_opcode,
    input  logic [2:0]                 d_param,
    input  logic [2:0]                 d_size,
    input  logic [TLSourceWidth-1:0]   d_source,
    input  logic [TLSinkWidth-1:0]     d_sink,
    input  logic                       d_denied,
    input  logic [TLDataWidth-1:0]     d_data,
    input  logic                       d_corrupt,
    input  logic                       d_valid,
    output logic                       d_ready,

    // Channel E: Grant acknowledgement from client to manager
    output logic [TLSinkWidth-1:0]     e_sink,
    output logic                       e_valid,
    input  logic                       e_ready,

    // ============================================================
    // TL-C Manager Interface (Target)
    // ============================================================
    // Channel B: Probe from manager to client
    input  logic [TLIDWidth-1:0]       b_id,
    input  logic [2:0]                 b_opcode,
    input  logic [2:0]                 b_param,
    input  logic [2:0]                 b_size,
    input  logic [TLSourceWidth-1:0]   b_source,
    input  logic [TLAddressWidth-1:0]  b_address,
    input  logic [TLDataWidth/8-1:0]   b_mask,
    input  logic [TLDataWidth-1:0]     b_data,
    input  logic                       b_corrupt,
    input  logic                       b_valid,
    output logic                       b_ready,

    // Channel C: Release from client to manager
    output logic [TLIDWidth-1:0]       c_id,
    output logic [2:0]                 c_opcode,
    output logic [2:0]                 c_param,
    output logic [2:0]                 c_size,
    output logic [TLSourceWidth-1:0]   c_source,
    output logic [TLAddressWidth-1:0]  c_address,
    output logic [TLDataWidth-1:0]     c_data,
    output logic                       c_corrupt,
    output logic                       c_valid,
    input  logic                       c_ready,

    // ============================================================
    // Status and Debug
    // ============================================================
    output logic [31:0]               status,
    output logic [31:0]               error_count,
    output logic [127:0]              debug_bus
);

    // ============================================================
    // Include TileLink Package
    // ============================================================
    `include "tl_package.sv"

    // ============================================================
    // Type Definitions
    // ============================================================
    // A-Channel Opcodes [8†L17-L18]
    typedef enum logic [2:0] {
        A_PUT_FULL_DATA    = 3'd0,
        A_PUT_PARTIAL_DATA = 3'd1,
        A_ARITHMETIC_DATA  = 3'd2,
        A_LOGICAL_DATA     = 3'd3,
        A_GET              = 3'd4,
        A_GET_BLOCK        = 3'd5,
        A_GET_PREFETCH     = 3'd6,
        A_ATOMIC_OP        = 3'd7
    } a_opcode_t;

    // B-Channel Opcodes (Probe)
    typedef enum logic [2:0] {
        B_PROBE_BLOCK      = 3'd4,
        B_PROBE_PREFETCH   = 3'd5
    } b_opcode_t;

    // C-Channel Opcodes (Release)
    typedef enum logic [2:0] {
        C_RELEASE_DATA     = 3'd4,
        C_RELEASE_ACK      = 3'd5,
        C_PROBE_ACK        = 3'd6,
        C_PROBE_ACK_DATA   = 3'd7
    } c_opcode_t;

    // D-Channel Opcodes (Response)
    typedef enum logic [2:0] {
        D_ACCESS_ACK       = 3'd0,
        D_ACCESS_ACK_DATA  = 3'd1,
        D_HINT_ACK         = 3'd2,
        D_GRANT            = 3'd4,
        D_GRANT_DATA       = 3'd5,
        D_PROBE_ACK        = 3'd6,
        D_PROBE_ACK_DATA   = 3'd7
    } d_opcode_t;

    // E-Channel: Grant acknowledgement (no opcode, just sink)

    // ============================================================
    // Internal Signals
    // ============================================================
    // Coherence state for each cache line
    typedef enum logic [1:0] {
        COH_INVALID  = 2'b00,   // I - Invalid
        COH_SHARED   = 2'b01,   // S - Shared (clean)
        COH_EXCLUSIVE = 2'b10,  // E - Exclusive (clean)
        COH_MODIFIED = 2'b11    // M - Modified (dirty)
    } coh_state_t;

    // Cache tag array
    logic [TLAddressWidth-1:0] tag_array [NUM_SETS-1:0][NUM_WAYS-1:0];
    coh_state_t                coh_state [NUM_SETS-1:0][NUM_WAYS-1:0];
    logic [TLDataWidth-1:0]    data_array [NUM_SETS-1:0][NUM_WAYS-1:0];
    logic [TLDataWidth/8-1:0]  dirty_mask [NUM_SETS-1:0][NUM_WAYS-1:0];

    // Channel state machines
    typedef enum logic [1:0] {
        CH_IDLE,
        CH_WAIT,
        CH_DONE
    } ch_state_t;

    ch_state_t a_state, b_state, c_state, d_state, e_state;

    // ============================================================
    // Channel A: Request Processing (Client → Manager)
    // ============================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_state <= CH_IDLE;
            a_valid <= 1'b0;
            a_id <= '0;
            a_opcode <= '0;
            a_param <= '0;
            a_size <= '0;
            a_source <= '0;
            a_address <= '0;
            a_mask <= '0;
            a_data <= '0;
            a_corrupt <= 1'b0;
        end
        else begin
            case (a_state)
                CH_IDLE: begin
                    if (req_pending) begin
                        a_valid <= 1'b1;
                        a_opcode <= next_req_opcode;
                        a_address <= next_req_address;
                        a_data <= next_req_data;
                        a_state <= CH_WAIT;
                    end
                end

                CH_WAIT: begin
                    if (a_ready) begin
                        a_valid <= 1'b0;
                        a_state <= CH_DONE;
                    end
                end

                CH_DONE: begin
                    a_state <= CH_IDLE;
                end

                default: a_state <= CH_IDLE;
            endcase
        end
    end

    // ============================================================
    // Channel D: Response Processing (Manager → Client)
    // ============================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_state <= CH_IDLE;
            d_ready <= 1'b0;
        end
        else begin
            case (d_state)
                CH_IDLE: begin
                    if (d_valid) begin
                        d_ready <= 1'b1;
                        // Process response
                        case (d_opcode)
                            D_GRANT, D_GRANT_DATA: begin
                                // Update cache state based on grant
                                update_cache_on_grant(d_address, d_data);
                            end
                            D_ACCESS_ACK_DATA: begin
                                // Data response for Get/GetBlock
                                update_cache_on_response(d_address, d_data);
                            end
                            default: begin
                                // Other responses
                            end
                        endcase
                        d_state <= CH_WAIT;
                    end
                end

                CH_WAIT: begin
                    if (!d_valid) begin
                        d_ready <= 1'b0;
                        d_state <= CH_IDLE;
                    end
                end

                default: d_state <= CH_IDLE;
            endcase
        end
    end

    // ============================================================
    // Channel B: Probe Processing (Manager → Client)
    // ============================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b_state <= CH_IDLE;
            b_ready <= 1'b0;
        end
        else begin
            case (b_state)
                CH_IDLE: begin
                    if (b_valid) begin
                        b_ready <= 1'b1;
                        // Handle probe based on opcode
                        case (b_opcode)
                            B_PROBE_BLOCK: begin
                                // Check if address is in cache
                                probe_cache(b_address);
                            end
                            default: begin
                                // Other probes
                            end
                        endcase
                        b_state <= CH_WAIT;
                    end
                end

                CH_WAIT: begin
                    if (!b_valid) begin
                        b_ready <= 1'b0;
                        b_state <= CH_IDLE;
                    end
                end

                default: b_state <= CH_IDLE;
            endcase
        end
    end

    // ============================================================
    // Channel C: Release Processing (Client → Manager)
    // ============================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_state <= CH_IDLE;
            c_valid <= 1'b0;
            c_id <= '0;
            c_opcode <= '0;
            c_param <= '0;
            c_size <= '0;
            c_source <= '0;
            c_address <= '0;
            c_data <= '0;
            c_corrupt <= 1'b0;
        end
        else begin
            case (c_state)
                CH_IDLE: begin
                    if (release_pending) begin
                        c_valid <= 1'b1;
                        c_opcode <= C_RELEASE_DATA;
                        c_address <= release_address;
                        c_data <= release_data;
                        c_state <= CH_WAIT;
                    end
                end

                CH_WAIT: begin
                    if (c_ready) begin
                        c_valid <= 1'b0;
                        c_state <= CH_DONE;
                    end
                end

                CH_DONE: begin
                    c_state <= CH_IDLE;
                end

                default: c_state <= CH_IDLE;
            endcase
        end
    end

    // ============================================================
    // Channel E: Grant Acknowledgement (Client → Manager)
    // ============================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            e_state <= CH_IDLE;
            e_valid <= 1'b0;
            e_sink <= '0;
        end
        else begin
            case (e_state)
                CH_IDLE: begin
                    if (grant_ack_pending) begin
                        e_valid <= 1'b1;
                        e_sink <= grant_sink;
                        e_state <= CH_WAIT;
                    end
                end

                CH_WAIT: begin
                    if (e_ready) begin
                        e_valid <= 1'b0;
                        e_state <= CH_DONE;
                    end
                end

                CH_DONE: begin
                    e_state <= CH_IDLE;
                end

                default: e_state <= CH_IDLE;
            endcase
        end
    end

    // ============================================================
    // Cache Coherence Helper Functions
    // ============================================================

    // Probe cache for a given address
    function automatic void probe_cache(input logic [TLAddressWidth-1:0] addr);
        logic hit;
        int set_idx, way_idx;

        set_idx = get_set_index(addr);
        hit = 1'b0;

        for (int w = 0; w < NUM_WAYS; w++) begin
            if (tag_array[set_idx][w] == get_tag(addr)) begin
                hit = 1'b1;
                way_idx = w;
                break;
            end
        end

        if (hit) begin
            // Send probe response on Channel C
            c_pending <= 1'b1;
            c_address <= addr;
            c_data <= data_array[set_idx][way_idx];
            c_opcode <= C_PROBE_ACK_DATA;

            // Update coherence state based on probe
            case (b_opcode)
                B_PROBE_BLOCK: begin
                    // Invalidate or downgrade based on probe type
                    case (coh_state[set_idx][way_idx])
                        COH_MODIFIED: begin
                            // Write back dirty data before invalidation
                            c_data <= data_array[set_idx][way_idx];
                            coh_state[set_idx][way_idx] <= COH_INVALID;
                        end
                        COH_EXCLUSIVE: begin
                            coh_state[set_idx][way_idx] <= COH_INVALID;
                        end
                        COH_SHARED: begin
                            coh_state[set_idx][way_idx] <= COH_INVALID;
                        end
                        default: begin
                            // Already invalid
                        end
                    endcase
                end
                default: begin
                    // Other probe types
                end
            endcase
        end
    endfunction

    // Update cache on grant
    function automatic void update_cache_on_grant(
        input logic [TLAddressWidth-1:0] addr,
        input logic [TLDataWidth-1:0] data
    );
        int set_idx, way_idx;

        set_idx = get_set_index(addr);
        way_idx = select_way(set_idx);

        tag_array[set_idx][way_idx] = get_tag(addr);
        data_array[set_idx][way_idx] = data;
        coh_state[set_idx][way_idx] = COH_EXCLUSIVE;
        dirty_mask[set_idx][way_idx] = '0;
    endfunction

    // Update cache on response
    function automatic void update_cache_on_response(
        input logic [TLAddressWidth-1:0] addr,
        input logic [TLDataWidth-1:0] data
    );
        int set_idx, way_idx;
        logic hit;

        set_idx = get_set_index(addr);
        hit = 1'b0;

        for (int w = 0; w < NUM_WAYS; w++) begin
            if (tag_array[set_idx][w] == get_tag(addr)) begin
                hit = 1'b1;
                way_idx = w;
                break;
            end
        end

        if (hit) begin
            data_array[set_idx][way_idx] <= data;
            coh_state[set_idx][way_idx] <= COH_SHARED;
        end
    endfunction

    // Helper: Get set index from address
    function automatic int get_set_index(input logic [TLAddressWidth-1:0] addr);
        return addr[$clog2(CACHE_LINE_SIZE) +: $clog2(NUM_SETS)];
    endfunction

    // Helper: Get tag from address
    function automatic logic [TLAddressWidth-1:0] get_tag(
        input logic [TLAddressWidth-1:0] addr
    );
        return addr[$clog2(CACHE_LINE_SIZE) + $clog2(NUM_SETS) +: 
                    TLAddressWidth - $clog2(CACHE_LINE_SIZE) - $clog2(NUM_SETS)];
    endfunction

    // Helper: Select a way for allocation (pseudo-LRU)
    function automatic int select_way(input int set_idx);
        // Simple round-robin replacement
        static int rr_ptr [NUM_SETS-1:0];
        int way;

        way = rr_ptr[set_idx];
        rr_ptr[set_idx] = (rr_ptr[set_idx] + 1) % NUM_WAYS;
        return way;
    endfunction

    // ============================================================
    // Request Generation Logic
    // ============================================================
    logic req_pending;
    logic [2:0] next_req_opcode;
    logic [TLAddressWidth-1:0] next_req_address;
    logic [TLDataWidth-1:0] next_req_data;

    // ============================================================
    // Status and Debug Outputs
    // ============================================================
    assign status = {
        a_valid,
        d_valid,
        b_valid,
        c_valid,
        e_valid,
        27'b0
    };

    assign debug_bus = {
        a_state,
        d_state,
        b_state,
        c_state,
        e_state,
        a_opcode,
        d_opcode,
        b_opcode,
        c_opcode,
        112'b0
    };

endmodule