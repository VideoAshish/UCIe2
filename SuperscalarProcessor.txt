//=============================================================================
// gpgpu_streaming_multiprocessor.sv
// Top-Level Compute SM
//=============================================================================
module gpgpu_streaming_multiprocessor (
    input  logic                                  clk,
    input  logic                                  rst_n,

    // Instruction Cache (Fetch)
    output logic [gpgpu_pkg::PC_W-1:0]            icache_addr,
    input  logic [gpgpu_pkg::PC_W+15:0]           icache_rdata,
    input  logic                                  icache_valid,

    // Global Memory Interface (Async, simulates DRAM latency)
    output gpgpu_pkg::mem_req_t                   gmem_req,
    input  gpgpu_pkg::mem_rsp_t                   gmem_rsp,

    output logic                                  sm_idle
);
    import gpgpu_pkg::*;

    //=========================================================================
    // State & Per-Warp Context
    //=========================================================================
    logic [NUM_WARPS-1:0]             warp_active;
    logic [NUM_WARPS-1:0]             warp_stall;  // Set by LDGS, cleared by gmem_rsp
    logic [PC_W-1:0]                  pc [NUM_WARPS];
    mask_t                            active_mask [NUM_WARPS];
    
    // Scheduler
    logic [$clog2(NUM_WARPS)-1:0]     scheduled_warp;
    logic                             schedule_valid;
    
    // Pipeline Registers
    gpgpu_instr_t                     current_instr;
    logic                             is_executing;
    
    // Register File Wires
    logic [THREADS_PER_WARP-1:0][DATA_W-1:0] rs1, rs2, rs3, wb_data;
    logic                                     rf_we;
    logic [4:0]                                rf_rd;
    
    // ALU/Shared Mem Wires
    logic [THREADS_PER_WARP-1:0][DATA_W-1:0] alu_out, lds_out;
    logic                                     smem_req, smem_wen;
    logic [THREADS_PER_WARP-1:0][DATA_W-1:0] smem_wdata;
    logic [THREADS_PER_WARP-1:0][DATA_W-1:0] smem_rdata;

    //=========================================================================
    // Sub-module Instantiation
    //=========================================================================
    warp_scheduler u_sched (
        .clk(clk), .rst_n(rst_n),
        .warp_active(warp_active),
        .warp_stall(warp_stall),
        .scheduled_warp(scheduled_warp),
        .schedule_valid(schedule_valid),
        .all_warps_done(sm_idle)
    );

    // Massive Multi-ported Register File (Banked in real HW)
    gpu_register_file #(
        .NUM_WARPS(NUM_WARPS), .THREADS_PER_WARP(THREADS_PER_WARP),
        .REGS_PER_THREAD(REGS_PER_THREAD), .DATA_W(DATA_W)
    ) u_rf (
        .clk(clk), .rst_n(rst_n),
        .r1_warp_id(scheduled_warp), .r1_addr(current_instr.rs1), .r1_data(rs1),
        .r2_warp_id(scheduled_warp), .r2_addr(current_instr.rs2), .r2_data(rs2),
        .r3_warp_id(scheduled_warp), .r3_addr(current_instr.rs3), .r3_data(rs3),
        .w_warp_id(scheduled_warp), .w_addr(rf_rd), .w_data(wb_data), .w_enable(rf_we)
    );

    gpgpu_execution_unit u_exe (
        .clk(clk), .rst_n(rst_n),
        .enable(is_executing),
        .opcode(current_instr.opcode),
        .exec_mask(active_mask[scheduled_warp]),
        .vec_rs1(rs1), .vec_rs2(rs2), .vec_rs3(rs3),
        .alu_result(alu_out),
        .smem_addr_base(current_instr.smem_offset),
        .smem_req_valid(smem_req), .smem_wr_en(smem_wen),
        .smem_wr_data(smem_wdata), .smem_rd_data(smem_rdata),
        .lds_result(lds_out)
    );

    // On-chip Shared Memory (Scratchpad)
    shared_memory #(
        .ADDR_W(SHMEM_ADDR_W), .DATA_W(DATA_W * THREADS_PER_WARP)
    ) u_smem (
        .clk(clk),
        .addr(current_instr.smem_offset), // Simplified addressing
        .wr_en(smem_wen),
        .wr_data(smem_wdata),
        .rd_data(smem_rdata)
    );

    //=========================================================================
    // Main Execution Pipeline FSM
    //=========================================================================
    typedef enum logic [2:0] { FETCH, DECODE, EXECUTE, MEM_WAIT } state_t;
    state_t state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= FETCH; pc <= '{default: '0}; 
            warp_active <= '{default: '0}; warp_active[0] <= 1'b1; // Launch 1 CTA
            active_mask <= '{default: '1}; 
            warp_stall <= '0;
            is_executing <= 1'b0; rf_we <= 1'b0;
            gmem_req.valid <= 1'b0;
        end else begin
            rf_we <= 1'b0;
            gmem_req.valid <= 1'b0;
            is_executing <= 1'b0;

            // Handle Async Global Memory Response (Unblocks stalled warp)
            if (gmem_rsp.valid) begin
                warp_stall[gmem_rsp.warp_id] <= 1'b0;
                // In real HW, data is routed back to RF via a writeback network
            end

            case (state)
                FETCH: begin
                    if (schedule_valid) begin
                        icache_addr <= pc[scheduled_warp];
                        state <= DECODE;
                    end
                end

                DECODE: begin
                    if (icache_valid) begin
                        current_instr <= gpgpu_instr_t'(icache_rdata[15:0]);
                        state <= EXECUTE;
                    end
                end

                EXECUTE: begin
                    is_executing <= 1'b1;
                    rf_rd <= current_instr.rd;
                    
                    case (current_instr.opcode)
                        OP_FADD, OP_FFMA: begin
                            rf_we <= 1'b1;
                            wb_data <= alu_out;
                            pc[scheduled_warp] <= pc[scheduled_warp] + 1'b1;
                            state <= FETCH;
                        end
                        
                        OP_LDS: begin
                            rf_we <= 1'b1;
                            wb_data <= lds_out; // Assume 1-cycle shared mem
                            pc[scheduled_warp] <= pc[scheduled_warp] + 1'b1;
                            state <= FETCH;
                        end

                        OP_STS: begin
                            // Store to shared memory, no register writeback
                            pc[scheduled_warp] <= pc[scheduled_warp] + 1'b1;
                            state <= FETCH;
                        end

                        OP_LDGS: begin
                            // GLOBAL MEMORY LOAD - Stalls this warp!
                            gmem_req.valid <= 1'b1;
                            gmem_req.warp_id <= scheduled_warp;
                            gmem_req.dest_reg <= current_instr.rd;
                            gmem_req.addr <= current_instr.smem_offset; // Simplified
                            
                            warp_stall[scheduled_warp] <= 1'b1;
                            state <= FETCH; // Scheduler will instantly pick another warp
                        end

                        OP_EXIT: begin
                            warp_active[scheduled_warp] <= 1'b0;
                            state <= FETCH;
                        end

                        default: begin
                            pc[scheduled_warp] <= pc[scheduled_warp] + 1'b1;
                            state <= FETCH;
                        end
                    endcase
                end
            endcase
        end
    end

endmodule

// Simplified Shared Memory Wrapper
module shared_memory #(
    parameter int ADDR_W = 11,
    parameter int DATA_W = 512
)(
    input  logic             clk,
    input  logic [ADDR_W-1:0] addr,
    input  logic             wr_en,
    input  logic [DATA_W-1:0] wr_data,
    output logic [DATA_W-1:0] rd_data
);
    logic [DATA_W-1:0] mem [0:2**ADDR_W-1];
    always_ff @(posedge clk) begin
        if (wr_en) mem[addr] <= wr_data;
        rd_data <= mem[addr];
    end
endmodule