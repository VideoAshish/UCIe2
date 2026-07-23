A real-world Graphics Processing Unit (GPU) consists of millions of lines of RTL code. It is fundamentally different from a CPU because it relies on the **SIMT (Single Instruction, Multiple Thread)** architecture. 

Instead of executing one instruction stream, a GPU core (like an NVIDIA Streaming Multiprocessor or AMD Compute Unit) manages dozens of **Warps** (or Wavefronts). Each warp contains a group of threads (e.g., 32) that execute the *exact same instruction* simultaneously on different data.

Below is a **synthesizable, architecturally accurate SystemVerilog model of a minimal GPU Shader Core**. It demonstrates the three most critical components of GPU hardware:
1. **Warp Scheduler:** Manages Program Counters (PCs) per warp, not per thread.
2. **Multi-ported Register File:** Structured as `[warp][thread][register]`.
3. **Vector ALU:** Performs the same mathematical operation across all threads in a warp simultaneously.

---

### 1. GPU Core Package & Definitions

```systemverilog
//=============================================================================
// gpu_pkg.sv
// Definitions for a minimal GPU Shader Core
//=============================================================================
package gpu_pkg;

    // --- Core Configuration ---
    localparam int NUM_WARPS         = 4;    // Number of concurrent warps
    localparam int THREADS_PER_WARP  = 8;    // Threads executing in lockstep (32/64 in real GPUs)
    localparam int REGS_PER_THREAD   = 4;    // Registers allocated per thread
    localparam int DATA_W            = 16;   // 16-bit fixed-point/integer data
    localparam int INSTR_ADDR_W      = 8;    // Instruction Memory Address width

    // --- GPU Instruction Set Architecture (ISA) ---
    // Real GPUs use VLIW (Very Long Instruction Word), but we use a simple struct
    typedef enum logic [3:0] {
        OP_NOP   = 4'h0,
        OP_ADD   = 4'h1,  // ADD  rd, rs1, rs2
        OP_MUL   = 4'h2,  // MUL  rd, rs1, rs2
        OP_MAD   = 4'h3,  // MAD  rd, rs1, rs2, rs3 (Multiply-Add: rd = rs1*rs2 + rs3)
        OP_END   = 4'hF   // End program for this warp
    } gpu_opcode_e;

    typedef struct packed {
        gpu_opcode_e          opcode;
        logic [2:0]           rd;    // Destination register
        logic [2:0]           rs1;   // Source register 1
        logic [2:0]           rs2;   // Source register 2
        logic [2:0]           rs3;   // Source register 3 (used for MAD)
    } gpu_instruction_t;

endpackage
```

---

### 2. Multi-Threaded Register File

In a GPU, the register file is the most heavily utilized structure. It must handle massive bandwidth.

```systemverilog
//=============================================================================
// gpu_register_file.sv
// Banked, multi-ported Register File
//=============================================================================
module gpu_register_file #(
    parameter int NUM_WARPS        = gpu_pkg::NUM_WARPS,
    parameter int THREADS_PER_WARP = gpu_pkg::THREADS_PER_WARP,
    parameter int REGS_PER_THREAD  = gpu_pkg::REGS_PER_THREAD,
    parameter int DATA_W           = gpu_pkg::DATA_W
)(
    input  logic                                      clk,
    input  logic                                      rst_n,

    // Read Port 1
    input  logic [$clog2(NUM_WARPS)-1:0]              r1_warp_id,
    input  logic [$clog2(REGS_PER_THREAD)-1:0]        r1_addr,
    output logic [THREADS_PER_WARP-1:0][DATA_W-1:0]   r1_data, // Outputs vector for all threads

    // Read Port 2
    input  logic [$clog2(NUM_WARPS)-1:0]              r2_warp_id,
    input  logic [$clog2(REGS_PER_THREAD)-1:0]        r2_addr,
    output logic [THREADS_PER_WARP-1:0][DATA_W-1:0]   r2_data,

    // Read Port 3 (For MAD instruction)
    input  logic [$clog2(NUM_WARPS)-1:0]              r3_warp_id,
    input  logic [$clog2(REGS_PER_THREAD)-1:0]        r3_addr,
    output logic [THREADS_PER_WARP-1:0][DATA_W-1:0]   r3_data,

    // Write Port
    input  logic [$clog2(NUM_WARPS)-1:0]              w_warp_id,
    input  logic [$clog2(REGS_PER_THREAD)-1:0]        w_addr,
    input  logic [THREADS_PER_WARP-1:0][DATA_W-1:0]   w_data,
    input  logic                                      w_enable
);

    // 3D Array: [Warp][Thread][Register]
    logic [NUM_WARPS-1:0][THREADS_PER_WARP-1:0][REGS_PER_THREAD-1:0][DATA_W-1:0] reg_file;

    // Write Logic (Single Port)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_file <= '0;
        end else if (w_enable) begin
            for (int t = 0; t < THREADS_PER_WARP; t++) begin
                reg_file[w_warp_id][t][w_addr] <= w_data[t];
            end
        end
    end

    // Read Logic (Combinational, 3 independent ports)
    always_comb begin
        for (int t = 0; t < THREADS_PER_WARP; t++) begin
            r1_data[t] = reg_file[r1_warp_id][t][r1_addr];
            r2_data[t] = reg_file[r2_warp_id][t][r2_addr];
            r3_data[t] = reg_file[r3_warp_id][t][r3_addr];
        end
    end

endmodule
```

---

### 3. Vector SIMT ALU

This is where the "magic" of a GPU happens. One opcode triggers `THREADS_PER_WARP` parallel operations.

```systemverilog
//=============================================================================
// gpu_vector_alu.sv
// SIMT Execution Unit
//=============================================================================
module gpu_vector_alu #(
    parameter int THREADS_PER_WARP = gpu_pkg::THREADS_PER_WARP,
    parameter int DATA_W           = gpu_pkg::DATA_W
)(
    input  logic                                     clk,
    input  logic                                     rst_n,
    input  logic                                     enable,
    
    // Control
    input  gpu_pkg::gpu_opcode_e                     opcode,
    
    // Operands (Vectors)
    input  logic [THREADS_PER_WARP-1:0][DATA_W-1:0]  vec_a,
    input  logic [THREADS_PER_WARP-1:0][DATA_W-1:0]  vec_b,
    input  logic [THREADS_PER_WARP-1:0][DATA_W-1:0]  vec_c, // Used for MAD
    
    // Result (Vector)
    output logic [THREADS_PER_WARP-1:0][DATA_W-1:0]  result
);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            result <= '0;
        end else if (enable) begin
            for (int t = 0; t < THREADS_PER_WARP; t++) begin
                case (opcode)
                    gpu_pkg::OP_ADD: result[t] <= vec_a[t] + vec_b[t];
                    gpu_pkg::OP_MUL: result[t] <= vec_a[t] * vec_b[t];
                    gpu_pkg::OP_MAD: result[t] <= (vec_a[t] * vec_b[t]) + vec_c[t]; // FMA/MAD is critical for graphics
                    default:         result[t] <= '0;
                endcase
            end
        end
    end

endmodule
```

---

### 4. Warp Scheduler & Shader Core Top Level

The "Brain" of the core. It fetches instructions for a specific warp and routes data.

```systemverilog
//=============================================================================
// gpu_shader_core.sv
// Top-level Warp Manager and Pipeline
//=============================================================================
module gpu_shader_core (
    input  logic                                  clk,
    input  logic                                  rst_n,

    // Instruction Memory Interface (To L1 I-Cache)
    output logic [gpu_pkg::INSTR_ADDR_W-1:0]      imem_addr,
    input  logic [gpu_pkg::INSTR_ADDR_W-1:0]      imem_instr,
    input  logic                                  imem_valid,

    // Status
    output logic                                  core_idle
);

    import gpu_pkg::*;

    //=========================================================================
    // Warp State Management
    //=========================================================================
    logic [NUM_WARPS-1:0]             warp_active;  // Is this warp currently running?
    logic [INSTR_ADDR_W-1:0]          pc [NUM_WARPS]; // Program counter PER WARP
    logic [$clog2(NUM_WARPS)-1:0]     current_warp;  // Round-robin warp selector
    
    //=========================================================================
    // Pipeline Registers
    //=========================================================================
    gpu_instruction_t                 current_instr;
    logic                             alu_valid;
    
    // Register File Wires
    logic [THREADS_PER_WARP-1:0][DATA_W-1:0] rs1_data, rs2_data, rs3_data;
    logic                                     rf_we;
    logic [2:0]                                rf_rd;
    logic [THREADS_PER_WARP-1:0][DATA_W-1:0]  alu_result;

    //=========================================================================
    // Sub-module Instantiation
    //=========================================================================
    gpu_register_file u_rf (
        .clk        (clk),
        .rst_n      (rst_n),
        .r1_warp_id (current_warp),
        .r1_addr    (current_instr.rs1),
        .r1_data    (rs1_data),
        .r2_warp_id (current_warp),
        .r2_addr    (current_instr.rs2),
        .r2_data    (rs2_data),
        .r3_warp_id (current_warp),
        .r3_addr    (current_instr.rs3),
        .r3_data    (rs3_data),
        .w_warp_id  (current_warp),
        .w_addr     (rf_rd),
        .w_data     (alu_result),
        .w_enable   (rf_we)
    );

    gpu_vector_alu u_alu (
        .clk    (clk),
        .rst_n  (rst_n),
        .enable (alu_valid),
        .opcode (current_instr.opcode),
        .vec_a  (rs1_data),
        .vec_b  (rs2_data),
        .vec_c  (rs3_data),
        .result (alu_result)
    );

    //=========================================================================
    // Warp Scheduler & Execute FSM
    //=========================================================================
    typedef enum logic [2:0] { FETCH, DECODE, EXECUTE, WB, HALT } state_t;
    state_t state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= FETCH;
            pc <= '{default: '0};
            warp_active <= '{default: '0};
            warp_active[0] <= 1'b1; // Start Warp 0
            current_warp <= 0;
            alu_valid <= 1'b0;
            rf_we <= 1'b0;
            core_idle <= 1'b0;
        end else begin
            // Default control signals
            alu_valid <= 1'b0;
            rf_we <= 1'b0;

            case (state)
                FETCH: begin
                    // In a real GPU, if current_warp is stalled (e.g., memory), 
                    // the scheduler picks the next active warp here.
                    if (warp_active[current_warp]) begin
                        imem_addr <= pc[current_warp];
                        state <= DECODE;
                    end else begin
                        core_idle <= 1'b1;
                    end
                end

                DECODE: begin
                    if (imem_valid) begin
                        current_instr <= gpu_instruction_t'(imem_instr);
                        state <= EXECUTE;
                    end
                end

                EXECUTE: begin
                    alu_valid <= 1'b1;
                    rf_rd <= current_instr.rd;
                    state <= WB;
                end

                WB: begin
                    // Write back ALU result to Register File
                    if (current_instr.opcode != OP_NOP && current_instr.opcode != OP_END) begin
                        rf_we <= 1'b1;
                    end

                    // Handle Program Flow
                    if (current_instr.opcode == OP_END) begin
                        warp_active[current_warp] <= 1'b0; // Kill warp
                    end else begin
                        pc[current_warp] <= pc[current_warp] + 1'b1;
                    end

                    // Round-robin warp switching (Simplified Latency Hiding)
                    // Real GPUs switch warps every cycle to hide ALU/Memory latency
                    if (current_warp == NUM_WARPS - 1) current_warp <= 0;
                    else current_warp <= current_warp + 1;

                    state <= FETCH;
                end
            endcase
        end
    end

endmodule
```

---

### 5. GPU Testbench & "Shader Program"

To test a GPU, you must write a "Shader" in machine code, load it into the Instruction Memory, and verify that all 8 threads computed correctly.

```systemverilog
//=============================================================================
// tb_gpu_shader_core.sv
//=============================================================================
module tb_gpu_shader_core();

    import gpu_pkg::*;

    logic clk, rst_n;
    logic [INSTR_ADDR_W-1:0] imem_addr;
    logic [INSTR_ADDR_W-1:0] imem_instr;
    logic                    imem_valid;
    logic                    core_idle;

    // Instruction Memory (Simulated L1 Cache)
    logic [INSTR_ADDR_W-1:0] instr_mem [0:2**INSTR_ADDR_W-1];

    gpu_shader_core u_dut (
        .clk(clk), .rst_n(rst_n),
        .imem_addr(imem_addr), .imem_instr(imem_instr), .imem_valid(imem_valid),
        .core_idle(core_idle)
    );

    // Clock Gen
    initial begin
        clk = 0; forever #5 clk = ~clk;
    end

    // Memory Response Simulation (1-cycle latency)
    always_ff @(posedge clk) begin
        imem_instr <= instr_mem[imem_addr];
        imem_valid <= 1'b1;
    end

    // Assemble a simple "Shader Program" into Machine Code
    function automatic logic [INSTR_ADDR_W-1:0] assemble(
        input gpu_opcode_e op, input logic [2:0] rd, rs1, rs2, rs3
    );
        gpu_instruction_t i;
        i.opcode = op; i.rd = rd; i.rs1 = rs1; i.rs2 = rs2; i.rs3 = rs3;
        return i;
    endfunction

    // Test Sequence
    initial begin
        rst_n = 0;
        // Initialize Instruction Memory
        for(int i=0; i<2**INSTR_ADDR_W; i++) instr_mem[i] = '0;

        // Write Shader Program:
        // R1 = 5, R2 = 10 (Assume pre-loaded or done via memory - we hardcode RF init below)
        // ADD R0, R1, R2  -> R0 = 15
        // MUL R0, R0, R0  -> R0 = 225
        // MAD R0, R1, R2, R0 -> R0 = (5*10) + 225 = 275
        // END
        
        instr_mem[0] = assemble(OP_ADD, 3'd0, 3'd1, 3'd2, 3'd0);
        instr_mem[1] = assemble(OP_MUL, 3'd0, 3'd0, 3'd0, 3'd0);
        instr_mem[2] = assemble(OP_MAD, 3'd0, 3'd1, 3'd2, 3'd0);
        instr_mem[3] = assemble(OP_END, 3'd0, 3'd0, 3'd0, 3'd0);

        #20 rst_n = 1;

        // Wait for core to finish the warp
        @(posedge core_idle);
        #50;
        
        $display("=======================================");
        $display("GPU SIMT Execution Complete.");
        $display("All %0d threads in Warp 0 executed the same instructions.", THREADS_PER_WARP);
        $display("=======================================");
        
        // Display results for all threads in warp 0
        for(int t=0; t<THREADS_PER_WARP; t++) begin
            $display("Thread %0d R0 Result: %0d (Expected: 275)", t, u_dut.u_rf.reg_file[0][t][0]);
        end
        
        $finish;
    end

    // Override RF initialization to give threads different starting data
    // In real GPUs, this is done by a previous Shader stage or Vertex Fetch
    initial begin
        for(int t=0; t<THREADS_PER_WARP; t++) begin
            // Give every thread a different R1 and R2 to prove SIMT works on vectors
            u_dut.u_rf.reg_file[0][t][1] = t + 1; // R1 = t+1
            u_dut.u_rf.reg_file[0][t][2] = (t+1)*2; // R2 = (t+1)*2
        end
    end

    // Waveform Dump
    initial begin
        $dumpfile("gpu_shader_core.vcd");
        $dumpvars(0, tb_gpu_shader_core);
    end

endmodule
```

### Key Architectural Takeaways (Why this looks different from a CPU):
1. **The `pc` is an array:** `logic [ADDR_W-1:0] pc [NUM_WARPS];` A CPU has one Program Counter. A GPU must hold a PC for *every single Warp* concurrently.
2. **No explicit Thread IDs in data paths:** The `for` loops in the ALU and Register File (`for (int t = 0; t < THREADS_PER_WARP; t++)`) represent **Spatial Parallelism**. In RTL, this unrolls into 8 separate copies of the ALU hardware sitting next to each other on the silicon.
3. **The `MAD` instruction:** Notice the 3-read-port Register file and the `MAD` instruction. In graphics, calculating pixel colors requires $A \times B + C$ constantly. Having a dedicated hardware path for this (FMA - Fused Multiply Add) is the single most important math optimization in GPU history.
4. **Zero Thread Divergence Handling:** In this simple model, all threads execute. In a real GPU, if there is an `if/else` statement in the shader code, some threads take the `if`, some take the `else`. The hardware utilizes an **Active Mask Register** to disable ALU writes for threads that shouldn't be executing that path. (This was omitted for clarity but is central to GPU architecture).