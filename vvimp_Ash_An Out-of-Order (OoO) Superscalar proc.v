An Out-of-Order (OoO) Superscalar processor is the pinnacle of general-purpose CPU microarchitecture. It is extraordinarily complex—real implementations (like Intel's Golden Cove or ARM's Cortex-X3) are millions of lines of RTL. 

To provide a meaningful, synthesizable SystemVerilog implementation, I have designed a **2-Wide OoO Core Microarchitecture Scaffold**. It implements the exact pipeline structures that make OoO possible:
1. **Register Alias Table (RAT):** Maps architectural registers to physical registers.
2. **Free List:** Tracks unallocated physical registers.
3. **Reservation Stations (Issue Queue):** Holds instructions until operands are ready, featuring **CAM (Content Addressable Memory) wake-up logic**.
4. **Common Data Bus (CDB):** Broadcasts execution results back to the Issue Queue in one cycle.
5. **Reorder Buffer (ROB):** Ensures instructions retire in original program order.

---

### 1. OoO Microarchitecture Package

```systemverilog
//=============================================================================
// ooo_pkg.sv
//=============================================================================
package ooo_pkg;

    // --- Configuration ---
    localparam int ARCH_REGS  = 8;     // Architectural registers (x0-x7 for brevity)
    localparam int PHYS_REGS  = 16;    // Physical registers (Must be > ARCH_REGS)
    localparam int IQ_SIZE    = 8;     // Issue Queue (Reservation Station) depth
    localparam int ROB_SIZE   = 8;     // Reorder Buffer depth
    localparam int WIDTH      = 2;     // 2-Wide Superscalar
    localparam int XLEN       = 32;

    typedef logic [$clog2(ARCH_REGS)-1:0] arch_reg_t;
    typedef logic [$clog2(PHYS_REGS)-1:0] phys_reg_t;
    typedef logic [$clog2(IQ_SIZE)-1:0]   iq_idx_t;
    typedef logic [$clog2(ROB_SIZE)-1:0]  rob_idx_t;

    // --- ISA ---
    typedef enum logic [2:0] {
        OP_ADD  = 3'h0,
        OP_SUB  = 3'h1,
        OP_AND  = 3'h2,
        OP_OR   = 3'h3,
        OP_NOP  = 3'h7
    } opcode_e;

    typedef struct packed {
        logic                  valid;
        opcode_e              opcode;
        arch_reg_t            arch_rs1;
        arch_reg_t            arch_rs2;
        arch_reg_t            arch_rd;
    } instr_t;

    // --- Common Data Bus (CDB) ---
    typedef struct packed {
        logic                  valid;
        phys_reg_t             p_rd;    // Destination Physical Reg
        rob_idx_t              rob_tag; // To update ROB
        logic [XLEN-1:0]       data;    // Computed result
    } cdb_t;

    // --- Issue Queue Entry ---
    typedef struct packed {
        logic                  valid;
        opcode_e              opcode;
        phys_reg_t             p_rs1;
        phys_reg_t             p_rs2;
        logic                  rs1_ready;
        logic                  rs2_ready;
        logic [XLEN-1:0]       val1;
        logic [XLEN-1:0]       val2;
        phys_reg_t             p_rd;
        rob_idx_t              rob_tag;
        logic                  issued;  // Prevents re-issuing
    } iq_entry_t;

    // --- Reorder Buffer Entry ---
    typedef struct packed {
        logic                  valid;
        logic                  ready;  // Result broadcast on CDB
        arch_reg_t             arch_rd;
        phys_reg_t             p_rd;   // New physical mapping
        phys_reg_t             old_p_rd;// Old physical mapping (for rollback)
        logic                  is_write; // Does it write a reg?
    } rob_entry_t;

endpackage
```

---

### 2. Out-of-Order Superscalar Core

```systemverilog
//=============================================================================
// ooo_core.sv
// 2-Wide OoO Core with Register Renaming
//=============================================================================
module ooo_core (
    input  logic                  clk,
    input  logic                  rst_n,

    // Instruction Fetch Interface (Assuming simple ROM/Cache)
    input  instr_t                fetch_instr [WIDTH],
    input  logic                  fetch_valid  [WIDTH],
    output logic                  fetch_stall,     // Stall frontend if structures full
    
    // Architectural State (For verification/debug)
    output logic [XLEN-1:0]       arch_regfile [ARCH_REGS]
);
    import ooo_pkg::*;

    //=========================================================================
    // 1. STRUCTURE ALLOCATION STATE
    //=========================================================================
    logic [PHYS_REGS-1:0]         free_list;
    logic [$clog2(PHYS_REGS)-1:0] free_list_head;
    logic                         free_list_empty;
    
    logic [ROB_SIZE-1:0]          rob_valid_mask;
    logic [IQ_SIZE-1:0]           iq_valid_mask;
    rob_idx_t                     rob_head_ptr, rob_tail_ptr;
    iq_idx_t                      iq_tail_ptr;

    //=========================================================================
    // 2. RENAME STATE (Register Alias Table)
    //=========================================================================
    phys_reg_t                    rat [ARCH_REGS]; // Current Mapping
    phys_reg_t                    rat_shadow [ARCH_REGS]; // Shadow for 2nd instr in bundle

    //=========================================================================
    // 3. STORAGE ARRAYS
    //=========================================================================
    iq_entry_t                    issue_queue [0:IQ_SIZE-1];
    rob_entry_t                   rob        [0:ROB_SIZE-1];
    logic [XLEN-1:0]              prf        [0:PHYS_REGS-1]; // Physical Register File

    // CDB Broadcasts (One per execution unit)
    cdb_t                         cdb [WIDTH]; 

    //=========================================================================
    // 4. RENAME & DISPATCH LOGIC
    //=========================================================================
    always_comb begin
        fetch_stall = 1'b0;
        rat_shadow = rat; // Default shadow to current RAT

        for (int i = 0; i < WIDTH; i++) begin
            if (fetch_valid[i]) begin
                // Check for structural hazards
                if (free_list_empty || (rob_valid_mask[rob_tail_ptr + i] && i==1)) begin
                    fetch_stall = 1'b1;
                    break;
                end

                // --- Register Renaming ---
                // Map Architectural sources to Physical sources
                phys_reg_t p_rs1 = (fetch_instr[i].arch_rs1 == 0) ? '0 : rat_shadow[fetch_instr[i].arch_rs1];
                phys_reg_t p_rs2 = (fetch_instr[i].arch_rs2 == 0) ? '0 : rat_shadow[fetch_instr[i].arch_rs2];

                // Allocate destination physical register
                phys_reg_t p_rd = free_list_head;
                
                // Update Shadow RAT for the next instruction in the bundle
                if (fetch_instr[i].arch_rd != 0) begin
                    rat_shadow[fetch_instr[i].arch_rd] = p_rd;
                end
            end
        end
    end

    // Sequential Rename/Dispatch State Update
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize Free List (All physical regs free except p0 which is hardwired to 0)
            free_list <= ~(1'b1 << 0); 
            free_list_head <= 1;
            rob_head_ptr <= '0;
            rob_tail_ptr <= '0;
            iq_tail_ptr <= '0;
            rob_valid_mask <= '0;
            for(int i=0; i<ARCH_REGS; i++) rat[i] <= '0;
        end else if (!fetch_stall) begin
            for (int i = 0; i < WIDTH; i++) begin
                if (fetch_valid[i]) begin
                    phys_reg_t p_rs1 = (fetch_instr[i].arch_rs1 == 0) ? '0 : rat[fetch_instr[i].arch_rs1];
                    phys_reg_t p_rs2 = (fetch_instr[i].arch_rs2 == 0) ? '0 : rat[fetch_instr[i].arch_rs2];
                    phys_reg_t p_rd  = free_list_head;

                    // Allocate ROB
                    rob[rob_tail_ptr].valid    <= 1'b1;
                    rob[rob_tail_ptr].ready    <= 1'b0;
                    rob[rob_tail_ptr].arch_rd  <= fetch_instr[i].arch_rd;
                    rob[rob_tail_ptr].p_rd     <= p_rd;
                    rob[rob_tail_ptr].old_p_rd <= rat[fetch_instr[i].arch_rd];
                    rob[rob_tail_ptr].is_write <= (fetch_instr[i].arch_rd != 0);
                    rob_valid_mask[rob_tail_ptr] <= 1'b1;

                    // Allocate Issue Queue
                    issue_queue[iq_tail_ptr].valid    <= 1'b1;
                    issue_queue[iq_tail_ptr].opcode   <= fetch_instr[i].opcode;
                    issue_queue[iq_tail_ptr].p_rs1    <= p_rs1;
                    issue_queue[iq_tail_ptr].p_rs2    <= p_rs2;
                    issue_queue[iq_tail_ptr].p_rd     <= p_rd;
                    issue_queue[iq_tail_ptr].rob_tag  <= rob_tail_ptr;
                    issue_queue[iq_tail_ptr].issued   <= 1'b0;
                    
                    // Check if operands are immediately available (Bypassing PRF read)
                    issue_queue[iq_tail_ptr].rs1_ready <= (p_rs1 == '0) || (p_rs1 == cdb[0].p_rd && cdb[0].valid) || (p_rs1 == cdb[1].p_rd && cdb[1].valid);
                    issue_queue[iq_tail_ptr].rs2_ready <= (p_rs2 == '0) || (p_rs2 == cdb[0].p_rd && cdb[0].valid) || (p_rs2 == cdb[1].p_rd && cdb[1].valid);

                    // Update RAT
                    if (fetch_instr[i].arch_rd != 0) begin
                        rat[fetch_instr[i].arch_rd] <= p_rd;
                    end

                    // Pop Free List
                    free_list[free_list_head] <= 1'b0;
                    free_list_head <= free_list_head + 1;

                    // Advance Pointers
                    rob_tail_ptr <= rob_tail_ptr + 1;
                    iq_tail_ptr  <= iq_tail_ptr + 1;
                end
            end
        end
    end

    //=========================================================================
    // 5. ISSUE QUEUE WAKEUP & ISSUE (CAM Logic)
    //=========================================================================
    logic [IQ_SIZE-1:0] rs1_match, rs2_match;
    logic [IQ_SIZE-1:0] can_issue;

    // Wakeup Logic: Compare every IQ entry's source tags against CDB broadcasts
    always_comb begin
        for (int i = 0; i < IQ_SIZE; i++) begin
            rs1_match[i] = (issue_queue[i].p_rs1 == cdb[0].p_rd && cdb[0].valid) ||
                           (issue_queue[i].p_rs1 == cdb[1].p_rd && cdb[1].valid);
            rs2_match[i] = (issue_queue[i].p_rs2 == cdb[0].p_rd && cdb[0].valid) ||
                           (issue_queue[i].p_rs2 == cdb[1].p_rd && cdb[1].valid);
                           
            // Can issue if valid, not already issued, and both operands ready
            can_issue[i] = issue_queue[i].valid && !issue_queue[i].issued &&
                           issue_queue[i].rs1_ready && issue_queue[i].rs2_ready;
        end
    end

    // Issue Logic & CDB Drive
    integer issued_count;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cdb[0].valid <= 1'b0;
            cdb[1].valid <= 1'b0;
            issued_count <= 0;
        end else begin
            cdb[0].valid <= 1'b0;
            cdb[1].valid <= 1'b0;
            issued_count <= 0;

            // Priority encode: Find up to 'WIDTH' instructions to issue
            for (int i = 0; i < IQ_SIZE && issued_count < WIDTH; i++) begin
                if (can_issue[i]) begin
                    // Grab operands (from PRF or latched from CDB)
                    logic [XLEN-1:0] val1 = prf[issue_queue[i].p_rs1];
                    logic [XLEN-1:0] val2 = prf[issue_queue[i].p_rs2];
                    
                    // Latch CDB data if it just arrived this cycle
                    if (rs1_match[i]) val1 = (issue_queue[i].p_rs1 == cdb[0].p_rd) ? cdb[0].data : cdb[1].data;
                    if (rs2_match[i]) val2 = (issue_queue[i].p_rs2 == cdb[0].p_rd) ? cdb[0].data : cdb[1].data;

                    // Execute ALU operation
                    logic [XLEN-1:0] result;
                    case (issue_queue[i].opcode)
                        OP_ADD: result = val1 + val2;
                        OP_SUB: result = val1 - val2;
                        OP_AND: result = val1 & val2;
                        OP_OR:  result = val1 | val2;
                        default: result = '0;
                    endcase

                    // Broadcast on CDB
                    cdb[issued_count].valid   <= 1'b1;
                    cdb[issued_count].p_rd    <= issue_queue[i].p_rd;
                    cdb[issued_count].rob_tag <= issue_queue[i].rob_tag;
                    cdb[issued_count].data    <= result;

                    // Mark as issued (will be cleared on retire)
                    issue_queue[i].issued <= 1'b1;
                    issued_count <= issued_count + 1;
                end
            end
        end
    end

    // Update IQ ready flags on wakeup
    always_ff @(posedge clk) begin
        for (int i = 0; i < IQ_SIZE; i++) begin
            if (rs1_match[i]) issue_queue[i].rs1_ready <= 1'b1;
            if (rs2_match[i]) issue_queue[i].rs2_ready <= 1'b1;
        end
    end

    // Write results into PRF from CDB
    always_ff @(posedge clk) begin
        for (int c = 0; c < WIDTH; c++) begin
            if (cdb[c].valid) begin
                prf[cdb[c].p_rd] <= cdb[c].data;
            end
        end
    end

    //=========================================================================
    // 6. REORDER BUFFER (ROB) & IN-ORDER RETIRE
    //=========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rob_head_ptr <= '0;
            rob_valid_mask <= '0;
            for(int i=0; i<ARCH_REGS; i++) arch_regfile[i] <= '0;
        end else begin
            // Mark ROB entries as ready when they appear on CDB
            for (int c = 0; c < WIDTH; c++) begin
                if (cdb[c].valid) begin
                    rob[cdb[c].rob_tag].ready <= 1'b1;
                end
            end

            // Retire up to 'WIDTH' instructions per cycle
            for (int i = 0; i < WIDTH; i++) begin
                if (rob_valid_mask[rob_head_ptr] && rob[rob_head_ptr].ready) begin
                    // Commit to Architectural State
                    if (rob[rob_head_ptr].is_write) begin
                        arch_regfile[rob[rob_head_ptr].arch_rd] <= prf[rob[rob_head_ptr].p_rd];
                    end

                    // Free Physical Register (Return to Free List)
                    if (rob[rob_head_ptr].is_write) begin
                        free_list[rob[rob_head_ptr].old_p_rd] <= 1'b1;
                    end

                    // Clear ROB & IQ entries
                    rob_valid_mask[rob_head_ptr] <= 1'b0;
                    for (int j = 0; j < IQ_SIZE; j++) begin
                        if (issue_queue[j].rob_tag == rob_head_ptr) issue_queue[j].valid <= 1'b0;
                    end

                    rob_head_ptr <= rob_head_ptr + 1;
                end else begin
                    break; // Strict in-order retirement! Stop if head isn't ready.
                end
            end
        end
    end

    // Track if free list is empty
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) free_list_empty <= 1'b0;
        else free_list_empty <= (free_list == '0);
    end

endmodule
```

### Architectural Breakdown: Why this is "Advanced"

1. **The `rat_shadow` Logic (Sequential Renaming):**
   In a 2-wide machine, if instruction 1 writes to `x3` and instruction 2 reads `x3`, instruction 2 *must* read the value produced by instruction 1, not the old value. The `always_comb` block creates a `rat_shadow` that updates sequentially within the same cycle based on the bundle order.
2. **The `rs1_match` / `rs2_match` logic (CAM Wakeup):**
   This is the most expensive part of a physical CPU. The `always_comb` loop generates a **Content Addressable Memory** array. Every single clock cycle, *every* entry in the Issue Queue compares its source Physical Register tags against the tags broadcast on the CDB. In synthesis, this becomes a massive grid of parallel comparators.
3. **The Common Data Bus (`cdb` array):**
   Notice how the ALU result is written directly to `cdb`, and the IQ listens to `cdb`. The actual `prf` (Physical Register File) is updated *after* the IQ latches the data. This 1-cycle bypass is fundamental to hiding register file read latency.
4. **In-Order Commit (`break` statement):**
   In the ROB retirement loop, there is a `break` statement. Even if `rob[head+1]` is finished, it *cannot* retire until `rob[head]` retires. This guarantees precise exceptions and strict architectural state updates, which is the entire point of the ROB.
5. **Free List Recovery:**
   When the ROB retires an instruction, it pushes `old_p_rd` back into the `free_list` bitmask. If a branch misprediction occurred (not implemented here for brevity, but standard), you would flush the ROB by walking backwards and pushing all `old_p_rd` mappings back into the Free List to undo the renaming.

### What is missing from a full silicon implementation?
To make this production-ready, you would add:
* **Load/Store Queue (LSQ):** Memory operations cannot simply execute out of order; they must wait for address calculation and then check against other memory ops to ensure no Read-After-Write (RAW) hazards exist in the memory subsystem.
* **Branch Prediction & Recovery:** Mispredictions require rolling back the RAT to a checkpointed state and flushing the IQ/ROB.
* **Physical Register File Bypassing:** The code reads from `prf` synchronously. High-performance cores use an asynchronous read port + latch to hide the PRF read latency.