package ucie3_mpg_pkg;

    // ============================================================
    // UCIe common parameters
    // ============================================================

    localparam int UCIe_68B_BYTES  = 68;
    localparam int UCIe_256B_BYTES = 256;

    localparam int UCIe_68B_W  = UCIe_68B_BYTES  * 8;
    localparam int UCIe_256B_W = UCIe_256B_BYTES * 8;

    // ============================================================
    // MPG stack identifiers
    // ============================================================

    typedef enum logic {
        MPG_STACK_0 = 1'b0,
        MPG_STACK_1 = 1'b1
    } mpg_stack_id_e;

    // ============================================================
    // Source of a transmitted flit.
    //
    // This is useful because the MPG bandwidth rule must apply
    // to both FDI and retry-buffer traffic.
    // ============================================================

    typedef enum logic [1:0] {
        MPG_SRC_FDI0   = 2'd0,
        MPG_SRC_FDI1   = 2'd1,
        MPG_SRC_REPLAY = 2'd2,
        MPG_SRC_NOP    = 2'd3
    } mpg_tx_source_e;

    // ============================================================
    // Generic MPG flit descriptor.
    // ============================================================

    typedef struct packed {
        logic                   valid;
        logic                   stack_id;
        logic [1:0]             source;
    } mpg_flit_meta_t;

endpackage