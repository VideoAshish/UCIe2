//==============================================================================
// File        : ucie_pkg.sv
// Project     : UCIe Adapter
// Description : Common package for UCIe Adapter RTL
//
// Notes:
//   - Synthesizable package
//   - Parameterizable widths
//   - Shared typedefs
//   - Common utility functions
//==============================================================================

package ucie_pkg;

    //------------------------------------------------------------
    // Version
    //------------------------------------------------------------
    parameter int UCIE_MAJOR_VERSION = 1;
    parameter int UCIE_MINOR_VERSION = 0;

    //------------------------------------------------------------
    // Global Configuration
    //------------------------------------------------------------
    parameter int FLIT_WIDTH          = 256;
    parameter int DATA_WIDTH          = 256;

    parameter int HEADER_WIDTH        = 32;
    parameter int CRC_WIDTH           = 32;
    parameter int ECC_WIDTH           = 8;

    parameter int VC_WIDTH            = 3;
    parameter int SEQ_WIDTH           = 10;

    parameter int NUM_VC              = (1 << VC_WIDTH);

    parameter int MAX_REPLAY_WINDOW   = 1024;
    parameter int MAX_PAYLOAD_BYTES   = 4096;

    parameter int CREDIT_WIDTH        = 8;

    parameter int PIPE_STAGES         = 4;

    //------------------------------------------------------------
    // Derived Parameters
    //------------------------------------------------------------
    localparam int CTRL_WIDTH = 8;

    localparam int PAYLOAD_WIDTH =
        FLIT_WIDTH -
        HEADER_WIDTH -
        SEQ_WIDTH -
        VC_WIDTH -
        CTRL_WIDTH -
        CRC_WIDTH;

    //------------------------------------------------------------
    // Virtual Channels
    //------------------------------------------------------------
    typedef enum logic [VC_WIDTH-1:0] {

        VC_POSTED      = 3'd0,
        VC_NONPOSTED   = 3'd1,
        VC_COMPLETION  = 3'd2,
        VC_CONTROL     = 3'd3,
        VC_STREAM      = 3'd4,
        VC_RSVD5       = 3'd5,
        VC_RSVD6       = 3'd6,
        VC_RSVD7       = 3'd7

    } vc_t;

    //------------------------------------------------------------
    // Packet Type
    //------------------------------------------------------------
    typedef enum logic [3:0] {

        PKT_MEM_RD      = 4'd0,
        PKT_MEM_WR      = 4'd1,
        PKT_COMPLETION  = 4'd2,
        PKT_MSG         = 4'd3,
        PKT_STREAM      = 4'd4,
        PKT_ACK         = 4'd5,
        PKT_NACK        = 4'd6,
        PKT_IDLE        = 4'd7

    } pkt_type_t;

    //------------------------------------------------------------
    // Link State
    //------------------------------------------------------------
    typedef enum logic [2:0] {

        LINK_RESET,
        LINK_TRAINING,
        LINK_ACTIVE,
        LINK_L1,
        LINK_RECOVERY,
        LINK_DISABLED

    } link_state_t;

    //------------------------------------------------------------
    // Header
    //------------------------------------------------------------
    typedef struct packed {

        logic [7:0] destination;
        logic [7:0] source;
        pkt_type_t  pkt_type;
        logic [11:0] reserved;

    } header_t;

    //------------------------------------------------------------
    // Control Bits
    //------------------------------------------------------------
    typedef struct packed {

        logic sop;
        logic eop;
        logic poison;
        logic retry;
        logic parity_error;
        logic reserved;

        logic [1:0] attr;

    } ctrl_t;

    //------------------------------------------------------------
    // Complete Flit
    //------------------------------------------------------------
    typedef struct packed {

        header_t                    header;
        logic [SEQ_WIDTH-1:0]       seq;
        vc_t                        vc;
        ctrl_t                      ctrl;
        logic [PAYLOAD_WIDTH-1:0]   payload;
        logic [CRC_WIDTH-1:0]       crc;

    } flit_t;

    //------------------------------------------------------------
    // Replay Entry
    //------------------------------------------------------------
    typedef struct packed {

        logic                       valid;
        logic                       acked;
        logic [SEQ_WIDTH-1:0]       seq;
        flit_t                      flit;

    } replay_entry_t;

    //------------------------------------------------------------
    // Credit State
    //------------------------------------------------------------
    typedef struct packed {

        logic [CREDIT_WIDTH-1:0] posted;
        logic [CREDIT_WIDTH-1:0] nonposted;
        logic [CREDIT_WIDTH-1:0] completion;

    } credit_t;

    //------------------------------------------------------------
    // Reorder Buffer Entry
    //------------------------------------------------------------
    typedef struct packed {

        logic                       valid;
        logic                       complete;
        logic [SEQ_WIDTH-1:0]       seq;
        flit_t                      flit;

    } rob_entry_t;

    //------------------------------------------------------------
    // TX Interface
    //------------------------------------------------------------
    typedef struct packed {

        logic                   valid;
        logic                   ready;
        logic [DATA_WIDTH-1:0]  data;
        logic                   sop;
        logic                   eop;

    } tx_if_t;

    //------------------------------------------------------------
    // RX Interface
    //------------------------------------------------------------
    typedef struct packed {

        logic                   valid;
        logic                   ready;
        logic [DATA_WIDTH-1:0]  data;
        logic                   sop;
        logic                   eop;

    } rx_if_t;

    //------------------------------------------------------------
    // Utility Functions
    //------------------------------------------------------------

    function automatic logic seq_less
    (
        input logic [SEQ_WIDTH-1:0] a,
        input logic [SEQ_WIDTH-1:0] b
    );
        seq_less = ($signed(a - b) < 0);
    endfunction

    function automatic logic seq_equal
    (
        input logic [SEQ_WIDTH-1:0] a,
        input logic [SEQ_WIDTH-1:0] b
    );
        seq_equal = (a == b);
    endfunction

    function automatic logic seq_next
    (
        input logic [SEQ_WIDTH-1:0] seq
    );
        seq_next = seq + 1'b1;
    endfunction

    function automatic logic [VC_WIDTH-1:0] vc_increment
    (
        input logic [VC_WIDTH-1:0] vc
    );
        vc_increment = vc + 1'b1;
    endfunction

    //------------------------------------------------------------
    // Default Flit
    //------------------------------------------------------------
    function automatic flit_t null_flit();

        flit_t tmp;

        tmp = '0;

        tmp.ctrl.sop = 1'b0;
        tmp.ctrl.eop = 1'b0;
        tmp.ctrl.poison = 1'b0;
        tmp.ctrl.retry = 1'b0;

        return tmp;

    endfunction

endpackage