//=============================================================================
// File        : tl_uh_slave.sv
// Description : TileLink Uncached Heavy (TL-UH) Slave / Client
// Auth        : Ashish Pradhan
// TileLink Specification : 1.8.x
//
// Supported A-channel operations
// --------------------------------
//   Get
//   PutFullData
//   PutPartialData
//   ArithmeticData
//   LogicalData
//   Intent
//
// Supported D-channel operations
// --------------------------------
//   AccessAck
//   AccessAckData
//   HintAck
//
// Architecture
// --------------------------------
//                  +------------------+
// TL A ----------->|                  |
//                  |    TL-UH SLAVE   |
//                  |                  |
//                  |  A capture       |
//                  |  validation      |
//                  |  beat engine     |
//                  |  atomic engine   |
//                  |  D response      |
//                  +--------+---------+
//                           |
//                           v
//                    Backend interface
//
// Design characteristics
// --------------------------------
//   * Single outstanding TL transaction.
//   * Multi-beat transfers supported.
//   * One backend request per beat.
//   * Full ready/valid backpressure.
//   * A-channel fields registered before processing.
//   * D-channel fields registered before presentation.
//   * D-channel remains stable while stalled.
//   * Source ID preserved.
//   * Byte masks supported.
//   * Arithmetic/logical atomics implemented as read-modify-write.
//   * No combinational A->D path.
//
// IMPORTANT
// --------------------------------
// This implementation assumes:
//   * DATA_WIDTH is a power-of-two number of bits.
//   * DATA_WIDTH >= 32.
//   * DATA_WIDTH is byte addressable.
//   * Backend operates on one DATA_WIDTH beat at a time.
//   * Atomic operations operate on one DATA_WIDTH beat.
//
//=============================================================================

module tl_uh_slave #(

  parameter int unsigned ADDR_WIDTH   = 40,
  parameter int unsigned DATA_WIDTH   = 64,
  parameter int unsigned SOURCE_WIDTH = 6,

  // Address decode:
  //
  // address_hit =
  //   ((address & ADDR_MASK) == (BASE_ADDR & ADDR_MASK))
  //
  parameter logic [ADDR_WIDTH-1:0] BASE_ADDR = '0,
  parameter logic [ADDR_WIDTH-1:0] ADDR_MASK = '0,

  // Permissions
  parameter bit ALLOW_READ   = 1'b1,
  parameter bit ALLOW_WRITE  = 1'b1,
  parameter bit ALLOW_ATOMIC = 1'b1,
  parameter bit ALLOW_INTENT = 1'b1,

  // Protocol checking
  parameter bit CHECK_ALIGNMENT  = 1'b1,
  parameter bit CHECK_MASK       = 1'b1,
  parameter bit ENABLE_ASSERTIONS = 1'b1,

  // Backend timeout
  // 0 = disabled
  parameter int unsigned BACKEND_TIMEOUT = 0

) (

  input logic clk_i,
  input logic rst_ni,

  //=========================================================================
  // TileLink A CHANNEL
  //=========================================================================

  input  logic [2:0] tl_a_opcode,

  input  logic [2:0] tl_a_param,

  input logic [SIZE_WIDTH-1:0] tl_a_size,

  input logic [SOURCE_WIDTH-1:0] tl_a_source,

  input logic [ADDR_WIDTH-1:0] tl_a_address,

  input logic [DATA_WIDTH/8-1:0] tl_a_mask,

  input logic tl_a_corrupt,

  input logic [DATA_WIDTH-1:0] tl_a_data,

  input logic tl_a_valid,
  output logic tl_a_ready,

  //=========================================================================
  // TileLink D CHANNEL
  //=========================================================================

  output logic [2:0] tl_d_opcode,

  output logic [1:0] tl_d_param,

  output logic [SIZE_WIDTH-1:0] tl_d_size,

  output logic [SOURCE_WIDTH-1:0] tl_d_source,

  output logic tl_d_sink,

  output logic tl_d_denied,

  output logic [DATA_WIDTH-1:0] tl_d_data,

  output logic tl_d_corrupt,

  output logic tl_d_valid,
  input logic tl_d_ready,

  //=========================================================================
  // BACKEND REQUEST
  //
  // One request represents one DATA_WIDTH beat.
  //=========================================================================

  output logic backend_req_valid,
  input logic backend_req_ready,

  output logic backend_req_write,

  output logic [ADDR_WIDTH-1:0] backend_req_addr,

  output logic [DATA_WIDTH-1:0] backend_req_wdata,

  output logic [DATA_WIDTH/8-1:0] backend_req_be,

  output logic [SIZE_WIDTH-1:0] backend_req_size,

  // Atomic operation information
  output logic backend_req_atomic,

  output logic [3:0] backend_req_atomic_op,

  //=========================================================================
  // BACKEND RESPONSE
  //=========================================================================

  input logic backend_rsp_valid,
  output logic backend_rsp_ready,

  input logic [DATA_WIDTH-1:0] backend_rsp_rdata,

  input logic backend_rsp_error

);

  //=========================================================================
  // CONSTANTS
  //=========================================================================

  localparam int unsigned BYTE_LANES = DATA_WIDTH / 8;

  localparam int unsigned SIZE_WIDTH =
      (BYTE_LANES <= 1) ? 1 :
      $clog2(BYTE_LANES + 1);

  localparam int unsigned BEAT_BITS =
      (BYTE_LANES <= 1) ? 1 :
      $clog2(BYTE_LANES);

  //-------------------------------------------------------------------------
  // TL A opcodes
  //-------------------------------------------------------------------------

  localparam logic [2:0] TL_PUTFULL       = 3'd0;
  localparam logic [2:0] TL_PUTPARTIAL    = 3'd1;
  localparam logic [2:0] TL_ARITHMETIC    = 3'd2;
  localparam logic [2:0] TL_LOGICAL       = 3'd3;
  localparam logic [2:0] TL_GET           = 3'd4;
  localparam logic [2:0] TL_INTENT        = 3'd5;

  //-------------------------------------------------------------------------
  // TL D opcodes
  //-------------------------------------------------------------------------

  localparam logic [2:0] TL_ACCESSACK      = 3'd0;
  localparam logic [2:0] TL_ACCESSACKDATA  = 3'd1;
  localparam logic [2:0] TL_HINTACK        = 3'd2;

  //-------------------------------------------------------------------------
  // Arithmetic atomic operations
  //
  // Encoding follows TileLink ArithmeticData parameterization.
  //-------------------------------------------------------------------------

  localparam logic [2:0] ARITH_MIN    = 3'd0;
  localparam logic [2:0] ARITH_MAX    = 3'd1;
  localparam logic [2:0] ARITH_MINU   = 3'd2;
  localparam logic [2:0] ARITH_MAXU   = 3'd3;
  localparam logic [2:0] ARITH_ADD    = 3'd4;

  //-------------------------------------------------------------------------
  // Logical atomic operations
  //-------------------------------------------------------------------------

  localparam logic [2:0] LOGIC_XOR    = 3'd0;
  localparam logic [2:0] LOGIC_OR     = 3'd1;
  localparam logic [2:0] LOGIC_AND    = 3'd2;
  localparam logic [2:0] LOGIC_SWAP   = 3'd3;

  //=========================================================================
  // State machine
  //=========================================================================

  typedef enum logic [3:0] {

    ST_IDLE             = 4'd0,

    ST_VALIDATE         = 4'd1,

    ST_WRITE_REQ        = 4'd2,

    ST_WRITE_RESP       = 4'd3,

    ST_READ_REQ         = 4'd4,

    ST_READ_WAIT        = 4'd5,

    ST_ATOMIC_READ_REQ  = 4'd6,

    ST_ATOMIC_READ_WAIT = 4'd7,

    ST_ATOMIC_WRITE_REQ = 4'd8,

    ST_ATOMIC_RESP      = 4'd9,

    ST_HINT_RESP        = 4'd10,

    ST_ERROR_RESP       = 4'd11,

    ST_D_RESP            = 4'd12

  } state_t;

  state_t state_q, state_d;

  //=========================================================================
  // REQUEST REGISTERS
  //=========================================================================

  logic [2:0] a_opcode_q;
  logic [2:0] a_param_q;

  logic [SIZE_WIDTH-1:0] a_size_q;

  logic [SOURCE_WIDTH-1:0] a_source_q;

  logic [ADDR_WIDTH-1:0] a_address_q;

  logic [DATA_WIDTH/8-1:0] a_mask_q;

  logic [DATA_WIDTH-1:0] a_data_q;

  logic a_corrupt_q;

  //=========================================================================
  // TRANSACTION PARAMETERS
  //=========================================================================

  logic [SIZE_WIDTH-1:0] bytes_per_beat;

  logic [SIZE_WIDTH-1:0] total_beats;

  logic [SIZE_WIDTH-1:0] beat_index_q;

  logic [ADDR_WIDTH-1:0] beat_address_q;

  //=========================================================================
  // ATOMIC STATE
  //=========================================================================

  logic [DATA_WIDTH-1:0] atomic_old_data_q;

  logic [DATA_WIDTH-1:0] atomic_new_data_q;

  logic [2:0] atomic_param_q;

  //=========================================================================
  // BACKEND RESPONSE
  //=========================================================================

  logic [DATA_WIDTH-1:0] backend_rdata_q;
  logic backend_error_q;

  //=========================================================================
  // D RESPONSE
  //=========================================================================

  logic [2:0] d_opcode_q;
  logic [1:0] d_param_q;

  logic [SOURCE_WIDTH-1:0] d_source_q;

  logic [DATA_WIDTH-1:0] d_data_q;

  logic d_denied_q;
  logic d_corrupt_q;

  //=========================================================================
  // TIMEOUT
  //=========================================================================

  localparam int unsigned TIMEOUT_WIDTH =
      (BACKEND_TIMEOUT <= 1)
      ? 1
      : $clog2(BACKEND_TIMEOUT + 1);

  logic [TIMEOUT_WIDTH-1:0] backend_timeout_q;

  //=========================================================================
  // CLASSIFICATION
  //=========================================================================

  logic is_get;
  logic is_put_full;
  logic is_put_partial;
  logic is_arithmetic;
  logic is_logical;
  logic is_intent;

  logic is_atomic;

  logic opcode_supported;
  logic address_hit;
  logic access_allowed;
  logic size_valid;
  logic alignment_ok;
  logic mask_valid;

  logic request_valid;

  //=========================================================================
  // HANDSHAKES
  //=========================================================================

  logic a_fire;
  logic d_fire;

  logic backend_req_fire;
  logic backend_rsp_fire;

  assign a_fire =
      tl_a_valid && tl_a_ready;

  assign d_fire =
      tl_d_valid && tl_d_ready;

  assign backend_req_fire =
      backend_req_valid &&
      backend_req_ready;

  assign backend_rsp_fire =
      backend_rsp_valid &&
      backend_rsp_ready;

  //=========================================================================
  // A OPCODE CLASSIFICATION
  //=========================================================================

  always_comb begin

    is_get        = 1'b0;
    is_put_full   = 1'b0;
    is_put_partial = 1'b0;
    is_arithmetic = 1'b0;
    is_logical    = 1'b0;
    is_intent     = 1'b0;

    unique case (a_opcode_q)

      TL_GET:
        is_get = 1'b1;

      TL_PUTFULL:
        is_put_full = 1'b1;

      TL_PUTPARTIAL:
        is_put_partial = 1'b1;

      TL_ARITHMETIC:
        is_arithmetic = 1'b1;

      TL_LOGICAL:
        is_logical = 1'b1;

      TL_INTENT:
        is_intent = 1'b1;

      default:
        begin
          is_get         = 1'b0;
          is_put_full    = 1'b0;
          is_put_partial = 1'b0;
          is_arithmetic  = 1'b0;
          is_logical     = 1'b0;
          is_intent      = 1'b0;
        end

    endcase

  end

  assign is_atomic =
      is_arithmetic ||
      is_logical;

  assign opcode_supported =
       is_get
    || is_put_full
    || is_put_partial
    || is_arithmetic
    || is_logical
    || is_intent;

  //=========================================================================
  // ADDRESS DECODE
  //=========================================================================

  always_comb begin

    if (ADDR_MASK == '0)

      address_hit = 1'b1;

    else

      address_hit =
          ((a_address_q & ADDR_MASK) ==
           (BASE_ADDR  & ADDR_MASK));

  end

  //=========================================================================
  // ACCESS CONTROL
  //=========================================================================

  always_comb begin

    access_allowed = 1'b0;

    if (is_get)

      access_allowed = ALLOW_READ;

    else if (is_put_full || is_put_partial)

      access_allowed = ALLOW_WRITE;

    else if (is_atomic)

      access_allowed = ALLOW_ATOMIC;

    else if (is_intent)

      access_allowed = ALLOW_INTENT;

  end

  //=========================================================================
  // SIZE VALIDATION
  //=========================================================================

  always_comb begin

    size_valid = 1'b0;

    //
    // A transfer may contain multiple DATA_WIDTH beats.
    //
    // Legal TL size values must describe at least one beat and may
    // represent powers of two.
    //

    if (a_size_q <= ADDR_WIDTH)

      size_valid = 1'b1;

  end

  //=========================================================================
  // BYTES / BEATS CALCULATION
  //=========================================================================

  always_comb begin

    if (a_size_q >= SIZE_WIDTH)

      bytes_per_beat = BYTE_LANES;

    else

      bytes_per_beat =
          (1 << a_size_q);

  end

  //=========================================================================
  // TOTAL BEAT COUNT
  //=========================================================================

  always_comb begin

    if (a_size_q <= $clog2(BYTE_LANES))

      total_beats = 1;

    else

      total_beats =
          (1 << (a_size_q - $clog2(BYTE_LANES)));

  end

  //=========================================================================
  // ALIGNMENT
  //=========================================================================

  always_comb begin

    alignment_ok = 1'b1;

    if (CHECK_ALIGNMENT) begin

      if (a_size_q != '0) begin

        alignment_ok =
          ((a_address_q &
            (({{(ADDR_WIDTH-1){1'b0}},1'b1}
              << a_size_q) - 1'b1))
            == '0);

      end

    end

  end

  //=========================================================================
  // MASK CHECK
  //=========================================================================

  always_comb begin

    mask_valid = 1'b1;

    if (CHECK_MASK) begin

      if (a_mask_q == '0)

        mask_valid = 1'b0;

    end

  end

  //=========================================================================
  // REQUEST VALIDATION
  //=========================================================================

  always_comb begin

    request_valid =
         opcode_supported
      && address_hit
      && access_allowed
      && size_valid
      && alignment_ok
      && mask_valid
      && !a_corrupt_q;

  end

  //=========================================================================
  // A READY
  //=========================================================================

  always_comb begin

    tl_a_ready = 1'b0;

    if (state_q == ST_IDLE)

      tl_a_ready = 1'b1;

  end

  //=========================================================================
  // STATE MACHINE
  //=========================================================================

  always_comb begin

    state_d = state_q;

    unique case (state_q)

      //=======================================================================
      // IDLE
      //=======================================================================

      ST_IDLE: begin

        if (a_fire)

          state_d = ST_VALIDATE;

      end

      //=======================================================================
      // VALIDATE
      //=======================================================================

      ST_VALIDATE: begin

        if (!request_valid)

          state_d = ST_ERROR_RESP;

        else if (is_intent)

          state_d = ST_HINT_RESP;

        else if (is_get)

          state_d = ST_READ_REQ;

        else if (is_atomic)

          state_d = ST_ATOMIC_READ_REQ;

        else

          state_d = ST_WRITE_REQ;

      end

      //=======================================================================
      // NORMAL WRITE
      //=======================================================================

      ST_WRITE_REQ: begin

        if (backend_req_fire)

          state_d = ST_WRITE_RESP;

      end

      ST_WRITE_RESP: begin

        state_d = ST_D_RESP;
      end

      //=======================================================================
      // NORMAL READ
      //=======================================================================

      ST_READ_REQ: begin

        if (backend_req_fire)

          state_d = ST_READ_WAIT;

      end

      ST_READ_WAIT: begin

        if (backend_rsp_fire)

          state_d = ST_D_RESP;

      end

      //=======================================================================
      // ATOMIC READ
      //=======================================================================

      ST_ATOMIC_READ_REQ: begin

        if (backend_req_fire)

          state_d = ST_ATOMIC_READ_WAIT;

      end

      ST_ATOMIC_READ_WAIT: begin

        if (backend_rsp_fire)

          state_d = ST_ATOMIC_WRITE_REQ;

      end

      //=======================================================================
      // ATOMIC WRITE
      //=======================================================================

      ST_ATOMIC_WRITE_REQ: begin

        if (backend_req_fire)

          state_d = ST_ATOMIC_RESP;

      end

      ST_ATOMIC_RESP: begin

        state_d = ST_D_RESP;
      end

      //=======================================================================
      // HINT
      //=======================================================================

      ST_HINT_RESP: begin

        state_d = ST_D_RESP;
      end

      //=======================================================================
      // ERROR
      //=======================================================================

      ST_ERROR_RESP: begin

        state_d = ST_D_RESP;
      end

      //=======================================================================
      // D RESPONSE
      //=======================================================================

      ST_D_RESP: begin

        if (d_fire)

          state_d = ST_IDLE;

      end

      default:

        state_d = ST_IDLE;

    endcase

  end

  //=========================================================================
  // REQUEST CAPTURE
  //=========================================================================

  always_ff @(posedge clk_i or negedge rst_ni) begin

    if (!rst_ni) begin

      state_q <= ST_IDLE;

      a_opcode_q  <= '0;
      a_param_q   <= '0;
      a_size_q    <= '0;
      a_source_q  <= '0;
      a_address_q <= '0;
      a_mask_q    <= '0;
      a_data_q    <= '0;
      a_corrupt_q <= 1'b0;

      beat_index_q   <= '0;
      beat_address_q <= '0;

    end
    else begin

      state_q <= state_d;

      //-----------------------------------------------------------------------
      // Capture A request
      //-----------------------------------------------------------------------

      if (a_fire) begin

        a_opcode_q  <= tl_a_opcode;
        a_param_q   <= tl_a_param;
        a_size_q    <= tl_a_size;
        a_source_q  <= tl_a_source;
        a_address_q <= tl_a_address;
        a_mask_q    <= tl_a_mask;
        a_data_q    <= tl_a_data;
        a_corrupt_q <= tl_a_corrupt;

        beat_index_q   <= '0;
        beat_address_q <= tl_a_address;

      end

    end

  end

  //=========================================================================
  // BEAT ADDRESS UPDATE
  //=========================================================================
  //
  // For a multi-beat transfer, each subsequent backend access advances by
  // one DATA_WIDTH beat.
  //
  //=========================================================================

  always_ff @(posedge clk_i or negedge rst_ni) begin

    if (!rst_ni) begin

      beat_index_q   <= '0;
      beat_address_q <= '0;

    end
    else begin

      if (backend_req_fire &&
          (beat_index_q + 1 < total_beats)) begin

        beat_index_q <= beat_index_q + 1'b1;

        beat_address_q <=
            beat_address_q + BYTE_LANES;

      end

    end

  end

  //=========================================================================
  // BACKEND REQUEST
  //=========================================================================

  always_comb begin

    backend_req_valid = 1'b0;

    backend_req_write = 1'b0;

    backend_req_addr  = beat_address_q;

    backend_req_wdata = a_data_q;

    backend_req_be    = a_mask_q;

    backend_req_size  = a_size_q;

    backend_req_atomic = 1'b0;

    backend_req_atomic_op = 4'd0;

    //-----------------------------------------------------------------------
    // Normal write
    //-----------------------------------------------------------------------

    if (state_q == ST_WRITE_REQ) begin

      backend_req_valid = 1'b1;

      backend_req_write = 1'b1;

    end

    //-----------------------------------------------------------------------
    // Normal read
    //-----------------------------------------------------------------------

    else if (state_q == ST_READ_REQ) begin

      backend_req_valid = 1'b1;

      backend_req_write = 1'b0;

    end

    //-----------------------------------------------------------------------
    // Atomic read
    //-----------------------------------------------------------------------

    else if (state_q == ST_ATOMIC_READ_REQ) begin

      backend_req_valid = 1'b1;

      backend_req_write = 1'b0;

      backend_req_atomic = 1'b1;

      backend_req_atomic_op =
          {1'b0, a_param_q};

    end

    //-----------------------------------------------------------------------
    // Atomic write
    //-----------------------------------------------------------------------

    else if (state_q == ST_ATOMIC_WRITE_REQ) begin

      backend_req_valid = 1'b1;

      backend_req_write = 1'b1;

      backend_req_atomic = 1'b1;

      backend_req_atomic_op =
          {1'b0, a_param_q};

      backend_req_wdata =
          atomic_new_data_q;

    end

  end

  //=========================================================================
  // BACKEND RESPONSE READY
  //=========================================================================

  always_comb begin

    backend_rsp_ready = 1'b0;

    if ((state_q == ST_READ_WAIT) ||
        (state_q == ST_ATOMIC_READ_WAIT))

      backend_rsp_ready = 1'b1;

  end

  //=========================================================================
  // ATOMIC DATA CALCULATION
  //=========================================================================

  always_comb begin

    atomic_new_data_q = atomic_old_data_q;

    //-----------------------------------------------------------------------
    // ArithmeticData
    //-----------------------------------------------------------------------

    if (is_arithmetic) begin

      unique case (atomic_param_q)

        ARITH_MIN:

          atomic_new_data_q =
            ($signed(atomic_old_data_q) <
             $signed(a_data_q))
            ? atomic_old_data_q
            : a_data_q;

        ARITH_MAX:

          atomic_new_data_q =
            ($signed(atomic_old_data_q) >
             $signed(a_data_q))
            ? atomic_old_data_q
            : a_data_q;

        ARITH_MINU:

          atomic_new_data_q =
            (atomic_old_data_q <
             a_data_q)
            ? atomic_old_data_q
            : a_data_q;

        ARITH_MAXU:

          atomic_new_data_q =
            (atomic_old_data_q >
             a_data_q)
            ? atomic_old_data_q
            : a_data_q;

        ARITH_ADD:

          atomic_new_data_q =
            atomic_old_data_q +
            a_data_q;

        default:

          atomic_new_data_q =
            atomic_old_data_q;

      endcase

    end

    //-----------------------------------------------------------------------
    // LogicalData
    //-----------------------------------------------------------------------

    else if (is_logical) begin

      unique case (atomic_param_q)

        LOGIC_XOR:

          atomic_new_data_q =
            atomic_old_data_q ^
            a_data_q;

        LOGIC_OR:

          atomic_new_data_q =
            atomic_old_data_q |
            a_data_q;

        LOGIC_AND:

          atomic_new_data_q =
            atomic_old_data_q &
            a_data_q;

        LOGIC_SWAP:

          atomic_new_data_q =
            a_data_q;

        default:

          atomic_new_data_q =
            atomic_old_data_q;

      endcase

    end

  end

  //=========================================================================
  // ATOMIC OLD DATA CAPTURE
  //=========================================================================

  always_ff @(posedge clk_i or negedge rst_ni) begin

    if (!rst_ni) begin

      atomic_old_data_q <= '0;
      atomic_param_q    <= '0;

    end
    else begin

      if (a_fire && is_atomic)

        atomic_param_q <= a_param_q;

      if (backend_rsp_fire &&
          (state_q == ST_ATOMIC_READ_WAIT))

        atomic_old_data_q <= backend_rsp_rdata;

    end

  end

  //=========================================================================
  // D RESPONSE REGISTERS
  //=========================================================================

  always_ff @(posedge clk_i or negedge rst_ni) begin

    if (!rst_ni) begin

      d_opcode_q  <= TL_ACCESSACK;
      d_param_q   <= 2'b00;
      d_source_q  <= '0;
      d_data_q    <= '0;
      d_denied_q  <= 1'b0;
      d_corrupt_q <= 1'b0;

    end
    else begin

      //-----------------------------------------------------------------------
      // Normal read response
      //-----------------------------------------------------------------------

      if (backend_rsp_fire &&
          (state_q == ST_READ_WAIT)) begin

        d_opcode_q = TL_ACCESSACKDATA;

        d_param_q <= 2'b00;

        d_source_q <= a_source_q;

        d_data_q <= backend_rsp_rdata;

        d_denied_q <= backend_rsp_error;

        d_corrupt_q <= 1'b0;

      end

      //-----------------------------------------------------------------------
      // Atomic response
      //
      // TL atomic response returns the original value.
      //-----------------------------------------------------------------------

      else if (backend_rsp_fire &&
               (state_q == ST_ATOMIC_READ_WAIT)) begin

        d_opcode_q <= TL_ACCESSACKDATA;

        d_param_q <= 2'b00;

        d_source_q <= a_source_q;

        d_data_q <= backend_rsp_rdata;

        d_denied_q <= backend_rsp_error;

        d_corrupt_q <= 1'b0;

      end

      //-----------------------------------------------------------------------
      // Put response
      //-----------------------------------------------------------------------

      else if (state_q == ST_WRITE_RESP) begin

        d_opcode_q <= TL_ACCESSACK;

        d_param_q <= 2'b00;

        d_source_q <= a_source_q;

        d_data_q <= '0;

        d_denied_q <= 1'b0;

        d_corrupt_q <= 1'b0;

      end

      //-----------------------------------------------------------------------
      // Atomic write response
      //-----------------------------------------------------------------------

      else if (state_q == ST_ATOMIC_RESP) begin

        d_opcode_q <= TL_ACCESSACK;

        d_param_q <= 2'b00;

        d_source_q <= a_source_q;

        d_data_q <= '0;

        d_denied_q <= 1'b0;

        d_corrupt_q <= 1'b0;

      end

      //-----------------------------------------------------------------------
      // Intent
      //-----------------------------------------------------------------------

      else if (state_q == ST_HINT_RESP) begin

        d_opcode_q <= TL_HINTACK;

        d_param_q <= 2'b00;

        d_source_q <= a_source_q;

        d_data_q <= '0;

        d_denied_q <= 1'b0;

        d_corrupt_q <= 1'b0;

      end

      //-----------------------------------------------------------------------
      // Error
      //-----------------------------------------------------------------------

      else if (state_q == ST_ERROR_RESP) begin

        if (is_get || is_atomic)

          d_opcode_q <= TL_ACCESSACKDATA;

        else if (is_intent)

          d_opcode_q <= TL_HINTACK;

        else

          d_opcode_q <= TL_ACCESSACK;

        d_param_q <= 2'b00;

        d_source_q <= a_source_q;

        d_data_q <= '0;

        d_denied_q <= 1'b1;

        d_corrupt_q <= 1'b0;

      end

    end

  end

  //=========================================================================
  // D CHANNEL
  //=========================================================================

  always_comb begin

    tl_d_opcode = d_opcode_q;

    tl_d_param = d_param_q;

    tl_d_size = a_size_q;

    tl_d_source = d_source_q;

    tl_d_sink = 1'b0;

    tl_d_denied = d_denied_q;

    tl_d_data = d_data_q;

    tl_d_corrupt = d_corrupt_q;

    tl_d_valid =
         (state_q == ST_D_RESP);

  end

  //=========================================================================
  // BACKEND TIMEOUT
  //=========================================================================

  always_ff @(posedge clk_i or negedge rst_ni) begin

    if (!rst_ni) begin

      backend_timeout_q <= '0;

    end
    else begin

      if ((state_q != ST_READ_WAIT) &&
          (state_q != ST_ATOMIC_READ_WAIT)) begin

        backend_timeout_q <= '0;

      end
      else if (BACKEND_TIMEOUT != 0) begin

        if (backend_rsp_fire)

          backend_timeout_q <= '0;

        else if (backend_timeout_q < BACKEND_TIMEOUT)

          backend_timeout_q <=
              backend_timeout_q + 1'b1;

      end

    end

  end

  //=========================================================================
  // ASSERTIONS
  //=========================================================================

  generate

    if (ENABLE_ASSERTIONS) begin : gen_tl_uh_sva

      //-----------------------------------------------------------------------
      // A-channel stability
      //-----------------------------------------------------------------------

      property p_a_stable;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_a_valid && !tl_a_ready

        |->
        $stable({
          tl_a_opcode,
          tl_a_param,
          tl_a_size,
          tl_a_source,
          tl_a_address,
          tl_a_mask,
          tl_a_corrupt,
          tl_a_data
        });

      endproperty

      assert property (p_a_stable)
        else
          $error("TL-UH: A channel changed while stalled");

      //-----------------------------------------------------------------------
      // D-channel stability
      //-----------------------------------------------------------------------

      property p_d_stable;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid && !tl_d_ready

        |->
        $stable({
          tl_d_opcode,
          tl_d_param,
          tl_d_size,
          tl_d_source,
          tl_d_sink,
          tl_d_denied,
          tl_d_data,
          tl_d_corrupt
        });

      endproperty

      assert property (p_d_stable)
        else
          $error("TL-UH: D channel changed while stalled");

      //-----------------------------------------------------------------------
      // Supported A opcode
      //-----------------------------------------------------------------------

      property p_supported_opcode;

        @(posedge clk_i)
        disable iff (!rst_ni)

        a_fire |->
        (
             (tl_a_opcode == TL_GET)
          || (tl_a_opcode == TL_PUTFULL)
          || (tl_a_opcode == TL_PUTPARTIAL)
          || (tl_a_opcode == TL_ARITHMETIC)
          || (tl_a_opcode == TL_LOGICAL)
          || (tl_a_opcode == TL_INTENT)
        );

      endproperty

      assert property (p_supported_opcode)
        else
          $error("TL-UH: unsupported A opcode");

      //-----------------------------------------------------------------------
      // A param is meaningful only for atomics/intent
      //-----------------------------------------------------------------------

      property p_non_atomic_param;

        @(posedge clk_i)
        disable iff (!rst_ni)

        a_fire &&
        ((tl_a_opcode == TL_GET) ||
         (tl_a_opcode == TL_PUTFULL) ||
         (tl_a_opcode == TL_PUTPARTIAL))

        |->
        (tl_a_param == 3'b000);

      endproperty

      assert property (p_non_atomic_param)
        else
          $error("TL-UH: illegal A param");

      //-----------------------------------------------------------------------
      // A corrupt
      //-----------------------------------------------------------------------

      property p_a_corrupt;

        @(posedge clk_i)
        disable iff (!rst_ni)

        a_fire && request_valid
        |->
        !tl_a_corrupt;

      endproperty

      assert property (p_a_corrupt)
        else
          $error("TL-UH: corrupt A request");

      //-----------------------------------------------------------------------
      // D source preservation
      //-----------------------------------------------------------------------

      property p_source_preserved;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid
        |->
        (tl_d_source == a_source_q);

      endproperty

      assert property (p_source_preserved)
        else
          $error("TL-UH: D source mismatch");

      //-----------------------------------------------------------------------
      // Get response
      //-----------------------------------------------------------------------

      property p_get_response;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid &&
        (a_opcode_q == TL_GET)

        |->
        (tl_d_opcode == TL_ACCESSACKDATA);

      endproperty

      assert property (p_get_response)
        else
          $error("TL-UH: Get must return AccessAckData");

      //-----------------------------------------------------------------------
      // Put response
      //-----------------------------------------------------------------------

      property p_put_response;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid &&
        ((a_opcode_q == TL_PUTFULL) ||
         (a_opcode_q == TL_PUTPARTIAL))

        |->
        (tl_d_opcode == TL_ACCESSACK);

      endproperty

      assert property (p_put_response)
        else
          $error("TL-UH: Put must return AccessAck");

      //-----------------------------------------------------------------------
      // Intent response
      //-----------------------------------------------------------------------

      property p_intent_response;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid &&
        (a_opcode_q == TL_INTENT)

        |->
        (tl_d_opcode == TL_HINTACK);

      endproperty

      assert property (p_intent_response)
        else
          $error("TL-UH: Intent must return HintAck");

      //-----------------------------------------------------------------------
      // D param
      //-----------------------------------------------------------------------

      property p_d_param_zero;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid
        |->
        (tl_d_param == 2'b00);

      endproperty

      assert property (p_d_param_zero)
        else
          $error("TL-UH: illegal D param");

      //-----------------------------------------------------------------------
      // D sink
      //-----------------------------------------------------------------------

      property p_d_sink_zero;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid
        |->
        (tl_d_sink == 1'b0);

      endproperty

      assert property (p_d_sink_zero)
        else
          $error("TL-UH: D sink must be zero");

      //-----------------------------------------------------------------------
      // Backend request only in legal states
      //-----------------------------------------------------------------------

      property p_backend_request_state;

        @(posedge clk_i)
        disable iff (!rst_ni)

        backend_req_valid
        |->
        (
             (state_q == ST_WRITE_REQ)
          || (state_q == ST_READ_REQ)
          || (state_q == ST_ATOMIC_READ_REQ)
          || (state_q == ST_ATOMIC_WRITE_REQ)
        );

      endproperty

      assert property (p_backend_request_state)
        else
          $error("TL-UH: backend request in illegal state");

      //-----------------------------------------------------------------------
      // Backend response only accepted in response states
      //-----------------------------------------------------------------------

      property p_backend_response_state;

        @(posedge clk_i)
        disable iff (!rst_ni)

        backend_rsp_fire
        |->
        (
             (state_q == ST_READ_WAIT)
          || (state_q == ST_ATOMIC_READ_WAIT)
        );

      endproperty

      assert property (p_backend_response_state)
        else
          $error("TL-UH: backend response in illegal state");

      //-----------------------------------------------------------------------
      // D response must be stable while stalled
      //-----------------------------------------------------------------------

      property p_d_hold;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid && !tl_d_ready
        |=> tl_d_valid;

      endproperty

      assert property (p_d_hold)
        else
          $error("TL-UH: D valid dropped while stalled");

      //-----------------------------------------------------------------------
      // No response before accepted A
      //-----------------------------------------------------------------------

      property p_no_spurious_d;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid
        |->
        (
          state_q == ST_D_RESP
        );

      endproperty

      assert property (p_no_spurious_d)
        else
          $error("TL-UH: spurious D response");

      //-----------------------------------------------------------------------
      // Timeout
      //-----------------------------------------------------------------------

      if (BACKEND_TIMEOUT != 0) begin : gen_backend_timeout_sva

        property p_backend_timeout;

          @(posedge clk_i)
          disable iff (!rst_ni)

          ((state_q == ST_READ_WAIT) ||
           (state_q == ST_ATOMIC_READ_WAIT))

          |->
          (backend_timeout_q < BACKEND_TIMEOUT);

        endproperty

        assert property (p_backend_timeout)
          else
            $error("TL-UH: backend timeout");

      end

    end

  endgenerate

endmodule
