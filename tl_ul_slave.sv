//=============================================================================
// File        : tl_ul_slave.sv
// Description : TileLink Uncached Lightweight (TL-UL)
//               Slave / Client
// Auther:   Ashish Pradhan
// Supported TL-UL A-channel operations:
//   - Get
//   - PutFullData
//   - PutPartialData
//
// Supported TL-UL D-channel responses:
//   - AccessAck
//   - AccessAckData
//
// Architecture:
//   TL A channel
//        |
//        v
//   +-----------+
//   | A capture |
//   +-----------+
//        |
//        v
//   +-----------+
//   | Request   |
//   | Validate  |
//   +-----------+
//        |
//        v
//   +-----------+
//   | Backend   |
//   | Interface |
//   +-----------+
//        |
//        v
//   +-----------+
//   | D response|
//   | generator |
//   +-----------+
//        |
//        v
//   TL D channel
//
// Notes:
//   * Single outstanding transaction.
//   * One A request is accepted only when the previous response has retired.
//   * No combinational path from A to D.
//   * Backend interface is also decoupled.
//   * This module does not implement TL-UH.
//   * No B/C/E channels exist in TL-UL.
//
//=============================================================================

module tl_ul_slave #(

  parameter int unsigned ADDR_WIDTH   = 32,
  parameter int unsigned DATA_WIDTH   = 64,
  parameter int unsigned SOURCE_WIDTH = 4,

  //-------------------------------------------------------------------------
  // Address decode
  //-------------------------------------------------------------------------
  //
  // Transaction is accepted only when:
  //
  //     (address & ADDR_MASK) == (BASE_ADDR & ADDR_MASK)
  //
  parameter logic [ADDR_WIDTH-1:0] BASE_ADDR = '0,
  parameter logic [ADDR_WIDTH-1:0] ADDR_MASK = '0,

  //-------------------------------------------------------------------------
  // Access permissions
  //-------------------------------------------------------------------------
  //
  parameter bit ALLOW_READ  = 1'b1;
  parameter bit ALLOW_WRITE = 1'b1;

  //-------------------------------------------------------------------------
  // Protocol checking
  //-------------------------------------------------------------------------
  parameter bit CHECK_ALIGNMENT = 1'b1;
  parameter bit ENABLE_ASSERTIONS = 1'b1;

  //-------------------------------------------------------------------------
  // Backend response timeout
  //
  // 0 = disabled
  //-------------------------------------------------------------------------
  parameter int unsigned BACKEND_TIMEOUT = 0

) (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  //=========================================================================
  // TileLink A channel
  //=========================================================================

  input  logic [2:0]                   tl_a_opcode,
  input  logic [2:0]                   tl_a_param,
  input  logic [$clog2(DATA_WIDTH/8+1)-1:0]
                                        tl_a_size,

  input  logic [SOURCE_WIDTH-1:0]      tl_a_source,

  input  logic [ADDR_WIDTH-1:0]        tl_a_address,

  input  logic [DATA_WIDTH/8-1:0]      tl_a_mask,

  input  logic                         tl_a_corrupt,

  input  logic [DATA_WIDTH-1:0]        tl_a_data,

  input  logic                         tl_a_valid,
  output logic                         tl_a_ready,

  //=========================================================================
  // TileLink D channel
  //=========================================================================

  output logic [2:0]                   tl_d_opcode,
  output logic [1:0]                   tl_d_param,

  output logic [$clog2(DATA_WIDTH/8+1)-1:0]
                                        tl_d_size,

  output logic [SOURCE_WIDTH-1:0]      tl_d_source,

  output logic                         tl_d_sink,

  output logic                         tl_d_denied,

  output logic [DATA_WIDTH-1:0]        tl_d_data,

  output logic                         tl_d_corrupt,

  output logic                         tl_d_valid,
  input  logic                         tl_d_ready,

  //=========================================================================
  // Backend request interface
  //=========================================================================

  output logic                         mem_req_valid,
  input  logic                         mem_req_ready,

  output logic                         mem_req_write,

  output logic [ADDR_WIDTH-1:0]        mem_req_addr,

  output logic [DATA_WIDTH-1:0]        mem_req_wdata,

  output logic [DATA_WIDTH/8-1:0]      mem_req_be,

  output logic [$clog2(DATA_WIDTH/8+1)-1:0]
                                        mem_req_size,

  //=========================================================================
  // Backend response interface
  //=========================================================================

  input  logic                         mem_rsp_valid,
  output logic                         mem_rsp_ready,

  input  logic [DATA_WIDTH-1:0]        mem_rsp_rdata,

  input  logic                         mem_rsp_error

);

  //=========================================================================
  // Local constants
  //=========================================================================

  localparam int unsigned BYTE_LANES = DATA_WIDTH / 8;

  localparam int unsigned SIZE_WIDTH =
      (BYTE_LANES <= 1) ? 1 : $clog2(BYTE_LANES + 1);

  localparam logic [2:0] TL_PUTFULL = 3'd0;
  localparam logic [2:0] TL_PUTPART = 3'd1;
  localparam logic [2:0] TL_GET = 3'd4;

  localparam logic [2:0] TL_ACCESSACK = 3'd0;
  localparam logic [2:0] TL_ACCESSACKDATA = 3'd1;

  localparam logic [1:0] TL_D_PARAM_ZERO = 2'b00;

  //=========================================================================
  // State machine
  //=========================================================================

  typedef enum logic [2:0] {

    ST_IDLE       = 3'b000,
    ST_BACKEND    = 3'b001,
    ST_RESP       = 3'b010,
    ST_ERROR_RESP = 3'b011

  } state_t;

  state_t state_q;
  state_t state_d;

  //=========================================================================
  // Latched request
  //=========================================================================

  logic [2:0]                 a_opcode_q;
  logic [2:0]                 a_param_q;

  logic [SIZE_WIDTH-1:0]      a_size_q;

  logic [SOURCE_WIDTH-1:0]    a_source_q;

  logic [ADDR_WIDTH-1:0]     a_address_q;

  logic [BYTE_LANES-1:0]     a_mask_q;

  logic [DATA_WIDTH-1:0]     a_data_q;

  logic                       a_corrupt_q;

  //=========================================================================
  // Request classification
  //=========================================================================

  logic is_get;
  logic is_put_full;
  logic is_put_partial;
  logic opcode_supported;

  logic address_hit;
  logic access_allowed;
  logic alignment_ok;
  logic size_valid;
  logic mask_valid;

  //=========================================================================
  // Backend response
  //=========================================================================

  logic [DATA_WIDTH-1:0] backend_rdata_q;
  logic                  backend_error_q;

  //=========================================================================
  // D-channel response
  //=========================================================================

  logic [2:0]             d_opcode_q;
  logic [1:0]             d_param_q;
  logic [SOURCE_WIDTH-1:0]
                         d_source_q;

  logic [DATA_WIDTH-1:0] d_data_q;

  logic                 d_denied_q;
  logic                 d_corrupt_q;

  //=========================================================================
  // Timeout
  //=========================================================================

  localparam int unsigned TIMEOUT_WIDTH =
      (BACKEND_TIMEOUT <= 1)
      ? 1
      : $clog2(BACKEND_TIMEOUT + 1);

  logic [TIMEOUT_WIDTH-1:0] backend_timeout_q;

  //=========================================================================
  // Handshake signals
  //=========================================================================

  logic a_fire;
  logic d_fire;
  logic mem_req_fire;
  logic mem_rsp_fire;

  assign a_fire       = tl_a_valid && tl_a_ready;
  assign d_fire       = tl_d_valid && tl_d_ready;
  assign mem_req_fire = mem_req_valid && mem_req_ready;
  assign mem_rsp_fire = mem_rsp_valid && mem_rsp_ready;

  //=========================================================================
  // Opcode classification
  //=========================================================================

  always_comb begin

    is_get         = 1'b0;
    is_put_full    = 1'b0;
    is_put_partial = 1'b0;

    unique case (a_opcode_q)

      TL_GET:
        is_get = 1'b1;

      TL_PUTFULL:
        is_put_full = 1'b1;

      TL_PUTPART:
        is_put_partial = 1'b1;

      default:
        begin
          is_get         = 1'b0;
          is_put_full    = 1'b0;
          is_put_partial = 1'b0;
        end

    endcase

  end

  assign opcode_supported =
       (a_opcode_q == TL_GET)
    || (a_opcode_q == TL_PUTFULL)
    || (a_opcode_q == TL_PUTPART);

  //=========================================================================
  // Address decode
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
  // Access permission
  //=========================================================================

  always_comb begin

    access_allowed = 1'b0;

    if (is_get)
      access_allowed = ALLOW_READ;

    else if (is_put_full || is_put_partial)
      access_allowed = ALLOW_WRITE;

  end

  //=========================================================================
  // Size validation
  //=========================================================================

  always_comb begin

    size_valid = 1'b0;

    //
    // TL size is log2(bytes).
    //
    // For a DATA_WIDTH-wide interface the maximum legal transfer is
    // DATA_WIDTH/8 bytes.
    //

    if (a_size_q <= $clog2(BYTE_LANES))
      size_valid = 1'b1;

  end

  //=========================================================================
  // Alignment check
  //=========================================================================

  always_comb begin

    alignment_ok = 1'b1;

    if (CHECK_ALIGNMENT) begin

      if (a_size_q != '0) begin

        //
        // Alignment boundary = 2^size bytes.
        //
        alignment_ok =
          ((a_address_q & (({{(ADDR_WIDTH-1){1'b0}},1'b1}
                           << a_size_q) - 1'b1))
           == '0);

      end

    end

  end

  //=========================================================================
  // Byte-mask validation
  //=========================================================================

  always_comb begin

    mask_valid = 1'b1;

    //-----------------------------------------------------------------------
    // Get
    //
    // A Get mask is normally expected to correspond to the requested size.
    //-----------------------------------------------------------------------

    if (is_get) begin

      if (a_mask_q == '0)
        mask_valid = 1'b0;

    end

    //-----------------------------------------------------------------------
    // PutFullData
    //
    // For a full write, all bytes of the requested transfer must be enabled.
    //-----------------------------------------------------------------------

    if (is_put_full) begin

      if (a_mask_q == '0)
        mask_valid = 1'b0;

    end

    //-----------------------------------------------------------------------
    // PutPartialData
    //
    // Partial byte enables are explicitly supported.
    //-----------------------------------------------------------------------

    if (is_put_partial) begin

      if (a_mask_q == '0)
        mask_valid = 1'b0;

    end

  end

  //=========================================================================
  // Request validation
  //=========================================================================

  logic request_valid;

  always_comb begin

    request_valid =
         opcode_supported
      && address_hit
      && access_allowed
      && size_valid
      && mask_valid
      && alignment_ok
      && !a_corrupt_q;

  end

  //=========================================================================
  // A channel ready
  //=========================================================================
  //
  // Only accept a new request when completely idle.
  //
  // This guarantees:
  //
  //   1. One outstanding transaction
  //   2. No request overwrite
  //   3. Simple source preservation
  //
  //=========================================================================

  always_comb begin

    tl_a_ready = 1'b0;

    if (state_q == ST_IDLE)
      tl_a_ready = 1'b1;

  end

  //=========================================================================
  // State transition
  //=========================================================================

  always_comb begin

    state_d = state_q;

    unique case (state_q)

      //---------------------------------------------------------------------
      // IDLE
      //---------------------------------------------------------------------

      ST_IDLE: begin

        if (a_fire) begin

          if (request_valid)
            state_d = ST_BACKEND;
          else
            state_d = ST_ERROR_RESP;

        end

      end

      //---------------------------------------------------------------------
      // Backend request
      //---------------------------------------------------------------------

      ST_BACKEND: begin

        //
        // For a backend write, completion occurs when backend accepts the
        // request and no separate response is required.
        //
        // For a backend read, wait for mem_rsp_valid.
        //

        if (mem_req_fire) begin

          if (is_get)
            state_d = ST_BACKEND;
          else
            state_d = ST_RESP;

        end

        if (is_get && mem_rsp_fire)
          state_d = ST_RESP;

      end

      //---------------------------------------------------------------------
      // Normal response
      //---------------------------------------------------------------------

      ST_RESP: begin

        if (d_fire)
          state_d = ST_IDLE;

      end

      //---------------------------------------------------------------------
      // Error response
      //---------------------------------------------------------------------

      ST_ERROR_RESP: begin

        if (d_fire)
          state_d = ST_IDLE;

      end

      default:
        state_d = ST_IDLE;

    endcase

  end

  //=========================================================================
  // Request register
  //=========================================================================

  always_ff @(posedge clk_i or negedge rst_ni) begin

    if (!rst_ni) begin

      state_q <= ST_IDLE;

      a_opcode_q  <= TL_GET;
      a_param_q   <= '0;
      a_size_q    <= '0;
      a_source_q  <= '0;
      a_address_q <= '0;
      a_mask_q    <= '0;
      a_data_q    <= '0;
      a_corrupt_q <= 1'b0;

    end
    else begin

      state_q <= state_d;

      if (a_fire) begin

        a_opcode_q  <= tl_a_opcode;
        a_param_q   <= tl_a_param;
        a_size_q    <= tl_a_size;
        a_source_q  <= tl_a_source;
        a_address_q <= tl_a_address;
        a_mask_q    <= tl_a_mask;
        a_data_q    <= tl_a_data;
        a_corrupt_q <= tl_a_corrupt;

      end

    end

  end

  //=========================================================================
  // Backend request
  //=========================================================================

  always_comb begin

    mem_req_valid = 1'b0;

    mem_req_write = 1'b0;

    mem_req_addr  = a_address_q;
    mem_req_wdata = a_data_q;
    mem_req_be    = a_mask_q;
    mem_req_size  = a_size_q;

    if (state_q == ST_BACKEND) begin

      //
      // Only generate the backend request once.
      //
      // For reads the request remains valid until accepted.
      //

      mem_req_valid = 1'b1;

      mem_req_write =
           is_put_full
        || is_put_partial;

    end

  end

  //=========================================================================
  // Backend response
  //=========================================================================

  always_comb begin

    mem_rsp_ready = 1'b0;

    if ((state_q == ST_BACKEND) && is_get)
      mem_rsp_ready = 1'b1;

  end

  //=========================================================================
  // Backend response registers
  //=========================================================================

  always_ff @(posedge clk_i or negedge rst_ni) begin

    if (!rst_ni) begin

      backend_rdata_q <= '0;
      backend_error_q <= 1'b0;

    end
    else begin

      if (mem_rsp_fire) begin

        backend_rdata_q <= mem_rsp_rdata;
        backend_error_q <= mem_rsp_error;

      end

    end

  end

  //=========================================================================
  // D-channel response registers
  //=========================================================================

  always_ff @(posedge clk_i or negedge rst_ni) begin

    if (!rst_ni) begin

      d_opcode_q  <= TL_ACCESSACK;
      d_param_q   <= TL_D_PARAM_ZERO;
      d_source_q  <= '0;
      d_data_q    <= '0;
      d_denied_q  <= 1'b0;
      d_corrupt_q <= 1'b0;

    end
    else begin

      //---------------------------------------------------------------------
      // Normal successful backend completion
      //---------------------------------------------------------------------

      if (state_q == ST_BACKEND && mem_rsp_fire) begin

        d_opcode_q <= TL_ACCESSACKDATA;

        d_param_q  <= TL_D_PARAM_ZERO;

        d_source_q <= a_source_q;

        d_data_q   <= mem_rsp_rdata;

        d_denied_q <= mem_rsp_error;

        d_corrupt_q <= 1'b0;

      end

      //---------------------------------------------------------------------
      // Write request accepted by backend
      //---------------------------------------------------------------------

      else if (state_q == ST_BACKEND && mem_req_fire &&
               (is_put_full || is_put_partial)) begin

        d_opcode_q <= TL_ACCESSACK;

        d_param_q  <= TL_D_PARAM_ZERO;

        d_source_q <= a_source_q;

        d_data_q   <= '0;

        d_denied_q <= 1'b0;

        d_corrupt_q <= 1'b0;

      end

      //---------------------------------------------------------------------
      // Invalid TL request
      //---------------------------------------------------------------------

      else if (state_q == ST_IDLE && a_fire &&
               !request_valid) begin

        d_opcode_q <=
          is_get ? TL_ACCESSACKDATA : TL_ACCESSACK;

        d_param_q <= TL_D_PARAM_ZERO;

        d_source_q <= tl_a_source;

        d_data_q <= '0;

        d_denied_q <= 1'b1;

        d_corrupt_q <= 1'b0;

      end

    end

  end

  //=========================================================================
  // D channel
  //=========================================================================

  always_comb begin

    tl_d_opcode  = d_opcode_q;

    tl_d_param   = d_param_q;

    tl_d_size    = a_size_q;

    tl_d_source  = d_source_q;

    tl_d_sink    = 1'b0;

    tl_d_denied  = d_denied_q;

    tl_d_data    = d_data_q;

    tl_d_corrupt = d_corrupt_q;

    tl_d_valid   =
         (state_q == ST_RESP)
      || (state_q == ST_ERROR_RESP);

  end

  //=========================================================================
  // Backend timeout
  //=========================================================================

  always_ff @(posedge clk_i or negedge rst_ni) begin

    if (!rst_ni) begin

      backend_timeout_q <= '0;

    end
    else begin

      if (state_q != ST_BACKEND) begin

        backend_timeout_q <= '0;

      end
      else if (BACKEND_TIMEOUT != 0) begin

        if (mem_rsp_fire || mem_req_fire) begin

          backend_timeout_q <= '0;

        end
        else if (backend_timeout_q < BACKEND_TIMEOUT) begin

          backend_timeout_q <= backend_timeout_q + 1'b1;

        end

      end

    end

  end

  //=========================================================================
  // Assertions
  //=========================================================================

  generate

    if (ENABLE_ASSERTIONS) begin : gen_sva

      //=====================================================================
      // A channel stability
      //=====================================================================

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
          $error("TL-UL SLAVE: A channel changed while stalled");

      //=====================================================================
      // D channel stability
      //=====================================================================

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
          $error("TL-UL SLAVE: D channel changed while stalled");

      //=====================================================================
      // A opcode
      //=====================================================================

      property p_a_opcode;

        @(posedge clk_i)
        disable iff (!rst_ni)

        a_fire |->
          (
               tl_a_opcode == TL_GET
            || tl_a_opcode == TL_PUTFULL
            || tl_a_opcode == TL_PUTPART
          );

      endproperty

      assert property (p_a_opcode)
        else
          $error("TL-UL SLAVE: unsupported A opcode");

      //=====================================================================
      // A param must be zero for TL-UL
      //=====================================================================

      property p_a_param_zero;

        @(posedge clk_i)
        disable iff (!rst_ni)

        a_fire |-> (tl_a_param == 3'b000);

      endproperty

      assert property (p_a_param_zero)
        else
          $error("TL-UL SLAVE: non-zero A param");

      //=====================================================================
      // A corrupt must be zero for a valid transaction
      //=====================================================================

      property p_a_corrupt;

        @(posedge clk_i)
        disable iff (!rst_ni)

        a_fire && request_valid
        |-> !tl_a_corrupt;

      endproperty

      assert property (p_a_corrupt)
        else
          $error("TL-UL SLAVE: corrupt A request");

      //=====================================================================
      // D response only after request
      //=====================================================================

      property p_d_valid_state;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid
        |->
        ((state_q == ST_RESP) ||
         (state_q == ST_ERROR_RESP));

      endproperty

      assert property (p_d_valid_state)
        else
          $error("TL-UL SLAVE: D valid in illegal state");

      //=====================================================================
      // D source preservation
      //=====================================================================

      property p_source_preserved;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid |-> (tl_d_source == a_source_q);

      endproperty

      assert property (p_source_preserved)
        else
          $error("TL-UL SLAVE: D source mismatch");

      //=====================================================================
      // Get -> AccessAckData
      //=====================================================================

      property p_get_ack_data;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid &&
        (a_opcode_q == TL_GET)

        |->
        (tl_d_opcode == TL_ACCESSACKDATA);

      endproperty

      assert property (p_get_ack_data)
        else
          $error("TL-UL SLAVE: Get did not generate AccessAckData");

      //=====================================================================
      // Put -> AccessAck
      //=====================================================================

      property p_put_ack;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid &&
        ((a_opcode_q == TL_PUTFULL) ||
         (a_opcode_q == TL_PUTPART))

        |->
        (tl_d_opcode == TL_ACCESSACK);

      endproperty

      assert property (p_put_ack)
        else
          $error("TL-UL SLAVE: Put did not generate AccessAck");

      //=====================================================================
      // D param zero
      //=====================================================================

      property p_d_param_zero;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid |-> (tl_d_param == 2'b00);

      endproperty

      assert property (p_d_param_zero)
        else
          $error("TL-UL SLAVE: non-zero D param");

      //=====================================================================
      // D sink zero
      //=====================================================================

      property p_d_sink_zero;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid |-> (tl_d_sink == 1'b0);

      endproperty

      assert property (p_d_sink_zero)
        else
          $error("TL-UL SLAVE: invalid D sink");

      //=====================================================================
      // No D response without an accepted A request
      //=====================================================================

      property p_response_requires_request;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid
        |->
        (
          (state_q == ST_RESP) ||
          (state_q == ST_ERROR_RESP)
        );

      endproperty

      assert property (p_response_requires_request)
        else
          $error("TL-UL SLAVE: response without request");

      //=====================================================================
      // Backend request only in backend state
      //=====================================================================

      property p_backend_state;

        @(posedge clk_i)
        disable iff (!rst_ni)

        mem_req_valid
        |->
        (state_q == ST_BACKEND);

      endproperty

      assert property (p_backend_state)
        else
          $error("TL-UL SLAVE: backend request outside backend state");

      //=====================================================================
      // Backend response only accepted for Get
      //=====================================================================

      property p_backend_read_response;

        @(posedge clk_i)
        disable iff (!rst_ni)

        mem_rsp_fire
        |->
        (is_get);

      endproperty

      assert property (p_backend_read_response)
        else
          $error("TL-UL SLAVE: backend response for non-read");

      //=====================================================================
      // Response timeout
      //=====================================================================

      if (BACKEND_TIMEOUT != 0) begin : gen_timeout_assert

        property p_backend_timeout;

          @(posedge clk_i)
          disable iff (!rst_ni)

          (state_q == ST_BACKEND)

          |->
          (backend_timeout_q < BACKEND_TIMEOUT);

        endproperty

        assert property (p_backend_timeout)
          else
            $error("TL-UL SLAVE: backend response timeout");

      end

    end

  endgenerate

endmodule
