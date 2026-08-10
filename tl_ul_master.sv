//=============================================================================
// File        : tl_ul_master.sv
// Description : TileLink Uncached Lightweight (TL-UL)
//               Master / Manager
// Auth : Ashish Pradhan
// TileLink Specification : v1.8.1
//
// Supported A-channel operations:
//   - Get
//   - PutFullData
//   - PutPartialData
//
// Supported D-channel responses:
//   - AccessAck
//   - AccessAckData
//
// Features:
//   - Single outstanding transaction
//   - Ready/valid decoupled request interface
//   - Full backpressure support
//   - Byte-granular write strobes
//   - Parameterized address/data/source widths
//   - Request capture before A-channel transmission
//   - Response capture before completion indication
//   - Stable A channel under backpressure
//   - Stable D-channel response under backpressure
//   - Timeout detection
//   - Protocol assertions
//   - Optional request/response error reporting
//   - No combinational path from TL D channel to request interface
//
// Notes:
//   1. TL-UL is single-beat. No burst support is implemented here.
//   2. a_corrupt is driven low.
//   3. a_param is driven zero.
//   4. d_sink is ignored by TL-UL master.
//   5. Source ID is fixed/configurable for this single-outstanding master.
//
//=============================================================================

module tl_ul_master #(
  parameter int unsigned ADDR_WIDTH       = 32,
  parameter int unsigned DATA_WIDTH       = 64,
  parameter int unsigned SOURCE_WIDTH     = 4,

  // Number of cycles allowed between A-channel handshake and D-channel
  // response. 0 disables timeout checking.
  parameter int unsigned RESPONSE_TIMEOUT = 0,

  // Fixed source ID used for all transactions.
  parameter logic [SOURCE_WIDTH-1:0] SOURCE_ID = '0,

  // Enable SystemVerilog assertions.
  parameter bit ENABLE_ASSERTIONS = 1'b1
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,

  //-------------------------------------------------------------------------
  // Local request interface
  //-------------------------------------------------------------------------
  //
  // req_opcode:
  //   0 = PutFullData
  //   1 = PutPartialData
  //   4 = Get
  //
  input  logic                         req_valid_i,
  output logic                         req_ready_o,

  input  logic [2:0]                   req_opcode_i,
  input  logic [ADDR_WIDTH-1:0]        req_addr_i,
  input  logic [DATA_WIDTH-1:0]        req_wdata_i,
  input  logic [DATA_WIDTH/8-1:0]      req_be_i,

  // log2(number of bytes accessed)
  input  logic [$clog2(DATA_WIDTH/8+1)-1:0] req_size_i,

  //-------------------------------------------------------------------------
  // Local response interface
  //-------------------------------------------------------------------------
  req_rsp_valid_o,
  req_rsp_ready_i,

  output logic [DATA_WIDTH-1:0]        req_rdata_o,
  output logic                         req_error_o,

  //-------------------------------------------------------------------------
  // TileLink A channel
  //-------------------------------------------------------------------------
  output logic [2:0]                   tl_a_opcode,
  output logic [2:0]                   tl_a_param,
  output logic [$clog2(DATA_WIDTH/8+1)-1:0] tl_a_size,
  output logic [SOURCE_WIDTH-1:0]      tl_a_source,
  output logic [ADDR_WIDTH-1:0]        tl_a_address,
  output logic [DATA_WIDTH/8-1:0]      tl_a_mask,
  output logic                         tl_a_corrupt,
  output logic [DATA_WIDTH-1:0]        tl_a_data,
  output logic                         tl_a_valid,
  input  logic                         tl_a_ready,

  //-------------------------------------------------------------------------
  // TileLink D channel
  //-------------------------------------------------------------------------
  input  logic [2:0]                   tl_d_opcode,
  input  logic [1:0]                   tl_d_param,
  input  logic [$clog2(DATA_WIDTH/8+1)-1:0] tl_d_size,
  input  logic [SOURCE_WIDTH-1:0]      tl_d_source,
  input  logic                         tl_d_sink,
  input  logic                         tl_d_denied,
  input  logic [DATA_WIDTH-1:0]        tl_d_data,
  input  logic                         tl_d_corrupt,
  input  logic                         tl_d_valid,
  output logic                         tl_d_ready
);

  //-------------------------------------------------------------------------
  // Local constants
  //-------------------------------------------------------------------------

  localparam int unsigned BYTE_LANES = DATA_WIDTH / 8;

  localparam int unsigned SIZE_WIDTH =
      (BYTE_LANES <= 1) ? 1 : $clog2(BYTE_LANES + 1);

  localparam logic [2:0] TL_PUTFULL   = 3'd0;
  localparam logic [2:0] TL_PUTPART   = 3'd1;
  localparam logic [2:0] TL_GET       = 3'd4;

  localparam logic [2:0] TL_ACCESSACK     = 3'd0;
  localparam logic [2:0] TL_ACCESSACKDATA = 3'd1;

  //-------------------------------------------------------------------------
  // State machine
  //-------------------------------------------------------------------------

  typedef enum logic [1:0] {
    ST_IDLE      = 2'b00,
    ST_SEND_A    = 2'b01,
    ST_WAIT_D    = 2'b10,
    ST_RSP       = 2'b11
  } state_t;

  state_t state_q, state_d;

  //-------------------------------------------------------------------------
  // Latched request
  //-------------------------------------------------------------------------

  logic [2:0]              req_opcode_q;
  logic [ADDR_WIDTH-1:0]   req_addr_q;
  logic [DATA_WIDTH-1:0]   req_wdata_q;
  logic [BYTE_LANES-1:0]   req_be_q;
  logic [SIZE_WIDTH-1:0]   req_size_q;

  //-------------------------------------------------------------------------
  // Latched response
  //-------------------------------------------------------------------------

  logic [DATA_WIDTH-1:0]   rsp_rdata_q;
  logic                    rsp_error_q;

  //-------------------------------------------------------------------------
  // Response timeout counter
  //-------------------------------------------------------------------------

  localparam int unsigned TIMEOUT_WIDTH =
      (RESPONSE_TIMEOUT <= 1) ? 1 : $clog2(RESPONSE_TIMEOUT + 1);

  logic [TIMEOUT_WIDTH-1:0] timeout_cnt_q;

  //-------------------------------------------------------------------------
  // Internal handshake signals
  //-------------------------------------------------------------------------

  logic a_fire;
  logic d_fire;
  logic req_fire;
  logic rsp_fire;

  assign req_fire = req_valid_i && req_ready_o;
  assign a_fire   = tl_a_valid && tl_a_ready;
  assign d_fire   = tl_d_valid && tl_d_ready;
  assign rsp_fire = req_rsp_valid_o && req_rsp_ready_i;

  //-------------------------------------------------------------------------
  // Request ready
  //-------------------------------------------------------------------------
  //
  // New request can only be accepted when the previous transaction has
  // completely retired.
  //
  // This deliberately prevents request overwriting and guarantees one
  // outstanding transaction.
  //-------------------------------------------------------------------------

  always_comb begin
    req_ready_o = 1'b0;

    if (state_q == ST_IDLE)
      req_ready_o = 1'b1;
  end

  //-------------------------------------------------------------------------
  // State transition
  //-------------------------------------------------------------------------

  always_comb begin

    state_d = state_q;

    unique case (state_q)

      ST_IDLE: begin
        if (req_fire)
          state_d = ST_SEND_A;
      end

      ST_SEND_A: begin
        if (a_fire)
          state_d = ST_WAIT_D;
      end

      ST_WAIT_D: begin
        if (d_fire)
          state_d = ST_RSP;
      end

      ST_RSP: begin
        if (rsp_fire)
          state_d = ST_IDLE;
      end

      default: begin
        state_d = ST_IDLE;
      end

    endcase
  end

  //-------------------------------------------------------------------------
  // Sequential state and transaction storage
  //-------------------------------------------------------------------------

  always_ff @(posedge clk_i or negedge rst_ni) begin

    if (!rst_ni) begin

      state_q       <= ST_IDLE;

      req_opcode_q  <= TL_GET;
      req_addr_q    <= '0;
      req_wdata_q   <= '0;
      req_be_q      <= '0;
      req_size_q    <= '0;

      rsp_rdata_q   <= '0;
      rsp_error_q   <= 1'b0;

      timeout_cnt_q <= '0;

    end
    else begin

      state_q <= state_d;

      //-----------------------------------------------------------------------
      // Capture local request
      //-----------------------------------------------------------------------

      if (req_fire) begin

        req_opcode_q <= req_opcode_i;
        req_addr_q   <= req_addr_i;
        req_wdata_q  <= req_wdata_i;
        req_be_q     <= req_be_i;
        req_size_q   <= req_size_i;

      end

      //-----------------------------------------------------------------------
      // Capture D-channel response
      //-----------------------------------------------------------------------

      if (d_fire) begin

        rsp_rdata_q <= tl_d_data;

        rsp_error_q <=
             tl_d_denied
          || tl_d_corrupt
          || (tl_d_opcode != expected_d_opcode(req_opcode_q));

      end

      //-----------------------------------------------------------------------
      // Response timeout
      //-----------------------------------------------------------------------

      if (state_q != ST_WAIT_D) begin

        timeout_cnt_q <= '0;

      end
      else if (RESPONSE_TIMEOUT != 0) begin

        if (d_fire) begin

          timeout_cnt_q <= '0;

        end
        else if (timeout_cnt_q < RESPONSE_TIMEOUT) begin

          timeout_cnt_q <= timeout_cnt_q + 1'b1;

        end

      end

    end

  end

  //-------------------------------------------------------------------------
  // TileLink A channel
  //-------------------------------------------------------------------------
  //
  // All A-channel fields are driven from registered request state.
  // Therefore all fields remain stable while a_valid=1 and a_ready=0.
  //-------------------------------------------------------------------------

  always_comb begin

    tl_a_opcode  = req_opcode_q;
    tl_a_param   = 3'b000;
    tl_a_size    = req_size_q;
    tl_a_source  = SOURCE_ID;
    tl_a_address = req_addr_q;
    tl_a_mask    = req_be_q;
    tl_a_corrupt = 1'b0;
    tl_a_data    = req_wdata_q;

    tl_a_valid   = (state_q == ST_SEND_A);

  end

  //-------------------------------------------------------------------------
  // TileLink D channel ready
  //-------------------------------------------------------------------------
  //
  // We accept exactly one D response and latch it before exposing it to the
  // local response interface.
  //
  // This decouples TL D-channel backpressure from the local response path.
  //-------------------------------------------------------------------------

  always_comb begin

    tl_d_ready = 1'b0;

    if (state_q == ST_WAIT_D)
      tl_d_ready = 1'b1;

  end

  //-------------------------------------------------------------------------
  // Local response interface
  //-------------------------------------------------------------------------

  assign req_rsp_valid_o = (state_q == ST_RSP);

  assign req_rdata_o     = rsp_rdata_q;
  assign req_error_o     = rsp_error_q;

  //-------------------------------------------------------------------------
  // Expected D opcode
  //-------------------------------------------------------------------------

  function automatic logic [2:0] expected_d_opcode (
    input logic [2:0] opcode
  );

    unique case (opcode)

      TL_GET:
        expected_d_opcode = TL_ACCESSACKDATA;

      TL_PUTFULL,
      TL_PUTPART:
        expected_d_opcode = TL_ACCESSACK;

      default:
        expected_d_opcode = 3'b111;

    endcase

  endfunction

  //-------------------------------------------------------------------------
  // Assertions
  //-------------------------------------------------------------------------

  generate

    if (ENABLE_ASSERTIONS) begin : gen_assertions

      //-----------------------------------------------------------------------
      // Request opcode must be supported
      //-----------------------------------------------------------------------

      property p_valid_request_opcode;
        @(posedge clk_i)
        disable iff (!rst_ni)
        req_fire |-> (
             req_opcode_i == TL_GET
          || req_opcode_i == TL_PUTFULL
          || req_opcode_i == TL_PUTPART
        );
      endproperty

      assert property (p_valid_request_opcode)
        else $error("TL-UL master: unsupported request opcode");

      //-----------------------------------------------------------------------
      // A-channel stability under backpressure
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
        else $error("TL-UL master: A channel changed while stalled");

      //-----------------------------------------------------------------------
      // D response source must match our source
      //-----------------------------------------------------------------------

      property p_d_source;
        @(posedge clk_i)
        disable iff (!rst_ni)
        d_fire |-> (tl_d_source == SOURCE_ID);
      endproperty

      assert property (p_d_source)
        else $error("TL-UL master: D source mismatch");

      //-----------------------------------------------------------------------
      // Get must receive AccessAckData
      //-----------------------------------------------------------------------

      property p_get_response;
        @(posedge clk_i)
        disable iff (!rst_ni)
        d_fire && (req_opcode_q == TL_GET)
        |-> (tl_d_opcode == TL_ACCESSACKDATA);
      endproperty

      assert property (p_get_response)
        else $error("TL-UL master: invalid response for Get");

      //-----------------------------------------------------------------------
      // Put must receive AccessAck
      //-----------------------------------------------------------------------

      property p_put_response;
        @(posedge clk_i)
        disable iff (!rst_ni)
        d_fire &&
        ((req_opcode_q == TL_PUTFULL) ||
         (req_opcode_q == TL_PUTPART))
        |-> (tl_d_opcode == TL_ACCESSACK);
      endproperty

      assert property (p_put_response)
        else $error("TL-UL master: invalid response for Put");

      //-----------------------------------------------------------------------
      // TL-UL A corrupt must always be zero
      //-----------------------------------------------------------------------

      property p_a_not_corrupt;
        @(posedge clk_i)
        disable iff (!rst_ni)
        tl_a_valid |-> !tl_a_corrupt;
      endproperty

      assert property (p_a_not_corrupt)
        else $error("TL-UL master: a_corrupt must be zero");

      //-----------------------------------------------------------------------
      // TL-UL param is reserved and must be zero
      //-----------------------------------------------------------------------

      property p_a_param_zero;
        @(posedge clk_i)
        disable iff (!rst_ni)
        tl_a_valid |-> (tl_a_param == 3'b000);
      endproperty

      assert property (p_a_param_zero)
        else $error("TL-UL master: a_param must be zero");

      //-----------------------------------------------------------------------
      // No D channel accepted outside WAIT_D
      //-----------------------------------------------------------------------

      property p_d_only_wait;
        @(posedge clk_i)
        disable iff (!rst_ni)
        d_fire |-> (state_q == ST_WAIT_D);
      endproperty

      assert property (p_d_only_wait)
        else $error("TL-UL master: D response accepted outside WAIT_D");

      //-----------------------------------------------------------------------
      // Response must not appear before D response
      //-----------------------------------------------------------------------

      property p_rsp_after_d;
        @(posedge clk_i)
        disable iff (!rst_ni)
        req_rsp_valid_o |-> (state_q == ST_RSP);
      endproperty

      assert property (p_rsp_after_d)
        else $error("TL-UL master: invalid local response state");

      //-----------------------------------------------------------------------
      // Address alignment
      //-----------------------------------------------------------------------
      //
      // For TL-UL, address must be aligned to the requested size for
      // Get/PutFullData. PutPartialData permits arbitrary alignment.
      //-----------------------------------------------------------------------

      property p_full_put_alignment;
        @(posedge clk_i)
        disable iff (!rst_ni)
        req_fire &&
        (req_opcode_i == TL_PUTFULL)
        |->
        ((req_addr_i & ((1 << req_size_i) - 1)) == '0);
      endproperty

      assert property (p_full_put_alignment)
        else $error("TL-UL master: PutFullData address is not aligned");

      //-----------------------------------------------------------------------
      // Timeout
      //-----------------------------------------------------------------------

      if (RESPONSE_TIMEOUT != 0) begin : gen_timeout_assert

        property p_response_timeout;
          @(posedge clk_i)
          disable iff (!rst_ni)
          (state_q == ST_WAIT_D)
          |->
          (timeout_cnt_q < RESPONSE_TIMEOUT);
        endproperty

        assert property (p_response_timeout)
          else $error("TL-UL master: response timeout");

      end

    end

  endgenerate

endmodule
