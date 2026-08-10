//=============================================================================
// File        : tl_uh_master.sv
// Description : TileLink Uncached Heavy (TL-UH) Master
// Author      : Ashish Pradhan
// TileLink specification : 1.8.x
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
// Features
// --------------------------------
//   * Parameterized address/data/source widths
//   * Multi-beat transactions
//   * Single outstanding TL transaction
//   * A-channel ready/valid
//   * D-channel ready/valid
//   * Full backpressure handling
//   * Registered request state
//   * Stable A channel while stalled
//   * Stable D channel while stalled
//   * Source ID preservation
//   * Byte enables
//   * Get
//   * PutFullData
//   * PutPartialData
//   * ArithmeticData
//   * LogicalData
//   * Intent
//   * Atomic operation support
//   * Error propagation
//   * Timeout assertion
//   * Protocol assertions
//
// Design note
// --------------------------------
// This module intentionally supports one outstanding TL-UH transaction.
// A transaction may contain multiple DATA_WIDTH beats.
//
// For a high-throughput SoC implementation, this module can subsequently be
// wrapped by a source-ID transaction table to allow multiple outstanding
// requests.
//
//=============================================================================

module tl_uh_master #(

  parameter int unsigned ADDR_WIDTH   = 40,
  parameter int unsigned DATA_WIDTH   = 64,
  parameter int unsigned SOURCE_WIDTH = 6,

  parameter logic [SOURCE_WIDTH-1:0] SOURCE_ID = '0,

  // Maximum TL transfer size represented as log2(bytes).
  //
  // Example:
  //   DATA_WIDTH = 64
  //   MAX_SIZE   = 6  -> maximum 64-byte transfer
  //
  parameter int unsigned MAX_SIZE = 6,

  // Optional response timeout.
  // 0 = disabled.
  parameter int unsigned RESPONSE_TIMEOUT = 0,

  parameter bit ENABLE_ASSERTIONS = 1'b1

) (

  input logic clk_i,
  input logic rst_ni,

  //=========================================================================
  // LOCAL REQUEST INTERFACE
  //=========================================================================

  input logic req_valid_i,
  output logic req_ready_o,

  // TileLink A opcode
  input logic [2:0] req_opcode_i,

  // TileLink A param
  input logic [2:0] req_param_i,

  // log2(total transfer bytes)
  input logic [SIZE_WIDTH-1:0] req_size_i,

  input logic [ADDR_WIDTH-1:0] req_addr_i,

  input logic [DATA_WIDTH-1:0] req_wdata_i,

  input logic [DATA_WIDTH/8-1:0] req_be_i,

  //=========================================================================
  // LOCAL RESPONSE
  //=========================================================================

  output logic rsp_valid_o,
  input logic rsp_ready_i,

  output logic [DATA_WIDTH-1:0] rsp_rdata_o,

  output logic rsp_error_o,

  //=========================================================================
  // TILELINK A CHANNEL
  //=========================================================================

  output logic [2:0] tl_a_opcode,

  output logic [2:0] tl_a_param,

  output logic [SIZE_WIDTH-1:0] tl_a_size,

  output logic [SOURCE_WIDTH-1:0] tl_a_source,

  output logic [ADDR_WIDTH-1:0] tl_a_address,

  output logic [DATA_WIDTH/8-1:0] tl_a_mask,

  output logic tl_a_corrupt,

  output logic [DATA_WIDTH-1:0] tl_a_data,

  output logic tl_a_valid,
  input logic tl_a_ready,

  //=========================================================================
  // TILELINK D CHANNEL
  //=========================================================================

  input logic [2:0] tl_d_opcode,

  input logic [1:0] tl_d_param,

  input logic [SIZE_WIDTH-1:0] tl_d_size,

  input logic [SOURCE_WIDTH-1:0] tl_d_source,

  input logic tl_d_sink,

  input logic tl_d_denied,

  input logic [DATA_WIDTH-1:0] tl_d_data,

  input logic tl_d_corrupt,

  input logic tl_d_valid,
  output logic tl_d_ready

);

  //=========================================================================
  // CONSTANTS
  //=========================================================================

  localparam int unsigned BYTE_LANES = DATA_WIDTH / 8;

  localparam int unsigned SIZE_WIDTH =
      (BYTE_LANES <= 1)
      ? 1
      : $clog2(BYTE_LANES + 1);

  localparam int unsigned BEAT_SIZE =
      (BYTE_LANES <= 1)
      ? 0
      : $clog2(BYTE_LANES);

  localparam int unsigned BEAT_COUNT_WIDTH =
      (MAX_SIZE <= BEAT_SIZE)
      ? 1
      : MAX_SIZE - BEAT_SIZE + 1;

  //=========================================================================
  // TILELINK OPCODES
  //=========================================================================

  localparam logic [2:0] TL_PUTFULL    = 3'd0;
  localparam logic [2:0] TL_PUTPARTIAL = 3'd1;
  localparam logic [2:0] TL_ARITHMETIC = 3'd2;
  localparam logic [2:0] TL_LOGICAL    = 3'd3;
  localparam logic [2:0] TL_GET        = 3'd4;
  localparam logic [2:0] TL_INTENT     = 3'd5;

  localparam logic [2:0] TL_ACCESSACK     = 3'd0;
  localparam logic [2:0] TL_ACCESSACKDATA = 3'd1;
  localparam logic [2:0] TL_HINTACK       = 3'd2;

  //=========================================================================
  // STATE MACHINE
  //=========================================================================

  typedef enum logic [3:0] {

    ST_IDLE             = 4'd0,

    ST_SEND_A           = 4'd1,

    ST_WAIT_D           = 4'd2,

    ST_SEND_NEXT_A      = 4'd3,

    ST_WAIT_NEXT_D      = 4'd4,

    ST_RESP             = 4'd5,

    ST_ERROR            = 4'd6

  } state_t;

  state_t state_q;
  state_t state_d;

  //=========================================================================
  // REQUEST REGISTERS
  //=========================================================================

  logic [2:0] a_opcode_q;
  logic [2:0] a_param_q;

  logic [SIZE_WIDTH-1:0] a_size_q;

  logic [ADDR_WIDTH-1:0] a_address_q;

  logic [DATA_WIDTH-1:0] a_wdata_q;

  logic [DATA_WIDTH/8-1:0] a_be_q;

  //=========================================================================
  // TRANSACTION STATE
  //=========================================================================

  logic [BEAT_COUNT_WIDTH-1:0] beat_index_q;

  logic [BEAT_COUNT_WIDTH-1:0] beat_count;

  logic [ADDR_WIDTH-1:0] beat_address_q;

  //=========================================================================
  // RESPONSE ACCUMULATION
  //=========================================================================

  logic [DATA_WIDTH-1:0] rsp_data_q;

  logic rsp_error_q;

  logic rsp_corrupt_q;

  //=========================================================================
  // TIMEOUT
  //=========================================================================

  localparam int unsigned TIMEOUT_WIDTH =
      (RESPONSE_TIMEOUT <= 1)
      ? 1
      : $clog2(RESPONSE_TIMEOUT + 1);

  logic [TIMEOUT_WIDTH-1:0] timeout_cnt_q;

  //=========================================================================
  // HANDSHAKES
  //=========================================================================

  logic req_fire;
  logic a_fire;
  logic d_fire;
  logic rsp_fire;

  assign req_fire =
      req_valid_i &&
      req_ready_o;

  assign a_fire =
      tl_a_valid &&
      tl_a_ready;

  assign d_fire =
      tl_d_valid &&
      tl_d_ready;

  assign rsp_fire =
      rsp_valid_o &&
      rsp_ready_i;

  //=========================================================================
  // OPCODE CLASSIFICATION
  //=========================================================================

  logic is_get;
  logic is_put;
  logic is_atomic;
  logic is_intent;

  always_comb begin

    is_get    = 1'b0;
    is_put    = 1'b0;
    is_atomic = 1'b0;
    is_intent = 1'b0;

    unique case (a_opcode_q)

      TL_GET:
        is_get = 1'b1;

      TL_PUTFULL,
      TL_PUTPARTIAL:
        is_put = 1'b1;

      TL_ARITHMETIC,
      TL_LOGICAL:
        is_atomic = 1'b1;

      TL_INTENT:
        is_intent = 1'b1;

      default:
        begin
          is_get    = 1'b0;
          is_put    = 1'b0;
          is_atomic = 1'b0;
          is_intent = 1'b0;
        end

    endcase

  end

  //=========================================================================
  // REQUEST SIZE / BEAT COUNT
  //=========================================================================

  always_comb begin

    if (a_size_q <= BEAT_SIZE)

      beat_count = 1;

    else

      beat_count =
          (1 << (a_size_q - BEAT_SIZE));

  end

  //=========================================================================
  // REQUEST READY
  //=========================================================================

  always_comb begin

    req_ready_o = 1'b0;

    if (state_q == ST_IDLE)

      req_ready_o = 1'b1;

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

        if (req_fire)

          state_d = ST_SEND_A;

      end

      //=======================================================================
      // SEND FIRST A BEAT
      //=======================================================================

      ST_SEND_A: begin

        if (a_fire)

          state_d = ST_WAIT_D;

      end

      //=======================================================================
      // WAIT FIRST D
      //=======================================================================

      ST_WAIT_D: begin

        if (d_fire) begin

          if (tl_d_denied || tl_d_corrupt)

            state_d = ST_ERROR;

          else if (beat_index_q + 1 < beat_count)

            state_d = ST_SEND_NEXT_A;

          else

            state_d = ST_RESP;

        end

      end

      //=======================================================================
      // SEND NEXT A
      //=======================================================================

      ST_SEND_NEXT_A: begin

        if (a_fire)

          state_d = ST_WAIT_NEXT_D;

      end

      //=======================================================================
      // WAIT NEXT D
      //=======================================================================

      ST_WAIT_NEXT_D: begin

        if (d_fire) begin

          if (tl_d_denied || tl_d_corrupt)

            state_d = ST_ERROR;

          else if (beat_index_q + 1 < beat_count)

            state_d = ST_SEND_NEXT_A;

          else

            state_d = ST_RESP;

        end

      end

      //=======================================================================
      // LOCAL RESPONSE
      //=======================================================================

      ST_RESP: begin

        if (rsp_fire)

          state_d = ST_IDLE;

      end

      //=======================================================================
      // ERROR
      //=======================================================================

      ST_ERROR: begin

        state_d = ST_RESP;

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

      a_opcode_q  <= TL_GET;
      a_param_q   <= '0;
      a_size_q    <= '0;
      a_address_q <= '0;
      a_wdata_q   <= '0;
      a_be_q      <= '0;

      beat_index_q   <= '0;
      beat_address_q <= '0;

      rsp_data_q    <= '0;
      rsp_error_q   <= 1'b0;
      rsp_corrupt_q <= 1'b0;

      timeout_cnt_q <= '0;

    end
    else begin

      state_q <= state_d;

      //-----------------------------------------------------------------------
      // Capture new local transaction
      //-----------------------------------------------------------------------

      if (req_fire) begin

        a_opcode_q  <= req_opcode_i;
        a_param_q   <= req_param_i;
        a_size_q    <= req_size_i;
        a_address_q <= req_addr_i;
        a_wdata_q   <= req_wdata_i;
        a_be_q      <= req_be_i;

        beat_index_q   <= '0;
        beat_address_q <= req_addr_i;

        rsp_data_q    <= '0;
        rsp_error_q   <= 1'b0;
        rsp_corrupt_q <= 1'b0;

      end

      //-----------------------------------------------------------------------
      // Advance beat after D response
      //-----------------------------------------------------------------------

      if (d_fire) begin

        if (!tl_d_denied &&
            !tl_d_corrupt &&
            (beat_index_q + 1 < beat_count)) begin

          beat_index_q <= beat_index_q + 1'b1;

          beat_address_q <=
              beat_address_q + BYTE_LANES;

        end

      end

      //-----------------------------------------------------------------------
      // Capture read data
      //
      // The response from the final beat is exposed to the local interface.
      // For a single DATA_WIDTH local interface, this represents the current
      // beat's returned data.
      //-----------------------------------------------------------------------

      if (d_fire &&
          (tl_d_opcode == TL_ACCESSACKDATA)) begin

        rsp_data_q <= tl_d_data;

      end

      //-----------------------------------------------------------------------
      // Capture errors
      //-----------------------------------------------------------------------

      if (d_fire) begin

        rsp_error_q <=
             rsp_error_q
          || tl_d_denied
          || tl_d_corrupt;

        rsp_corrupt_q <=
             rsp_corrupt_q
          || tl_d_corrupt;

      end

      //-----------------------------------------------------------------------
      // Timeout
      //-----------------------------------------------------------------------

      if ((state_q != ST_WAIT_D) &&
          (state_q != ST_WAIT_NEXT_D)) begin

        timeout_cnt_q <= '0;

      end
      else if (RESPONSE_TIMEOUT != 0) begin

        if (d_fire)

          timeout_cnt_q <= '0;

        else if (timeout_cnt_q < RESPONSE_TIMEOUT)

          timeout_cnt_q <=
              timeout_cnt_q + 1'b1;

      end

    end

  end

  //=========================================================================
  // A CHANNEL
  //=========================================================================
  //
  // All fields come from registered transaction state.
  // Therefore A is stable during backpressure.
  //
  //=========================================================================

  always_comb begin

    tl_a_opcode  = a_opcode_q;

    tl_a_param   = a_param_q;

    tl_a_size    = a_size_q;

    tl_a_source  = SOURCE_ID;

    tl_a_address = beat_address_q;

    tl_a_mask    = a_be_q;

    tl_a_corrupt = 1'b0;

    tl_a_data    = a_wdata_q;

    tl_a_valid =
         (state_q == ST_SEND_A)
      || (state_q == ST_SEND_NEXT_A);

  end

  //=========================================================================
  // D READY
  //=========================================================================

  always_comb begin

    tl_d_ready = 1'b0;

    if ((state_q == ST_WAIT_D) ||
        (state_q == ST_WAIT_NEXT_D))

      tl_d_ready = 1'b1;

  end

  //=========================================================================
  // LOCAL RESPONSE
  //=========================================================================

  always_comb begin

    rsp_valid_o = 1'b0;

    rsp_rdata_o = rsp_data_q;

    rsp_error_o = rsp_error_q;

    if (state_q == ST_RESP)

      rsp_valid_o = 1'b1;

  end

  //=========================================================================
  // ERROR RESPONSE
  //=========================================================================

  always_ff @(posedge clk_i or negedge rst_ni) begin

    if (!rst_ni) begin

      rsp_error_q <= 1'b0;

    end
    else if (state_q == ST_ERROR) begin

      rsp_error_q <= 1'b1;

    end

  end

  //=========================================================================
  // ASSERTIONS
  //=========================================================================

  generate

    if (ENABLE_ASSERTIONS) begin : gen_tl_uh_master_sva

      //=======================================================================
      // A-channel stability
      //=======================================================================

      property p_a_stable;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_a_valid &&
        !tl_a_ready

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
          $error(
            "TL-UH MASTER: A channel changed while stalled"
          );

      //=======================================================================
      // D ready only while waiting
      //=======================================================================

      property p_d_ready_state;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_ready
        |->
        (
             (state_q == ST_WAIT_D)
          || (state_q == ST_WAIT_NEXT_D)
        );

      endproperty

      assert property (p_d_ready_state)
        else
          $error(
            "TL-UH MASTER: D ready asserted in illegal state"
          );

      //=======================================================================
      // D source must match
      //=======================================================================

      property p_d_source;

        @(posedge clk_i)
        disable iff (!rst_ni)

        d_fire
        |->
        (tl_d_source == SOURCE_ID);

      endproperty

      assert property (p_d_source)
        else
          $error(
            "TL-UH MASTER: D source mismatch"
          );

      //=======================================================================
      // Get response
      //=======================================================================

      property p_get_response;

        @(posedge clk_i)
        disable iff (!rst_ni)

        d_fire &&
        (a_opcode_q == TL_GET)

        |->
        (tl_d_opcode == TL_ACCESSACKDATA);

      endproperty

      assert property (p_get_response)
        else
          $error(
            "TL-UH MASTER: Get requires AccessAckData"
          );

      //=======================================================================
      // Put response
      //=======================================================================

      property p_put_response;

        @(posedge clk_i)
        disable iff (!rst_ni)

        d_fire &&
        (
          (a_opcode_q == TL_PUTFULL) ||
          (a_opcode_q == TL_PUTPARTIAL)
        )

        |->
        (tl_d_opcode == TL_ACCESSACK);

      endproperty

      assert property (p_put_response)
        else
          $error(
            "TL-UH MASTER: Put requires AccessAck"
          );

      //=======================================================================
      // Intent response
      //=======================================================================

      property p_intent_response;

        @(posedge clk_i)
        disable iff (!rst_ni)

        d_fire &&
        (a_opcode_q == TL_INTENT)

        |->
        (tl_d_opcode == TL_HINTACK);

      endproperty

      assert property (p_intent_response)
        else
          $error(
            "TL-UH MASTER: Intent requires HintAck"
          );

      //=======================================================================
      // A param for normal accesses
      //=======================================================================

      property p_normal_param_zero;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_a_valid &&
        (
          (tl_a_opcode == TL_GET) ||
          (tl_a_opcode == TL_PUTFULL) ||
          (tl_a_opcode == TL_PUTPARTIAL)
        )

        |->
        (tl_a_param == 3'b000);

      endproperty

      assert property (p_normal_param_zero)
        else
          $error(
            "TL-UH MASTER: illegal param on normal access"
          );

      //=======================================================================
      // A corrupt must be zero
      //=======================================================================

      property p_a_corrupt_zero;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_a_valid
        |->
        !tl_a_corrupt;

      endproperty

      assert property (p_a_corrupt_zero)
        else
          $error(
            "TL-UH MASTER: a_corrupt must be zero"
          );

      //=======================================================================
      // D channel stability
      //=======================================================================

      property p_d_stable;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_d_valid &&
        !tl_d_ready

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
          $error(
            "TL-UH MASTER: D channel changed while stalled"
          );

      //=======================================================================
      // D response cannot be accepted outside wait states
      //=======================================================================

      property p_d_accept_state;

        @(posedge clk_i)
        disable iff (!rst_ni)

        d_fire
        |->
        (
             (state_q == ST_WAIT_D)
          || (state_q == ST_WAIT_NEXT_D)
        );

      endproperty

      assert property (p_d_accept_state)
        else
          $error(
            "TL-UH MASTER: unexpected D response"
          );

      //=======================================================================
      // A channel must be valid only in A states
      //=======================================================================

      property p_a_valid_state;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_a_valid
        |->
        (
             (state_q == ST_SEND_A)
          || (state_q == ST_SEND_NEXT_A)
        );

      endproperty

      assert property (p_a_valid_state)
        else
          $error(
            "TL-UH MASTER: A valid in illegal state"
          );

      //=======================================================================
      // Source remains constant
      //=======================================================================

      property p_source_constant;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_a_valid
        |->
        (tl_a_source == SOURCE_ID);

      endproperty

      assert property (p_source_constant)
        else
          $error(
            "TL-UH MASTER: invalid source ID"
          );

      //=======================================================================
      // D sink must be zero for this non-coherent master
      //=======================================================================

      property p_d_sink_zero;

        @(posedge clk_i)
        disable iff (!rst_ni)

        d_fire
        |->
        (tl_d_sink == 1'b0);

      endproperty

      assert property (p_d_sink_zero)
        else
          $error(
            "TL-UH MASTER: unexpected D sink"
          );

      //=======================================================================
      // Timeout
      //=======================================================================

      if (RESPONSE_TIMEOUT != 0) begin : gen_timeout_sva

        property p_response_timeout;

          @(posedge clk_i)
          disable iff (!rst_ni)

          (
            (state_q == ST_WAIT_D) ||
            (state_q == ST_WAIT_NEXT_D)
          )

          |->
          (timeout_cnt_q < RESPONSE_TIMEOUT);

        endproperty

        assert property (p_response_timeout)
          else
            $error(
              "TL-UH MASTER: D-channel response timeout"
            );

      end

    end

  endgenerate

endmodule
