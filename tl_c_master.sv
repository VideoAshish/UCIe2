//=============================================================================
// File        : tl_c_master.sv
// Description : TileLink Coherent (TL-C) Master / Client Endpoint
// Author      : Ashish Pradhan
// TileLink specification : 1.8.x
//
// Channels
// --------
//   A : Master -> Manager : Acquire / normal access requests
//   B : Manager -> Master : Probe requests
//   C : Master -> Manager : ProbeAck / Release / Writeback
//   D : Manager -> Master : Grant / AccessAck / ReleaseAck
//   E : Master -> Manager : GrantAck
//
// Supported coherent messages
// ----------------------------
// A:
//   AcquireBlock
//   AcquirePerm
//
// B:
//   ProbeBlock
//   ProbePerm
//
// C:
//   ProbeAck
//   ProbeAckData
//   Release
//   ReleaseData
//
// D:
//   Grant
//   GrantData
//   AccessAck
//   AccessAckData
//   ReleaseAck
//
// E:
//   GrantAck
//
// Architecture
// ------------
//
//                  +---------------------------------------+
//                  |              tl_c_master              |
//                  |                                       |
//   Local -------->|  Request Engine                       |
//   Cache          |        |                              |
//                  |        v                              |
//                  |  A Channel -------------------------> |
//                  |                                       |
//                  |  D Channel <------------------------- |
//                  |        |                              |
//                  |        +--> Grant Engine              |
//                  |                                       |
//                  |  B Channel <------------------------- |
//                  |        |                              |
//                  |        v                              |
//                  |  Probe Engine                         |
//                  |        |                              |
//                  |        v                              |
//                  |  C Channel -------------------------> |
//                  |                                       |
//                  |  E Channel -------------------------> |
//                  +---------------------------------------+
//
// Important
// ---------
// This module is the TileLink protocol endpoint. Actual cache-line data,
// tag lookup, permission transitions and replacement policy remain in the
// cache/coherence backend.
//
// The cache backend is expected to:
//   * identify whether a Probe hits,
//   * supply the current coherence state,
//   * supply dirty data when required,
//   * apply permission transitions,
//   * install GrantData,
//   * acknowledge cache-line installation.
//
//=============================================================================

module tl_c_master #(

  parameter int unsigned ADDR_WIDTH   = 40,
  parameter int unsigned DATA_WIDTH   = 64,
  parameter int unsigned SOURCE_WIDTH = 6,
  parameter int unsigned SINK_WIDTH   = 4,

  parameter int unsigned MAX_SIZE     = 6,

  parameter logic [SOURCE_WIDTH-1:0] SOURCE_BASE = '0,

  parameter bit ENABLE_ASSERTIONS = 1'b1

) (

  input  logic clk_i,
  input  logic rst_ni,

  //===========================================================================
  // LOCAL CACHE REQUEST INTERFACE
  //===========================================================================

  input  logic                         cache_req_valid_i,
  output logic                         cache_req_ready_o,

  input  logic [2:0]                   cache_req_opcode_i,
  input  logic [2:0]                   cache_req_param_i,

  input  logic [SIZE_WIDTH-1:0]        cache_req_size_i,

  input  logic [ADDR_WIDTH-1:0]        cache_req_addr_i,

  input  logic [DATA_WIDTH-1:0]        cache_req_wdata_i,

  input  logic [DATA_WIDTH/8-1:0]      cache_req_be_i,

  //===========================================================================
  // LOCAL CACHE RESPONSE
  //===========================================================================

  output logic                         cache_rsp_valid_o,
  input  logic                         cache_rsp_ready_i,

  output logic [2:0]                   cache_rsp_opcode_o,
  output logic [DATA_WIDTH-1:0]        cache_rsp_data_o,

  output logic                         cache_rsp_denied_o,
  output logic                         cache_rsp_corrupt_o,

  //===========================================================================
  // CACHE COHERENCE / PROBE BACKEND
  //===========================================================================

  // Lookup requested cache line.
  output logic                         coh_lookup_valid_o,
  input  logic                         coh_lookup_ready_i,

  output logic [ADDR_WIDTH-1:0]        coh_lookup_addr_o,

  // Current cache state.
  //
  // 0 = Invalid
  // 1 = Branch
  // 2 = Trunk
  // 3 = Tip
  //
  input  logic [1:0]                   coh_state_i,

  // Probe hit.
  input  logic                         coh_hit_i,

  // Dirty cache line.
  input  logic                         coh_dirty_i,

  // Data returned by cache backend.
  input  logic [DATA_WIDTH-1:0]        coh_rdata_i,

  // Backend is ready to consume a probe action.
  output logic                         coh_probe_valid_o,
  input  logic                         coh_probe_ready_i,

  // Requested coherence transition.
  output logic [2:0]                   coh_probe_param_o,

  // Cache line data response.
  output logic                         coh_data_valid_o,
  input  logic                         coh_data_ready_i,

  // Grant installation notification.
  output logic                         coh_grant_valid_o,
  input  logic                         coh_grant_ready_i,

  output logic [ADDR_WIDTH-1:0]        coh_grant_addr_o,
  output logic [DATA_WIDTH-1:0]         coh_grant_data_o,

  output logic [2:0]                   coh_grant_param_o,

  //===========================================================================
  // TILELINK A
  //===========================================================================

  output logic [2:0]                   tl_a_opcode,
  output logic [2:0]                   tl_a_param,
  output logic [SIZE_WIDTH-1:0]        tl_a_size,
  output logic [SOURCE_WIDTH-1:0]      tl_a_source,
  output logic [ADDR_WIDTH-1:0]        tl_a_address,
  output logic [DATA_WIDTH/8-1:0]      tl_a_mask,
  output logic [DATA_WIDTH-1:0]        tl_a_data,
  output logic                         tl_a_corrupt,
  output logic                         tl_a_valid,
  input  logic                         tl_a_ready,

  //===========================================================================
  // TILELINK B
  //===========================================================================

  input logic [2:0]                    tl_b_opcode,
  input logic [2:0]                    tl_b_param,
  input logic [SIZE_WIDTH-1:0]         tl_b_size,
  input logic [SOURCE_WIDTH-1:0]       tl_b_source,
  input logic [ADDR_WIDTH-1:0]         tl_b_address,
  input logic [DATA_WIDTH/8-1:0]       tl_b_mask,
  input logic [DATA_WIDTH-1:0]         tl_b_data,
  input logic                          tl_b_corrupt,
  input logic                          tl_b_valid,
  output logic                         tl_b_ready,

  //===========================================================================
  // TILELINK C
  //===========================================================================

  output logic [2:0]                   tl_c_opcode,
  output logic [2:0]                   tl_c_param,
  output logic [SIZE_WIDTH-1:0]        tl_c_size,
  output logic [SOURCE_WIDTH-1:0]      tl_c_source,
  output logic [ADDR_WIDTH-1:0]        tl_c_address,
  output logic [DATA_WIDTH-1:0]        tl_c_data,
  output logic                         tl_c_corrupt,
  output logic                         tl_c_valid,
  input logic                          tl_c_ready,

  //===========================================================================
  // TILELINK D
  //===========================================================================

  input logic [2:0]                    tl_d_opcode,
  input logic [1:0]                    tl_d_param,
  input logic [SIZE_WIDTH-1:0]         tl_d_size,
  input logic [SOURCE_WIDTH-1:0]       tl_d_source,
  input logic [SINK_WIDTH-1:0]         tl_d_sink,
  input logic                          tl_d_denied,
  input logic [DATA_WIDTH-1:0]         tl_d_data,
  input logic                          tl_d_corrupt,
  input logic                          tl_d_valid,
  output logic                         tl_d_ready,

  //===========================================================================
  // TILELINK E
  //===========================================================================

  output logic [SINK_WIDTH-1:0]        tl_e_sink,
  output logic                         tl_e_valid,
  input logic                          tl_e_ready

);

  //===========================================================================
  // CONSTANTS
  //===========================================================================

  localparam int unsigned BYTE_LANES = DATA_WIDTH / 8;

  localparam int unsigned SIZE_WIDTH =
      (BYTE_LANES <= 1)
      ? 1
      : $clog2(BYTE_LANES + 1);

  //===========================================================================
  // TL-C A OPCODES
  //===========================================================================

  localparam logic [2:0] A_ACQUIRE_BLOCK = 3'd6;
  localparam logic [2:0] A_ACQUIRE_PERM  = 3'd7;

  // Optional UL operations.
  localparam logic [2:0] A_PUTFULL       = 3'd0;
  localparam logic [2:0] A_PUTPARTIAL    = 3'd1;
  localparam logic [2:0] A_ARITHMETIC    = 3'd2;
  localparam logic [2:0] A_LOGICAL       = 3'd3;
  localparam logic [2:0] A_GET           = 3'd4;
  localparam logic [2:0] A_INTENT        = 3'd5;

  //===========================================================================
  // TL-C B OPCODES
  //===========================================================================

  localparam logic [2:0] B_PROBE_BLOCK = 3'd6;
  localparam logic [2:0] B_PROBE_PERM  = 3'd7;

  //===========================================================================
  // TL-C C OPCODES
  //===========================================================================

  localparam logic [2:0] C_PROBE_ACK      = 3'd4;
  localparam logic [2:0] C_PROBE_ACK_DATA = 3'd5;
  localparam logic [2:0] C_RELEASE        = 3'd6;
  localparam logic [2:0] C_RELEASE_DATA  = 3'd7;

  //===========================================================================
  // TL-C D OPCODES
  //===========================================================================

  localparam logic [2:0] D_ACCESS_ACK      = 3'd0;
  localparam logic [2:0] D_ACCESS_ACK_DATA = 3'd1;
  localparam logic [2:0] D_HINT_ACK        = 3'd2;

  localparam logic [2:0] D_GRANT           = 3'd4;
  localparam logic [2:0] D_GRANT_DATA     = 3'd5;
  localparam logic [2:0] D_RELEASE_ACK     = 3'd6;

  //===========================================================================
  // COHERENCE STATES
  //===========================================================================

  localparam logic [1:0] STATE_INVALID = 2'd0;
  localparam logic [1:0] STATE_BRANCH  = 2'd1;
  localparam logic [1:0] STATE_TRUNK   = 2'd2;
  localparam logic [1:0] STATE_TIP     = 2'd3;

  //===========================================================================
  // FSM
  //===========================================================================

  typedef enum logic [4:0] {

    ST_IDLE,

    ST_ACQUIRE_A,
    ST_ACQUIRE_D,

    ST_GRANT_INSTALL,
    ST_GRANT_ACK,

    ST_PROBE_LOOKUP,
    ST_PROBE_ACTION,
    ST_PROBE_C,

    ST_RELEASE_C,
    ST_RELEASE_D,

    ST_LOCAL_RESP

  } state_t;

  state_t state_q;
  state_t state_d;

  //===========================================================================
  // ACQUIRE REQUEST REGISTERS
  //===========================================================================

  logic [2:0]                  req_opcode_q;
  logic [2:0]                  req_param_q;
  logic [SIZE_WIDTH-1:0]       req_size_q;
  logic [ADDR_WIDTH-1:0]       req_addr_q;
  logic [DATA_WIDTH-1:0]       req_data_q;
  logic [DATA_WIDTH/8-1:0]     req_mask_q;

  logic [SOURCE_WIDTH-1:0]     req_source_q;

  //===========================================================================
  // GRANT REGISTERS
  //===========================================================================

  logic [SINK_WIDTH-1:0]       grant_sink_q;
  logic [2:0]                  grant_param_q;

  logic                        grant_has_data_q;

  //===========================================================================
  // PROBE REGISTERS
  //===========================================================================

  logic [2:0]                  probe_opcode_q;
  logic [2:0]                  probe_param_q;

  logic [SIZE_WIDTH-1:0]       probe_size_q;

  logic [SOURCE_WIDTH-1:0]     probe_source_q;

  logic [ADDR_WIDTH-1:0]       probe_addr_q;

  logic [DATA_WIDTH/8-1:0]     probe_mask_q;

  logic                        probe_hit_q;
  logic                        probe_dirty_q;

  logic [1:0]                  probe_state_q;

  //===========================================================================
  // RESPONSE
  //===========================================================================

  logic [2:0]                  local_rsp_opcode_q;
  logic [DATA_WIDTH-1:0]       local_rsp_data_q;

  logic                        local_rsp_denied_q;
  logic                        local_rsp_corrupt_q;

  //===========================================================================
  // HANDSHAKES
  //===========================================================================

  logic a_fire;
  logic b_fire;
  logic c_fire;
  logic d_fire;
  logic e_fire;

  assign a_fire = tl_a_valid && tl_a_ready;
  assign b_fire = tl_b_valid && tl_b_ready;
  assign c_fire = tl_c_valid && tl_c_ready;
  assign d_fire = tl_d_valid && tl_d_ready;
  assign e_fire = tl_e_valid && tl_e_ready;

  //===========================================================================
  // STATE MACHINE
  //===========================================================================

  always_comb begin

    state_d = state_q;

    unique case (state_q)

      //=======================================================================
      // IDLE
      //=======================================================================

      ST_IDLE: begin

        if (cache_req_valid_i &&
            cache_req_ready_o) begin

          if ((cache_req_opcode_i == A_ACQUIRE_BLOCK) ||
              (cache_req_opcode_i == A_ACQUIRE_PERM))

            state_d = ST_ACQUIRE_A;

          else

            state_d = ST_ACQUIRE_A;

        end
        else if (tl_b_valid) begin

          state_d = ST_PROBE_LOOKUP;

        end

      end

      //=======================================================================
      // ACQUIRE
      //=======================================================================

      ST_ACQUIRE_A: begin

        if (a_fire)

          state_d = ST_ACQUIRE_D;

      end

      ST_ACQUIRE_D: begin

        if (d_fire) begin

          if ((tl_d_opcode == D_GRANT) ||
              (tl_d_opcode == D_GRANT_DATA))

            state_d = ST_GRANT_INSTALL;

          else

            state_d = ST_LOCAL_RESP;

        end

      end

      //=======================================================================
      // INSTALL GRANT
      //=======================================================================

      ST_GRANT_INSTALL: begin

        if (coh_grant_ready_i)

          state_d =
              grant_has_data_q
              ? ST_GRANT_ACK
              : ST_GRANT_ACK;

      end

      //=======================================================================
      // GRANT ACK
      //=======================================================================

      ST_GRANT_ACK: begin

        if (e_fire)

          state_d = ST_LOCAL_RESP;

      end

      //=======================================================================
      // PROBE LOOKUP
      //=======================================================================

      ST_PROBE_LOOKUP: begin

        if (coh_lookup_ready_i)

          state_d = ST_PROBE_ACTION;

      end

      //=======================================================================
      // PROBE ACTION
      //=======================================================================

      ST_PROBE_ACTION: begin

        if (coh_probe_ready_i)

          state_d = ST_PROBE_C;

      end

      //=======================================================================
      // PROBE RESPONSE
      //=======================================================================

      ST_PROBE_C: begin

        if (c_fire)

          state_d = ST_IDLE;

      end

      //=======================================================================
      // RELEASE
      //=======================================================================

      ST_RELEASE_C: begin

        if (c_fire)

          state_d = ST_RELEASE_D;

      end

      ST_RELEASE_D: begin

        if (d_fire)

          state_d = ST_IDLE;

      end

      //=======================================================================
      // LOCAL RESPONSE
      //=======================================================================

      ST_LOCAL_RESP: begin

        if (cache_rsp_valid_o &&
            cache_rsp_ready_i)

          state_d = ST_IDLE;

      end

      default:

        state_d = ST_IDLE;

    endcase

  end

  //===========================================================================
  // REQUEST CAPTURE
  //===========================================================================

  always_ff @(posedge clk_i or negedge rst_ni) begin

    if (!rst_ni) begin

      state_q <= ST_IDLE;

      req_opcode_q <= '0;
      req_param_q  <= '0;
      req_size_q   <= '0;
      req_addr_q   <= '0;
      req_data_q   <= '0;
      req_mask_q   <= '0;
      req_source_q <= SOURCE_BASE;

      grant_sink_q     <= '0;
      grant_param_q    <= '0;
      grant_has_data_q <= 1'b0;

      local_rsp_opcode_q  <= D_ACCESS_ACK;
      local_rsp_data_q    <= '0;
      local_rsp_denied_q  <= 1'b0;
      local_rsp_corrupt_q <= 1'b0;

    end
    else begin

      state_q <= state_d;

      //=======================================================================
      // Local request
      //=======================================================================

      if (cache_req_valid_i &&
          cache_req_ready_o) begin

        req_opcode_q <= cache_req_opcode_i;
        req_param_q  <= cache_req_param_i;
        req_size_q   <= cache_req_size_i;
        req_addr_q   <= cache_req_addr_i;
        req_data_q   <= cache_req_wdata_i;
        req_mask_q   <= cache_req_be_i;

      end

      //=======================================================================
      // Grant capture
      //=======================================================================

      if (d_fire &&
          ((tl_d_opcode == D_GRANT) ||
           (tl_d_opcode == D_GRANT_DATA))) begin

        grant_sink_q     <= tl_d_sink;
        grant_param_q    <= tl_d_param;

        grant_has_data_q <=
          (tl_d_opcode == D_GRANT_DATA);

        local_rsp_data_q <= tl_d_data;

        local_rsp_denied_q  <= tl_d_denied;
        local_rsp_corrupt_q <= tl_d_corrupt;

      end

      //=======================================================================
      // Normal D response
      //=======================================================================

      if (d_fire &&
          ((tl_d_opcode == D_ACCESS_ACK) ||
           (tl_d_opcode == D_ACCESS_ACK_DATA))) begin

        local_rsp_opcode_q <= tl_d_opcode;

        local_rsp_data_q <= tl_d_data;

        local_rsp_denied_q <= tl_d_denied;

        local_rsp_corrupt_q <= tl_d_corrupt;

      end

      //=======================================================================
      // Probe capture
      //=======================================================================

      if (b_fire) begin

        probe_opcode_q <= tl_b_opcode;
        probe_param_q  <= tl_b_param;
        probe_size_q   <= tl_b_size;
        probe_source_q <= tl_b_source;
        probe_addr_q   <= tl_b_address;
        probe_mask_q   <= tl_b_mask;

      end

      //=======================================================================
      // Cache lookup result
      //=======================================================================

      if (coh_lookup_valid_o &&
          coh_lookup_ready_i) begin

        probe_hit_q   <= coh_hit_i;
        probe_dirty_q <= coh_dirty_i;
        probe_state_q <= coh_state_i;

      end

    end

  end

  //===========================================================================
  // LOCAL CACHE REQUEST READY
  //===========================================================================

  always_comb begin

    cache_req_ready_o = 1'b0;

    if (state_q == ST_IDLE &&
        !tl_b_valid)

      cache_req_ready_o = 1'b1;

  end

  //===========================================================================
  // TILELINK A CHANNEL
  //===========================================================================

  always_comb begin

    tl_a_opcode  = req_opcode_q;
    tl_a_param   = req_param_q;
    tl_a_size    = req_size_q;

    tl_a_source  = req_source_q;

    tl_a_address = req_addr_q;

    tl_a_mask    = req_mask_q;

    tl_a_data    = req_data_q;

    tl_a_corrupt = 1'b0;

    tl_a_valid =
      (state_q == ST_ACQUIRE_A);

  end

  //===========================================================================
  // TILELINK D CHANNEL
  //===========================================================================

  always_comb begin

    tl_d_ready = 1'b0;

    if ((state_q == ST_ACQUIRE_D) ||
        (state_q == ST_RELEASE_D))

      tl_d_ready = 1'b1;

  end

  //===========================================================================
  // TILELINK E CHANNEL
  //===========================================================================

  always_comb begin

    tl_e_sink  = grant_sink_q;

    tl_e_valid =
      (state_q == ST_GRANT_ACK);

  end

  //===========================================================================
  // TILELINK B CHANNEL
  //===========================================================================

  always_comb begin

    tl_b_ready = 1'b0;

    //
    // Only accept a new Probe while completely idle.
    //
    // This single-entry implementation prevents multiple simultaneous
    // coherence actions from conflicting with cache-state updates.
    //

    if (state_q == ST_IDLE &&
        !cache_req_valid_i)

      tl_b_ready = 1'b1;

  end

  //===========================================================================
  // CACHE LOOKUP
  //===========================================================================

  always_comb begin

    coh_lookup_valid_o = 1'b0;

    coh_lookup_addr_o = probe_addr_q;

    if (state_q == ST_PROBE_LOOKUP)

      coh_lookup_valid_o = 1'b1;

  end

  //===========================================================================
  // CACHE PROBE ACTION
  //===========================================================================

  always_comb begin

    coh_probe_valid_o = 1'b0;

    coh_probe_param_o = probe_param_q;

    if (state_q == ST_PROBE_ACTION)

      coh_probe_valid_o = 1'b1;

  end

  //===========================================================================
  // CACHE DATA VALID
  //===========================================================================

  always_comb begin

    coh_data_valid_o = 1'b0;

    if (state_q == ST_PROBE_C &&
        probe_dirty_q)

      coh_data_valid_o = 1'b1;

  end

  //===========================================================================
  // CACHE GRANT INSTALL
  //===========================================================================

  always_comb begin

    coh_grant_valid_o = 1'b0;

    coh_grant_addr_o  = req_addr_q;

    coh_grant_data_o  = local_rsp_data_q;

    coh_grant_param_o = grant_param_q;

    if (state_q == ST_GRANT_INSTALL)

      coh_grant_valid_o = 1'b1;

  end

  //===========================================================================
  // TILELINK C CHANNEL
  //===========================================================================

  always_comb begin

    tl_c_opcode  = C_PROBE_ACK;

    tl_c_param   = probe_param_q;

    tl_c_size    = probe_size_q;

    tl_c_source  = probe_source_q;

    tl_c_address = probe_addr_q;

    tl_c_data    = coh_rdata_i;

    tl_c_corrupt = 1'b0;

    //-----------------------------------------------------------------------
    // ProbeAck / ProbeAckData
    //-----------------------------------------------------------------------

    if (state_q == ST_PROBE_C) begin

      if (probe_dirty_q)

        tl_c_opcode = C_PROBE_ACK_DATA;

      else

        tl_c_opcode = C_PROBE_ACK;

    end

  end

  assign tl_c_valid =
    (state_q == ST_PROBE_C);

  //===========================================================================
  // LOCAL RESPONSE
  //===========================================================================

  always_comb begin

    cache_rsp_valid_o =
      (state_q == ST_LOCAL_RESP);

    cache_rsp_opcode_o =
      local_rsp_opcode_q;

    cache_rsp_data_o =
      local_rsp_data_q;

    cache_rsp_denied_o =
      local_rsp_denied_q;

    cache_rsp_corrupt_o =
      local_rsp_corrupt_q;

  end

  //===========================================================================
  // PROTOCOL ASSERTIONS
  //===========================================================================

  generate

    if (ENABLE_ASSERTIONS) begin : gen_tl_c_sva

      //=======================================================================
      // A channel stability
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
          tl_a_data,
          tl_a_corrupt
        });

      endproperty

      assert property (p_a_stable)
        else
          $error(
            "TL-C MASTER: A channel changed while stalled"
          );

      //=======================================================================
      // B channel stability
      //=======================================================================

      property p_b_stable;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_b_valid &&
        !tl_b_ready

        |->
        $stable({
          tl_b_opcode,
          tl_b_param,
          tl_b_size,
          tl_b_source,
          tl_b_address,
          tl_b_mask,
          tl_b_data,
          tl_b_corrupt
        });

      endproperty

      assert property (p_b_stable)
        else
          $error(
            "TL-C MASTER: B channel changed while stalled"
          );

      //=======================================================================
      // C channel stability
      //=======================================================================

      property p_c_stable;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_c_valid &&
        !tl_c_ready

        |->
        $stable({
          tl_c_opcode,
          tl_c_param,
          tl_c_size,
          tl_c_source,
          tl_c_address,
          tl_c_data,
          tl_c_corrupt
        });

      endproperty

      assert property (p_c_stable)
        else
          $error(
            "TL-C MASTER: C channel changed while stalled"
          );

      //=======================================================================
      // E channel stability
      //=======================================================================

      property p_e_stable;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_e_valid &&
        !tl_e_ready

        |->
        $stable(tl_e_sink);

      endproperty

      assert property (p_e_stable)
        else
          $error(
            "TL-C MASTER: E sink changed while stalled"
          );

      //=======================================================================
      // Grant must be followed by GrantAck
      //=======================================================================

      property p_grant_ack;

        @(posedge clk_i)
        disable iff (!rst_ni)

        d_fire &&
        ((tl_d_opcode == D_GRANT) ||
         (tl_d_opcode == D_GRANT_DATA))

        |->
        ##[1:$]
        tl_e_valid;

      endproperty

      assert property (p_grant_ack)
        else
          $error(
            "TL-C MASTER: Grant not acknowledged"
          );

      //=======================================================================
      // E only used for Grant
      //=======================================================================

      property p_e_only_grant;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_e_valid
        |->
        (state_q == ST_GRANT_ACK);

      endproperty

      assert property (p_e_only_grant)
        else
          $error(
            "TL-C MASTER: E asserted outside GrantAck state"
          );

      //=======================================================================
      // Probe response must use ProbeAck/ProbeAckData
      //=======================================================================

      property p_probe_response;

        @(posedge clk_i)
        disable iff (!rst_ni)

        c_fire &&
        ((state_q == ST_PROBE_C))

        |->
        (
          (tl_c_opcode == C_PROBE_ACK) ||
          (tl_c_opcode == C_PROBE_ACK_DATA)
        );

      endproperty

      assert property (p_probe_response)
        else
          $error(
            "TL-C MASTER: invalid Probe response"
          );

      //=======================================================================
      // Probe source preserved
      //=======================================================================

      property p_probe_source;

        @(posedge clk_i)
        disable iff (!rst_ni)

        c_fire &&
        (state_q == ST_PROBE_C)

        |->
        (tl_c_source == probe_source_q);

      endproperty

      assert property (p_probe_source)
        else
          $error(
            "TL-C MASTER: Probe source mismatch"
          );

      //=======================================================================
      // Probe address preserved
      //=======================================================================

      property p_probe_address;

        @(posedge clk_i)
        disable iff (!rst_ni)

        c_fire &&
        (state_q == ST_PROBE_C)

        |->
        (tl_c_address == probe_addr_q);

      endproperty

      assert property (p_probe_address)
        else
          $error(
            "TL-C MASTER: Probe address mismatch"
          );

      //=======================================================================
      // D source must match outstanding A source
      //=======================================================================

      property p_d_source;

        @(posedge clk_i)
        disable iff (!rst_ni)

        d_fire &&
        (state_q == ST_ACQUIRE_D)

        |->
        (tl_d_source == req_source_q);

      endproperty

      assert property (p_d_source)
        else
          $error(
            "TL-C MASTER: D source mismatch"
          );

      //=======================================================================
      // C source is local source for ProbeAck
      //=======================================================================

      property p_c_source_valid;

        @(posedge clk_i)
        disable iff (!rst_ni)

        c_fire &&
        (state_q == ST_PROBE_C)

        |->
        (tl_c_source == probe_source_q);

      endproperty

      assert property (p_c_source_valid)
        else
          $error(
            "TL-C MASTER: illegal C source"
          );

      //=======================================================================
      // C corrupt is allowed only on data-bearing messages
      //=======================================================================

      property p_c_corrupt;

        @(posedge clk_i)
        disable iff (!rst_ni)

        c_fire &&
        (tl_c_opcode == C_PROBE_ACK)

        |->
        !tl_c_corrupt;

      endproperty

      assert property (p_c_corrupt)
        else
          $error(
            "TL-C MASTER: ProbeAck cannot be corrupt"
          );

      //=======================================================================
      // Grant data consistency
      //=======================================================================

      property p_grant_data;

        @(posedge clk_i)
        disable iff (!rst_ni)

        d_fire &&
        (tl_d_opcode == D_GRANT)

        |->
        !tl_d_corrupt;

      endproperty

      assert property (p_grant_data)
        else
          $error(
            "TL-C MASTER: Grant without data must not be corrupt"
          );

      //=======================================================================
      // No spurious C
      //=======================================================================

      property p_c_state;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_c_valid
        |->
        (state_q == ST_PROBE_C);

      endproperty

      assert property (p_c_state)
        else
          $error(
            "TL-C MASTER: C valid outside C state"
          );

      //=======================================================================
      // No spurious E
      //=======================================================================

      property p_e_state;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_e_valid
        |->
        (state_q == ST_GRANT_ACK);

      endproperty

      assert property (p_e_state)
        else
          $error(
            "TL-C MASTER: E valid outside E state"
          );

      //=======================================================================
      // Acquire alignment
      //=======================================================================

      property p_acquire_alignment;

        @(posedge clk_i)
        disable iff (!rst_ni)

        a_fire &&
        ((tl_a_opcode == A_ACQUIRE_BLOCK) ||
         (tl_a_opcode == A_ACQUIRE_PERM))

        |->
        (
          (tl_a_address &
           ((1 << tl_a_size) - 1))
          == 0
        );

      endproperty

      assert property (p_acquire_alignment)
        else
          $error(
            "TL-C MASTER: Acquire address is not aligned"
          );

      //=======================================================================
      // B probe alignment
      //=======================================================================

      property p_probe_alignment;

        @(posedge clk_i)
        disable iff (!rst_ni)

        b_fire

        |->
        (
          (tl_b_address &
           ((1 << tl_b_size) - 1))
          == 0
        );

      endproperty

      assert property (p_probe_alignment)
        else
          $error(
            "TL-C MASTER: Probe address is not aligned"
          );

    end

  endgenerate

endmodule
