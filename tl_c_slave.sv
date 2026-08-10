//=============================================================================
// File Name   : tl_c_slave.sv
//
// Description :  TileLink Coherent Manager / Slave Endpoint
// Author      :  Ashish Pradhan
//
// TileLink 1.8.x style implementation
//
// Channels
// --------
// A : Client -> Manager
// B : Manager -> Client
// C : Client -> Manager
// D : Manager -> Client
// E : Client -> Manager
//
// Supported A:
//   Get
//   PutFullData
//   PutPartialData
//   AcquireBlock
//   AcquirePerm
//
// Supported B:
//   ProbeBlock
//   ProbePerm
//
// Supported C:
//   ProbeAck
//   ProbeAckData
//   Release
//   ReleaseData
//
// Supported D:
//   AccessAck
//   AccessAckData
//   Grant
//   GrantData
//   ReleaseAck
//
// Supported E:
//   GrantAck
//
// Design characteristics
// ----------------------
// * Single outstanding manager transaction
// * Full valid/ready backpressure
// * Registered transaction state
// * Stable channels while stalled
// * Source ID preservation
// * Sink ID generation
// * Acquire / Grant protocol
// * GrantAck tracking
// * Probe generation
// * ProbeAck / ProbeAckData handling
// * Release / ReleaseData handling
// * Memory backend interface
// * Coherence directory interface
// * Error propagation
// * Optional assertions
//
// NOTE:
// This module is a protocol endpoint. A production cache-coherence system
// should place the actual directory/tag/data-array implementation behind
// coh_* interfaces.
//
//=============================================================================

module tl_c_slave #(

  parameter int unsigned ADDR_WIDTH   = 40,
  parameter int unsigned DATA_WIDTH   = 64,
  parameter int unsigned SOURCE_WIDTH = 6,
  parameter int unsigned SINK_WIDTH   = 4,

  parameter int unsigned MAX_SIZE     = 6,

  parameter bit ENABLE_ASSERTIONS = 1'b1,

  parameter bit ENABLE_GET         = 1'b1,
  parameter bit ENABLE_PUT         = 1'b1,
  parameter bit ENABLE_ACQUIRE     = 1'b1,
  parameter bit ENABLE_RELEASE     = 1'b1,
  parameter bit ENABLE_PROBE       = 1'b1

) (

  input  logic clk_i,
  input  logic rst_ni,

  //===========================================================================
  // LOCAL MEMORY INTERFACE
  //===========================================================================

  output logic                         mem_req_valid_o,
  input  logic                         mem_req_ready_i,

  output logic                         mem_req_write_o,

  output logic [ADDR_WIDTH-1:0]        mem_req_addr_o,

  output logic [DATA_WIDTH-1:0]         mem_req_wdata_o,

  output logic [DATA_WIDTH/8-1:0]       mem_req_be_o,

  input  logic                         mem_rsp_valid_i,
  output logic                         mem_rsp_ready_o,

  input  logic [DATA_WIDTH-1:0]         mem_rsp_rdata_i,

  input  logic                         mem_rsp_error_i,

  //===========================================================================
  // COHERENCE DIRECTORY INTERFACE
  //===========================================================================

  // Lookup a cache line before generating a Probe.
  output logic                         coh_lookup_valid_o,
  input  logic                         coh_lookup_ready_i,

  output logic [ADDR_WIDTH-1:0]         coh_lookup_addr_o,

  input  logic                         coh_line_present_i,
  input  logic                         coh_line_dirty_i,

  // Request invalidation / downgrade of a cached line.
  output logic                         coh_probe_valid_o,
  input  logic                         coh_probe_ready_i,

  output logic [ADDR_WIDTH-1:0]         coh_probe_addr_o,
  output logic [2:0]                   coh_probe_param_o,

  // Probe completion.
  input  logic                         coh_probe_done_i,

  // Release/writeback from client.
  output logic                         coh_release_valid_o,
  input  logic                         coh_release_ready_i,

  output logic [ADDR_WIDTH-1:0]         coh_release_addr_o,
  output logic [2:0]                   coh_release_param_o,

  // Grant allocation.
  output logic                         coh_grant_valid_o,
  input  logic                         coh_grant_ready_i,

  output logic [ADDR_WIDTH-1:0]         coh_grant_addr_o,
  output logic [2:0]                   coh_grant_param_o,

  //===========================================================================
  // TILELINK A
  //===========================================================================

  input  logic [2:0]                    tl_a_opcode,
  input  logic [2:0]                    tl_a_param,
  input  logic [SIZE_WIDTH-1:0]         tl_a_size,
  input  logic [SOURCE_WIDTH-1:0]       tl_a_source,
  input  logic [ADDR_WIDTH-1:0]         tl_a_address,
  input  logic [DATA_WIDTH/8-1:0]       tl_a_mask,
  input  logic [DATA_WIDTH-1:0]         tl_a_data,
  input  logic                          tl_a_corrupt,
  input  logic                          tl_a_valid,
  output logic                          tl_a_ready,

  //===========================================================================
  // TILELINK B
  //===========================================================================

  output logic [2:0]                    tl_b_opcode,
  output logic [2:0]                    tl_b_param,
  output logic [SIZE_WIDTH-1:0]         tl_b_size,
  output logic [SOURCE_WIDTH-1:0]       tl_b_source,
  output logic [ADDR_WIDTH-1:0]         tl_b_address,
  output logic [DATA_WIDTH/8-1:0]       tl_b_mask,
  output logic [DATA_WIDTH-1:0]         tl_b_data,
  output logic                          tl_b_corrupt,
  output logic                          tl_b_valid,
  input logic                           tl_b_ready,

  //===========================================================================
  // TILELINK C
  //===========================================================================

  input logic [2:0]                    tl_c_opcode,
  input logic [2:0]                    tl_c_param,
  input logic [SIZE_WIDTH-1:0]         tl_c_size,
  input logic [SOURCE_WIDTH-1:0]       tl_c_source,
  input logic [ADDR_WIDTH-1:0]         tl_c_address,
  input logic [DATA_WIDTH-1:0]         tl_c_data,
  input logic                          tl_c_corrupt,
  input logic                          tl_c_valid,
  output logic                         tl_c_ready,

  //===========================================================================
  // TILELINK D
  //===========================================================================

  output logic [2:0]                    tl_d_opcode,
  output logic [1:0]                    tl_d_param,
  output logic [SIZE_WIDTH-1:0]         tl_d_size,
  output logic [SOURCE_WIDTH-1:0]       tl_d_source,
  output logic [SINK_WIDTH-1:0]         tl_d_sink,
  output logic                          tl_d_denied,
  output logic [DATA_WIDTH-1:0]         tl_d_data,
  output logic                          tl_d_corrupt,
  output logic                          tl_d_valid,
  input logic                           tl_d_ready,

  //===========================================================================
  // TILELINK E
  //===========================================================================

  input logic [SINK_WIDTH-1:0]          tl_e_sink,
  input logic                           tl_e_valid,
  output logic                          tl_e_ready

);

  //===========================================================================
  // DERIVED PARAMETERS
  //===========================================================================

  localparam int unsigned BYTE_LANES =
      DATA_WIDTH / 8;

  localparam int unsigned SIZE_WIDTH_LOCAL =
      (BYTE_LANES <= 1)
      ? 1
      : $clog2(MAX_SIZE + 1);

  localparam int unsigned SOURCE_TABLE_DEPTH =
      (1 << SOURCE_WIDTH);

  //===========================================================================
  // TL OPCODES
  //===========================================================================

  localparam logic [2:0] A_PUTFULL       = 3'd0;
  localparam logic [2:0] A_PUTPARTIAL    = 3'd1;
  localparam logic [2:0] A_ARITHMETIC    = 3'd2;
  localparam logic [2:0] A_LOGICAL       = 3'd3;
  localparam logic [2:0] A_GET           = 3'd4;
  localparam logic [2:0] A_INTENT        = 3'd5;
  localparam logic [2:0] A_ACQUIRE_BLOCK = 3'd6;
  localparam logic [2:0] A_ACQUIRE_PERM  = 3'd7;

  localparam logic [2:0] B_PROBE_BLOCK   = 3'd6;
  localparam logic [2:0] B_PROBE_PERM    = 3'd7;

  localparam logic [2:0] C_PROBE_ACK     = 3'd4;
  localparam logic [2:0] C_PROBE_ACKDATA = 3'd5;
  localparam logic [2:0] C_RELEASE       = 3'd6;
  localparam logic [2:0] C_RELEASEDATA   = 3'd7;

  localparam logic [2:0] D_ACCESS_ACK     = 3'd0;
  localparam logic [2:0] D_ACCESS_ACKDATA = 3'd1;
  localparam logic [2:0] D_HINT_ACK       = 3'd2;
  localparam logic [2:0] D_GRANT          = 3'd4;
  localparam logic [2:0] D_GRANTDATA      = 3'd5;
  localparam logic [2:0] D_RELEASE_ACK    = 3'd6;

  //===========================================================================
  // STATE MACHINE
  //===========================================================================

  typedef enum logic [4:0] {

    ST_IDLE,

    // A channel
    ST_A_MEM_REQ,
    ST_A_MEM_WAIT,

    ST_A_PROBE_LOOKUP,
    ST_A_PROBE_SEND,
    ST_A_PROBE_WAIT,

    ST_A_D_SEND,
    ST_A_D_WAIT_E,

    // C channel
    ST_C_MEM_REQ,
    ST_C_MEM_WAIT,
    ST_C_D_SEND,

    // Probe request
    ST_B_SEND,
    ST_B_WAIT_C,
    ST_B_MEM_REQ,
    ST_B_MEM_WAIT,
    ST_B_COMPLETE,

    // E
    ST_E_WAIT,

    // Error
    ST_ERROR

  } state_t;

  state_t state_q;
  state_t state_d;

  //===========================================================================
  // REQUEST REGISTERS
  //===========================================================================

  logic [2:0]                    req_opcode_q;
  logic [2:0]                    req_param_q;
  logic [SIZE_WIDTH-1:0]         req_size_q;
  logic [SOURCE_WIDTH-1:0]       req_source_q;

  logic [ADDR_WIDTH-1:0]         req_addr_q;

  logic [DATA_WIDTH/8-1:0]       req_mask_q;
  logic [DATA_WIDTH-1:0]         req_data_q;

  logic                          req_corrupt_q;

  //===========================================================================
  // RESPONSE REGISTERS
  //===========================================================================

  logic [2:0]                    rsp_opcode_q;
  logic [1:0]                    rsp_param_q;

  logic [SIZE_WIDTH-1:0]         rsp_size_q;

  logic [SOURCE_WIDTH-1:0]       rsp_source_q;

  logic [SINK_WIDTH-1:0]         rsp_sink_q;

  logic [DATA_WIDTH-1:0]         rsp_data_q;

  logic                          rsp_denied_q;
  logic                          rsp_corrupt_q;

  //===========================================================================
  // PROBE REGISTERS
  //===========================================================================

  logic [2:0]                    probe_opcode_q;
  logic [2:0]                    probe_param_q;
  logic [SIZE_WIDTH-1:0]         probe_size_q;

  logic [SOURCE_WIDTH-1:0]       probe_source_q;

  logic [ADDR_WIDTH-1:0]         probe_addr_q;

  logic [DATA_WIDTH/8-1:0]       probe_mask_q;

  logic                          probe_line_present_q;
  logic                          probe_line_dirty_q;

  logic [DATA_WIDTH-1:0]         probe_data_q;

  //===========================================================================
  // RELEASE REGISTERS
  //===========================================================================

  logic [2:0]                    release_opcode_q;
  logic [2:0]                    release_param_q;

  logic [SIZE_WIDTH-1:0]         release_size_q;

  logic [SOURCE_WIDTH-1:0]       release_source_q;

  logic [ADDR_WIDTH-1:0]         release_addr_q;

  logic [DATA_WIDTH-1:0]         release_data_q;

  logic                          release_corrupt_q;

  //===========================================================================
  // SINK ALLOCATION
  //===========================================================================

  logic [SINK_WIDTH-1:0]         sink_counter_q;

  //===========================================================================
  // HANDSHAKES
  //===========================================================================

  logic a_fire;
  logic b_fire;
  logic c_fire;
  logic d_fire;
  logic e_fire;

  logic mem_req_fire;
  logic mem_rsp_fire;

  assign a_fire =
      tl_a_valid &&
      tl_a_ready;

  assign b_fire =
      tl_b_valid &&
      tl_b_ready;

  assign c_fire =
      tl_c_valid &&
      tl_c_ready;

  assign d_fire =
      tl_d_valid &&
      tl_d_ready;

  assign e_fire =
      tl_e_valid &&
      tl_e_ready;

  assign mem_req_fire =
      mem_req_valid_o &&
      mem_req_ready_i;

  assign mem_rsp_fire =
      mem_rsp_valid_i &&
      mem_rsp_ready_o;

  //===========================================================================
  // REQUEST CLASSIFICATION
  //===========================================================================

  logic req_is_get;
  logic req_is_put;
  logic req_is_acquire;

  always_comb begin

    req_is_get     = 1'b0;
    req_is_put     = 1'b0;
    req_is_acquire = 1'b0;

    unique case (req_opcode_q)

      A_GET:
        req_is_get = 1'b1;

      A_PUTFULL,
      A_PUTPARTIAL:
        req_is_put = 1'b1;

      A_ACQUIRE_BLOCK,
      A_ACQUIRE_PERM:
        req_is_acquire = 1'b1;

      default:
        begin
          req_is_get     = 1'b0;
          req_is_put     = 1'b0;
          req_is_acquire = 1'b0;
        end

    endcase

  end

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

        if (tl_a_valid &&
            tl_a_ready)

          state_d = ST_A_MEM_REQ;

        else if (tl_c_valid &&
                 tl_c_ready)

          state_d = ST_C_MEM_REQ;

        else if (ENABLE_PROBE &&
                 coh_probe_valid_o)

          state_d = ST_B_SEND;

      end

      //=======================================================================
      // A MEMORY REQUEST
      //=======================================================================

      ST_A_MEM_REQ: begin

        if (req_is_acquire) begin

          state_d = ST_A_PROBE_LOOKUP;

        end
        else if (mem_req_fire) begin

          if (req_is_get ||
              req_is_put)

            state_d =
              req_is_get
              ? ST_A_MEM_WAIT
              : ST_A_D_SEND;

          else

            state_d = ST_A_D_SEND;

        end

      end

      //=======================================================================
      // A MEMORY WAIT
      //=======================================================================

      ST_A_MEM_WAIT: begin

        if (mem_rsp_fire)

          state_d = ST_A_D_SEND;

      end

      //=======================================================================
      // ACQUIRE PROBE LOOKUP
      //=======================================================================

      ST_A_PROBE_LOOKUP: begin

        if (coh_lookup_ready_i)

          state_d = ST_A_PROBE_SEND;

      end

      //=======================================================================
      // ACQUIRE PROBE SEND
      //=======================================================================

      ST_A_PROBE_SEND: begin

        if (coh_probe_ready_i) begin

          if (probe_line_present_q)

            state_d = ST_A_PROBE_WAIT;

          else

            state_d = ST_A_D_SEND;

        end

      end

      //=======================================================================
      // ACQUIRE PROBE WAIT
      //=======================================================================

      ST_A_PROBE_WAIT: begin

        if (coh_probe_done_i)

          state_d = ST_A_D_SEND;

      end

      //=======================================================================
      // D RESPONSE
      //=======================================================================

      ST_A_D_SEND: begin

        if (d_fire) begin

          if (req_is_acquire)

            state_d = ST_A_D_WAIT_E;

          else

            state_d = ST_IDLE;

        end

      end

      //=======================================================================
      // WAIT GRANTACK
      //=======================================================================

      ST_A_D_WAIT_E: begin

        if (e_fire)

          state_d = ST_IDLE;

      end

      //=======================================================================
      // C MEMORY REQUEST
      //=======================================================================

      ST_C_MEM_REQ: begin

        if (release_opcode_q == C_RELEASEDATA ||
            release_opcode_q == C_RELEASE) begin

          if (release_opcode_q == C_RELEASEDATA)

            state_d = ST_C_MEM_WAIT;

          else

            state_d = ST_C_D_SEND;

        end

      end

      //=======================================================================
      // C MEMORY WAIT
      //=======================================================================

      ST_C_MEM_WAIT: begin

        if (mem_rsp_fire)

          state_d = ST_C_D_SEND;

      end

      //=======================================================================
      // C D RESPONSE
      //=======================================================================

      ST_C_D_SEND: begin

        if (d_fire)

          state_d = ST_IDLE;

      end

      //=======================================================================
      // B PROBE SEND
      //=======================================================================

      ST_B_SEND: begin

        if (b_fire)

          state_d = ST_B_WAIT_C;

      end

      //=======================================================================
      // WAIT PROBE ACK
      //=======================================================================

      ST_B_WAIT_C: begin

        if (c_fire) begin

          if ((tl_c_opcode == C_PROBE_ACKDATA) ||
              (tl_c_opcode == C_RELEASEDATA))

            state_d = ST_B_MEM_REQ;

          else

            state_d = ST_B_COMPLETE;

        end

      end

      //=======================================================================
      // PROBE WRITEBACK
      //=======================================================================

      ST_B_MEM_REQ: begin

        if (mem_req_fire)

          state_d = ST_B_MEM_WAIT;

      end

      ST_B_MEM_WAIT: begin

        if (mem_rsp_fire)

          state_d = ST_B_COMPLETE;

      end

      //=======================================================================
      // PROBE COMPLETE
      //=======================================================================

      ST_B_COMPLETE: begin

        state_d = ST_IDLE;
      end

      //=======================================================================
      // ERROR
      //=======================================================================

      ST_ERROR: begin

        if (d_fire)

          state_d = ST_IDLE;

      end

      default:

        state_d = ST_IDLE;

    endcase

  end

  //===========================================================================
  // REGISTERED REQUEST / CHANNEL STATE
  //===========================================================================

  always_ff @(posedge clk_i or negedge rst_ni) begin

    if (!rst_ni) begin

      state_q <= ST_IDLE;

      req_opcode_q  <= '0;
      req_param_q   <= '0;
      req_size_q    <= '0;
      req_source_q  <= '0;
      req_addr_q    <= '0;
      req_mask_q    <= '0;
      req_data_q    <= '0;
      req_corrupt_q <= 1'b0;

      rsp_opcode_q  <= D_ACCESS_ACK;
      rsp_param_q   <= '0;
      rsp_size_q    <= '0;
      rsp_source_q  <= '0;
      rsp_sink_q    <= '0;
      rsp_data_q    <= '0;
      rsp_denied_q  <= 1'b0;
      rsp_corrupt_q <= 1'b0;

      probe_opcode_q        <= B_PROBE_BLOCK;
      probe_param_q         <= '0;
      probe_size_q          <= '0;
      probe_source_q        <= '0;
      probe_addr_q          <= '0;
      probe_mask_q          <= '0;
      probe_line_present_q  <= 1'b0;
      probe_line_dirty_q    <= 1'b0;
      probe_data_q          <= '0;

      release_opcode_q  <= C_RELEASE;
      release_param_q   <= '0;
      release_size_q    <= '0;
      release_source_q  <= '0;
      release_addr_q    <= '0;
      release_data_q    <= '0;
      release_corrupt_q <= 1'b0;

      sink_counter_q <= '0;

    end
    else begin

      state_q <= state_d;

      //=======================================================================
      // A capture
      //=======================================================================

      if (a_fire) begin

        req_opcode_q  <= tl_a_opcode;
        req_param_q   <= tl_a_param;
        req_size_q    <= tl_a_size;
        req_source_q  <= tl_a_source;
        req_addr_q    <= tl_a_address;
        req_mask_q    <= tl_a_mask;
        req_data_q    <= tl_a_data;
        req_corrupt_q <= tl_a_corrupt;

        rsp_source_q <= tl_a_source;
        rsp_size_q   <= tl_a_size;

        // Allocate a sink for coherent Acquire.
        if ((tl_a_opcode == A_ACQUIRE_BLOCK) ||
            (tl_a_opcode == A_ACQUIRE_PERM)) begin

          rsp_sink_q <= sink_counter_q;

          sink_counter_q <=
            sink_counter_q + 1'b1;

        end

      end

      //=======================================================================
      // Memory response
      //=======================================================================

      if (mem_rsp_fire) begin

        rsp_data_q <= mem_rsp_rdata_i;

        rsp_denied_q <=
          mem_rsp_error_i;

      end

      //=======================================================================
      // Probe lookup result
      //=======================================================================

      if (state_q == ST_A_PROBE_LOOKUP &&
          coh_lookup_ready_i) begin

        probe_line_present_q <=
          coh_line_present_i;

        probe_line_dirty_q <=
          coh_line_dirty_i;

      end

      //=======================================================================
      // Incoming C release / probe response
      //=======================================================================

      if (c_fire) begin

        release_opcode_q  <= tl_c_opcode;
        release_param_q   <= tl_c_param;
        release_size_q    <= tl_c_size;
        release_source_q  <= tl_c_source;
        release_addr_q    <= tl_c_address;
        release_data_q    <= tl_c_data;
        release_corrupt_q <= tl_c_corrupt;

        probe_data_q <= tl_c_data;

      end

    end

  end

  //===========================================================================
  // A CHANNEL READY
  //===========================================================================

  always_comb begin

    tl_a_ready = 1'b0;

    if (state_q == ST_IDLE)

      tl_a_ready = 1'b1;

  end

  //===========================================================================
  // A MEMORY REQUEST
  //===========================================================================

  always_comb begin

    mem_req_valid_o = 1'b0;

    mem_req_write_o = 1'b0;

    mem_req_addr_o  = req_addr_q;

    mem_req_wdata_o = req_data_q;

    mem_req_be_o    = req_mask_q;

    // Normal GET/PUT.
    if (state_q == ST_A_MEM_REQ) begin

      if (!req_is_acquire) begin

        mem_req_valid_o = 1'b1;

        mem_req_write_o = req_is_put;

      end

    end

    // Writeback resulting from ProbeAckData.
    else if (state_q == ST_B_MEM_REQ) begin

      mem_req_valid_o = 1'b1;

      mem_req_write_o = 1'b1;

      mem_req_addr_o  = probe_addr_q;

      mem_req_wdata_o = probe_data_q;

      mem_req_be_o    = {BYTE_LANES{1'b1}};

    end

    // ReleaseData writeback.
    else if (state_q == ST_C_MEM_REQ) begin

      if (release_opcode_q == C_RELEASEDATA) begin

        mem_req_valid_o = 1'b1;

        mem_req_write_o = 1'b1;

        mem_req_addr_o  = release_addr_q;

        mem_req_wdata_o = release_data_q;

        mem_req_be_o    = {BYTE_LANES{1'b1}};

      end

    end

  end

  //===========================================================================
  // MEMORY RESPONSE READY
  //===========================================================================

  always_comb begin

    mem_rsp_ready_o = 1'b0;

    if ((state_q == ST_A_MEM_WAIT) ||
        (state_q == ST_B_MEM_WAIT) ||
        (state_q == ST_C_MEM_WAIT))

      mem_rsp_ready_o = 1'b1;

  end

  //===========================================================================
  // COHERENCE LOOKUP
  //===========================================================================

  always_comb begin

    coh_lookup_valid_o = 1'b0;

    coh_lookup_addr_o = req_addr_q;

    if (state_q == ST_A_PROBE_LOOKUP)

      coh_lookup_valid_o = 1'b1;

  end

  //===========================================================================
  // COHERENCE PROBE
  //===========================================================================

  always_comb begin

    coh_probe_valid_o = 1'b0;

    coh_probe_addr_o  = req_addr_q;

    coh_probe_param_o = req_param_q;

    if (state_q == ST_A_PROBE_SEND)

      coh_probe_valid_o = 1'b1;

  end

  //===========================================================================
  // GRANT
  //===========================================================================

  always_comb begin

    coh_grant_valid_o = 1'b0;

    coh_grant_addr_o  = req_addr_q;

    coh_grant_param_o = req_param_q;

    if (state_q == ST_A_D_SEND &&
        req_is_acquire)

      coh_grant_valid_o = 1'b1;

  end

  //===========================================================================
  // C CHANNEL
  //===========================================================================

  always_comb begin

    tl_c_ready = 1'b0;

    // Accept ProbeAck / ProbeAckData / Release / ReleaseData.
    if ((state_q == ST_IDLE) ||
        (state_q == ST_B_WAIT_C))

      tl_c_ready = 1'b1;

  end

  //===========================================================================
  // B CHANNEL
  //===========================================================================

  always_comb begin

    tl_b_opcode  = probe_opcode_q;

    tl_b_param   = probe_param_q;

    tl_b_size    = probe_size_q;

    tl_b_source  = probe_source_q;

    tl_b_address = probe_addr_q;

    tl_b_mask    = {BYTE_LANES{1'b1}};

    tl_b_data    = '0;

    tl_b_corrupt = 1'b0;

    tl_b_valid =
      (state_q == ST_B_SEND);

  end

  //===========================================================================
  // D CHANNEL
  //===========================================================================

  always_comb begin

    tl_d_opcode  = rsp_opcode_q;

    tl_d_param   = rsp_param_q;

    tl_d_size    = rsp_size_q;

    tl_d_source  = rsp_source_q;

    tl_d_sink    = rsp_sink_q;

    tl_d_denied  = rsp_denied_q;

    tl_d_data    = rsp_data_q;

    tl_d_corrupt = rsp_corrupt_q;

    tl_d_valid = 1'b0;

    //-----------------------------------------------------------------------
    // Normal GET
    //-----------------------------------------------------------------------

    if (state_q == ST_A_D_SEND) begin

      if (req_is_get)

        tl_d_opcode = D_ACCESS_ACKDATA;

      else if (req_is_put)

        tl_d_opcode = D_ACCESS_ACK;

      //---------------------------------------------------------------------
      // Acquire
      //---------------------------------------------------------------------

      else if (req_is_acquire) begin

        if (rsp_data_q != '0)

          tl_d_opcode = D_GRANTDATA;

        else

          tl_d_opcode = D_GRANT;

      end

      tl_d_valid = 1'b1;

    end

    //-----------------------------------------------------------------------
    // ReleaseAck
    //-----------------------------------------------------------------------

    else if (state_q == ST_C_D_SEND) begin

      tl_d_opcode = D_RELEASE_ACK;

      tl_d_valid  = 1'b1;

      tl_d_source = release_source_q;

      tl_d_size   = release_size_q;

      tl_d_sink   = '0;

      tl_d_data   = '0;

      tl_d_denied =
        release_corrupt_q;

    end

  end

  //===========================================================================
  // E CHANNEL
  //===========================================================================

  always_comb begin

    tl_e_ready = 1'b0;

    if (state_q == ST_A_D_WAIT_E)

      tl_e_ready = 1'b1;

  end

  //===========================================================================
  // RELEASE / PROBE DIRECTORY INTERFACE
  //===========================================================================

  always_comb begin

    coh_release_valid_o = 1'b0;

    coh_release_addr_o  = release_addr_q;

    coh_release_param_o = release_param_q;

    if (state_q == ST_B_COMPLETE)

      coh_release_valid_o = 1'b1;

  end

  //===========================================================================
  // PROTOCOL ASSERTIONS
  //===========================================================================

  generate

    if (ENABLE_ASSERTIONS) begin : gen_tl_c_slave_sva

      //=======================================================================
      // A stability
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
            "TL-C SLAVE: A changed while stalled"
          );

      //=======================================================================
      // B stability
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
            "TL-C SLAVE: B changed while stalled"
          );

      //=======================================================================
      // C stability
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
            "TL-C SLAVE: C changed while stalled"
          );

      //=======================================================================
      // D stability
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
            "TL-C SLAVE: D changed while stalled"
          );

      //=======================================================================
      // E ready only during GrantAck
      //=======================================================================

      property p_e_ready;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_e_ready
        |->
        (state_q == ST_A_D_WAIT_E);

      endproperty

      assert property (p_e_ready)
        else
          $error(
            "TL-C SLAVE: E ready in illegal state"
          );

      //=======================================================================
      // Grant must be acknowledged
      //=======================================================================

      property p_grant_requires_ack;

        @(posedge clk_i)
        disable iff (!rst_ni)

        d_fire &&
        ((tl_d_opcode == D_GRANT) ||
         (tl_d_opcode == D_GRANTDATA))

        |->
        ##1
        (
          (state_q == ST_A_D_WAIT_E) ||
          (state_q == ST_A_D_SEND)
        );

      endproperty

      assert property (p_grant_requires_ack)
        else
          $error(
            "TL-C SLAVE: GrantAck sequencing failure"
          );

      //=======================================================================
      // E sink match
      //=======================================================================

      property p_e_sink_match;

        @(posedge clk_i)
        disable iff (!rst_ni)

        e_fire
        |->
        (tl_e_sink == rsp_sink_q);

      endproperty

      assert property (p_e_sink_match)
        else
          $error(
            "TL-C SLAVE: GrantAck sink mismatch"
          );

      //=======================================================================
      // Probe opcode
      //=======================================================================

      property p_b_probe_opcode;

        @(posedge clk_i)
        disable iff (!rst_ni)

        b_fire
        |->
        (
          (tl_b_opcode == B_PROBE_BLOCK) ||
          (tl_b_opcode == B_PROBE_PERM)
        );

      endproperty

      assert property (p_b_probe_opcode)
        else
          $error(
            "TL-C SLAVE: invalid B opcode"
          );

      //=======================================================================
      // C response opcode
      //=======================================================================

      property p_c_opcode;

        @(posedge clk_i)
        disable iff (!rst_ni)

        c_fire &&
        (state_q == ST_B_WAIT_C)

        |->
        (
          (tl_c_opcode == C_PROBE_ACK)     ||
          (tl_c_opcode == C_PROBE_ACKDATA) ||
          (tl_c_opcode == C_RELEASE)       ||
          (tl_c_opcode == C_RELEASEDATA)
        );

      endproperty

      assert property (p_c_opcode)
        else
          $error(
            "TL-C SLAVE: illegal C opcode"
          );

      //=======================================================================
      // Probe response address
      //=======================================================================

      property p_probe_address;

        @(posedge clk_i)
        disable iff (!rst_ni)

        c_fire &&
        (state_q == ST_B_WAIT_C)

        |->
        (tl_c_address == probe_addr_q);

      endproperty

      assert property (p_probe_address)
        else
          $error(
            "TL-C SLAVE: Probe response address mismatch"
          );

      //=======================================================================
      // Probe response source
      //=======================================================================

      property p_probe_source;

        @(posedge clk_i)
        disable iff (!rst_ni)

        c_fire &&
        (state_q == ST_B_WAIT_C)

        |->
        (tl_c_source == probe_source_q);

      endproperty

      assert property (p_probe_source)
        else
          $error(
            "TL-C SLAVE: Probe response source mismatch"
          );

      //=======================================================================
      // D source follows A source
      //=======================================================================

      property p_d_source;

        @(posedge clk_i)
        disable iff (!rst_ni)

        d_fire &&
        (
          (state_q == ST_A_D_SEND)
        )

        |->
        (tl_d_source == req_source_q);

      endproperty

      assert property (p_d_source)
        else
          $error(
            "TL-C SLAVE: D source mismatch"
          );

      //=======================================================================
      // ReleaseAck source
      //=======================================================================

      property p_release_ack_source;

        @(posedge clk_i)
        disable iff (!rst_ni)

        d_fire &&
        (tl_d_opcode == D_RELEASE_ACK)

        |->
        (tl_d_source == release_source_q);

      endproperty

      assert property (p_release_ack_source)
        else
          $error(
            "TL-C SLAVE: ReleaseAck source mismatch"
          );

      //=======================================================================
      // A corrupt
      //=======================================================================

      property p_a_corrupt;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_a_valid
        |->
        !tl_a_corrupt;

      endproperty

      assert property (p_a_corrupt)
        else
          $error(
            "TL-C SLAVE: A corrupt not supported"
          );

      //=======================================================================
      // B corrupt
      //=======================================================================

      property p_b_corrupt;

        @(posedge clk_i)
        disable iff (!rst_ni)

        tl_b_valid
        |->
        !tl_b_corrupt;

      endproperty

      assert property (p_b_corrupt)
        else
          $error(
            "TL-C SLAVE: B corrupt not supported"
          );

      //=======================================================================
      // D GrantAck sequencing
      //=======================================================================

      property p_no_new_a_during_grant_ack;

        @(posedge clk_i)
        disable iff (!rst_ni)

        (state_q == ST_A_D_WAIT_E)
        |->
        !tl_a_ready;

      endproperty

      assert property (p_no_new_a_during_grant_ack)
        else
          $error(
            "TL-C SLAVE: accepting A while waiting GrantAck"
          );

      //=======================================================================
      // No B while waiting GrantAck
      //=======================================================================

      property p_no_b_during_grant_ack;

        @(posedge clk_i)
        disable iff (!rst_ni)

        (state_q == ST_A_D_WAIT_E)
        |->
        !tl_b_valid;

      endproperty

      assert property (p_no_b_during_grant_ack)
        else
          $error(
            "TL-C SLAVE: illegal Probe during GrantAck"
          );

      //=======================================================================
      // C data response consistency
      //=======================================================================

      property p_probe_ackdata_data;

        @(posedge clk_i)
        disable iff (!rst_ni)

        c_fire &&
        (tl_c_opcode == C_PROBE_ACKDATA)

        |->
        !tl_c_corrupt;

      endproperty

      assert property (p_probe_ackdata_data)
        else
          $error(
            "TL-C SLAVE: ProbeAckData marked corrupt"
          );

    end

  endgenerate

endmodule
