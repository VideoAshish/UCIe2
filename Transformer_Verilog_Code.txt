BFCMHOTDEAL4K


//*************************************************************************//

// Transformer Encoder (one block)
// Fixed sizes for clarity. Behavioral matrix ops for simulation.
// Token-major layout: sequence length (SEQ) x model dimension (D_MODEL).
module transformer_encoder #(
  parameter int SEQ      = 16,
  parameter int D_MODEL  = 64,
  parameter int N_HEADS  = 4,
  parameter int D_HEAD   = D_MODEL / N_HEADS,
  parameter int D_FF     = 256,
  // Data type: change to fixed-point or wider for synthesis
  parameter type T = real
) (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              in_valid,
  input  T                  x_in   [SEQ][D_MODEL],  // input tokens
  // Weights (pre-loaded). In a real design, drive from memories/AXI.
  input  T                  Wq     [D_MODEL][D_MODEL],
  input  T                  Wk     [D_MODEL][D_MODEL],
  input  T                  Wv     [D_MODEL][D_MODEL],
  input  T                  Wo     [D_MODEL][D_MODEL],
  input  T                  W1     [D_MODEL][D_FF],
  input  T                  b1     [D_FF],
  input  T                  W2     [D_FF][D_MODEL],
  input  T                  b2     [D_MODEL],
  input  T                  ln1_g  [D_MODEL],       // LayerNorm1 gamma
  input  T                  ln1_b  [D_MODEL],       // LayerNorm1 beta
  input  T                  ln2_g  [D_MODEL],       // LayerNorm2 gamma
  input  T                  ln2_b  [D_MODEL],       // LayerNorm2 beta

  output logic              out_valid,
  output T                  x_out  [SEQ][D_MODEL]
);

  // Stage 1: Multi-Head Self-Attention with residual + LayerNorm
  T mha_out [SEQ][D_MODEL];
  T res1    [SEQ][D_MODEL];
  T ln1_out [SEQ][D_MODEL];

  // Stage 2: Feedforward (MLP) with residual + LayerNorm
  T ff_out  [SEQ][D_MODEL];
  T res2    [SEQ][D_MODEL];

  // Valid staging (single-cycle comb for sim; register if pipelining)
  always_comb begin
    out_valid = in_valid;
  end

  // Compute attention
  mha_block #(
    .SEQ(SEQ), .D_MODEL(D_MODEL), .N_HEADS(N_HEADS), .D_HEAD(D_HEAD), .T(T)
  ) u_mha (
    .x(x_in), .Wq(Wq), .Wk(Wk), .Wv(Wv), .Wo(Wo),
    .y(mha_out)
  );

  // Residual 1
  always_comb begin
    for (int i = 0; i < SEQ; i++) begin
      for (int d = 0; d < D_MODEL; d++) begin
        res1[i][d] = x_in[i][d] + mha_out[i][d];
      end
    end
  end

  // LayerNorm 1
  layer_norm #(.SEQ(SEQ), .D(D_MODEL), .T(T)) u_ln1 (
    .x(res1), .gamma(ln1_g), .beta(ln1_b), .y(ln1_out)
  );

  // Feedforward: GELU approx via tanh-based form for simulation
  ff_block #(.SEQ(SEQ), .D_IN(D_MODEL), .D_FF(D_FF), .T(T)) u_ff (
    .x(ln1_out), .W1(W1), .b1(b1), .W2(W2), .b2(b2), .y(ff_out)
  );

  // Residual 2
  always_comb begin
    for (int i = 0; i < SEQ; i++) begin
      for (int d = 0; d < D_MODEL; d++) begin
        res2[i][d] = ln1_out[i][d] + ff_out[i][d];
      end
    end
  end

  // LayerNorm 2
  layer_norm #(.SEQ(SEQ), .D(D_MODEL), .T(T)) u_ln2 (
    .x(res2), .gamma(ln2_g), .beta(ln2_b), .y(x_out)
  );

endmodule

//*******************************************************************//

module mha_block #(
  parameter int SEQ = 16,
  parameter int D_MODEL = 64,
  parameter int N_HEADS = 4,
  parameter int D_HEAD = D_MODEL / N_HEADS,
  parameter type T = real
) (
  input  T x  [SEQ][D_MODEL],
  input  T Wq [D_MODEL][D_MODEL],
  input  T Wk [D_MODEL][D_MODEL],
  input  T Wv [D_MODEL][D_MODEL],
  input  T Wo [D_MODEL][D_MODEL],
  output T y  [SEQ][D_MODEL]
);
  // Project Q, K, V: [SEQ x D_MODEL] * [D_MODEL x D_MODEL]
  T Q [SEQ][D_MODEL];
  T K [SEQ][D_MODEL];
  T V [SEQ][D_MODEL];

  matmul_2d #(.A_ROWS(SEQ), .A_COLS(D_MODEL), .B_COLS(D_MODEL), .T(T)) mm_q (.A(x), .B(Wq), .C(Q));
  matmul_2d #(.A_ROWS(SEQ), .A_COLS(D_MODEL), .B_COLS(D_MODEL), .T(T)) mm_k (.A(x), .B(Wk), .C(K));
  matmul_2d #(.A_ROWS(SEQ), .A_COLS(D_MODEL), .B_COLS(D_MODEL), .T(T)) mm_v (.A(x), .B(Wv), .C(V));

  // Split heads
  T Qh [N_HEADS][SEQ][D_HEAD];
  T Kh [N_HEADS][SEQ][D_HEAD];
  T Vh [N_HEADS][SEQ][D_HEAD];

  always_comb begin
    for (int h = 0; h < N_HEADS; h++) begin
      for (int i = 0; i < SEQ; i++) begin
        for (int d = 0; d < D_HEAD; d++) begin
          Qh[h][i][d] = Q[i][h*D_HEAD + d];
          Kh[h][i][d] = K[i][h*D_HEAD + d];
          Vh[h][i][d] = V[i][h*D_HEAD + d];
        end
      end
    end
  end

  // Attention per head: softmax(QK^T / sqrt(D_HEAD)) * V
  T head_out [N_HEADS][SEQ][D_HEAD];

  for (genvar gh = 0; gh < N_HEADS; gh++) begin : HEADS
    attention_head #(.SEQ(SEQ), .D(D_HEAD), .T(T)) u_head (
      .Q(Qh[gh]), .K(Kh[gh]), .V(Vh[gh]), .Y(head_out[gh])
    );
  end

  // Concatenate heads -> [SEQ x D_MODEL]
  T concat [SEQ][D_MODEL];
  always_comb begin
    for (int i = 0; i < SEQ; i++) begin
      for (int h = 0; h < N_HEADS; h++) begin
        for (int d = 0; d < D_HEAD; d++) begin
          concat[i][h*D_HEAD + d] = head_out[h][i][d];
        end
      end
    end
  end

  // Output projection
  matmul_2d #(.A_ROWS(SEQ), .A_COLS(D_MODEL), .B_COLS(D_MODEL), .T(T)) mm_o (.A(concat), .B(Wo), .C(y));
endmodule

//***********************************************************************//



module attention_head #(
  parameter int SEQ = 16,
  parameter int D   = 16,
  parameter type T  = real
) (
  input  T Q [SEQ][D],
  input  T K [SEQ][D],
  input  T V [SEQ][D],
  output T Y [SEQ][D]
);
  // Scores S = Q * K^T / sqrt(D)  => [SEQ x SEQ]
  T S     [SEQ][SEQ];
  T P     [SEQ][SEQ]; // softmax rows

  // Q * K^T
  always_comb begin
    for (int i = 0; i < SEQ; i++) begin
      for (int j = 0; j < SEQ; j++) begin
        T acc = 0.0;
        for (int d = 0; d < D; d++) acc += Q[i][d] * K[j][d];
        S[i][j] = acc / sqrt(D); // scale
      end
    end
  end

  // Softmax row-wise over j
  softmax_row #(.ROWS(SEQ), .COLS(SEQ), .T(T)) u_sm (.X(S), .Y(P));

  // Y = P * V   => [SEQ x SEQ] * [SEQ x D] = [SEQ x D]
  matmul_2d #(.A_ROWS(SEQ), .A_COLS(SEQ), .B_COLS(D), .T(T)) mm_att (.A(P), .B(V), .C(Y));
endmodule


module softmax_row #(
  parameter int ROWS = 16,
  parameter int COLS = 16,
  parameter type T = real
) (
  input  T X [ROWS][COLS],
  output T Y [ROWS][COLS]
);
  // Numerically stable softmax: exp(x - max) / sum(exp(x - max))
  always_comb begin
    for (int r = 0; r < ROWS; r++) begin
      T m = X[r][0];
      for (int c = 1; c < COLS; c++) if (X[r][c] > m) m = X[r][c];

      T sumexp = 0.0;
      T E [COLS];
      for (int c = 0; c < COLS; c++) begin
        E[c] = exp(X[r][c] - m);
        sumexp += E[c];
      end
      for (int c = 0; c < COLS; c++) begin
        Y[r][c] = E[c] / (sumexp + 1e-12);
      end
    end
  end
endmodule

//************************************************************************//


module ff_block #(
  parameter int SEQ = 16,
  parameter int D_IN = 64,
  parameter int D_FF = 256,
  parameter type T = real
) (
  input  T x  [SEQ][D_IN],
  input  T W1 [D_IN][D_FF],
  input  T b1 [D_FF],
  input  T W2 [D_FF][D_IN],
  input  T b2 [D_IN],
  output T y  [SEQ][D_IN]
);
  T h [SEQ][D_FF];

  // h = x*W1 + b1
  matmul_2d #(.A_ROWS(SEQ), .A_COLS(D_IN), .B_COLS(D_FF), .T(T)) mm1 (.A(x), .B(W1), .C(h));
  always_comb begin
    for (int i = 0; i < SEQ; i++)
      for (int f = 0; f < D_FF; f++)
        h[i][f] = h[i][f] + b1[f];
  end

  // GELU approx: 0.5*x*(1 + tanh(√(2/π)*(x + 0.044715*x^3)))
  T a [SEQ][D_FF];
  always_comb begin
    for (int i = 0; i < SEQ; i++) begin
      for (int f = 0; f < D_FF; f++) begin
        T x3 = h[i][f]*h[i][f]*h[i][f];
        T k  = sqrt(2.0/3.1415926535);
        T t  = tanh(k*(h[i][f] + 0.044715*x3));
        a[i][f] = 0.5*h[i][f]*(1.0 + t);
      end
    end
  end

  // y = a*W2 + b2
  matmul_2d #(.A_ROWS(SEQ), .A_COLS(D_FF), .B_COLS(D_IN), .T(T)) mm2 (.A(a), .B(W2), .C(y));
  always_comb begin
    for (int i = 0; i < SEQ; i++)
      for (int d = 0; d < D_IN; d++)
        y[i][d] = y[i][d] + b2[d];
  end
endmodule


//*************************************************************************//


module layer_norm #(
  parameter int SEQ = 16,
  parameter int D   = 64,
  parameter type T  = real
) (
  input  T x     [SEQ][D],
  input  T gamma [D],
  input  T beta  [D],
  output T y     [SEQ][D]
);
  // epsilon for numerical stability
  localparam T EPS = 1e-5;

  always_comb begin
    for (int i = 0; i < SEQ; i++) begin
      // mean
      T mean = 0.0;
      for (int d = 0; d < D; d++) mean += x[i][d];
      mean /= D;

      // variance
      T var = 0.0;
      for (int d = 0; d < D; d++) begin
        T t = x[i][d] - mean;
        var += t*t;
      end
      var /= D;

      // normalize and scale/shift
      for (int d = 0; d < D; d++) begin
        T xn = (x[i][d] - mean) / sqrt(var + EPS);
        y[i][d] = gamma[d]*xn + beta[d];
      end
    end
  end
endmodule


//*********************************************************//


module matmul_2d #(
  parameter int A_ROWS = 16,
  parameter int A_COLS = 64,
  parameter int B_COLS = 64,
  parameter type T = real
) (
  input  T A [A_ROWS][A_COLS],
  input  T B [A_COLS][B_COLS],
  output T C [A_ROWS][B_COLS]
);
  always_comb begin
    for (int i = 0; i < A_ROWS; i++) begin
      for (int j = 0; j < B_COLS; j++) begin
        T acc = 0.0;
        for (int k = 0; k < A_COLS; k++) acc += A[i][k] * B[k][j];
        C[i][j] = acc;
      end
    end
  end
endmodule

//**************************************************************//

Notes on synthesis and adaptation
Fixed-point and pipelining: Replace real with a fixed-point or integer type. Pipeline matmul and softmax; map to DSPs/BRAMs. Use block RAMs for Q/K/V and on-chip buffers for sequence batching.

Masking: For causal or padding masks, add a mask matrix M and set S[i][j] = S[i][j] + M[i][j] before softmax.

Throughput: For streaming, adopt a systolic matmul and row-by-row softmax with running max/sumexp to avoid full-row buffering.

Weights: Drive weights from external memory with a load protocol; add valid/ready on interfaces and register the outputs across stages.

Validation: Compare attention outputs against a Python/NumPy reference for small sequences to ensure numerical alignment.

If you want a version targeted for synthesis (fixed-point, pipelined, with valid/ready handshakes and masks), tell me your sequence length, model dimension, and clock target, and I’ll tailor it.