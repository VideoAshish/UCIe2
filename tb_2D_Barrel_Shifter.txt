//==============================================================================
// Testbench for 2D Barrel Shifter
//==============================================================================
module tb_barrel_shifter_2d;
    
    // Parameters
    localparam int NUM_ROWS = 4;
    localparam int NUM_COLS = 8;
    localparam int SHAMT_H_WIDTH = $clog2(NUM_COLS);
    localparam int SHAMT_V_WIDTH = $clog2(NUM_ROWS);
    
    // Signals - Simple shifter
    logic [NUM_ROWS-1:0][NUM_COLS-1:0] data_in_s;
    logic [SHAMT_H_WIDTH-1:0]          shamt_h_s;
    logic [SHAMT_V_WIDTH-1:0]          shamt_v_s;
    logic                              left_h_s, up_v_s, arithmetic_s;
    logic [NUM_ROWS-1:0][NUM_COLS-1:0] data_out_s;
    
    // Signals - Log shifter
    logic                              clk, rst_n, valid_in;
    logic                              valid_out;
    logic [NUM_ROWS-1:0][NUM_COLS-1:0] data_out_log;
    
    // Signals - Wrap shifter
    logic [NUM_ROWS-1:0][NUM_COLS-1:0] data_out_wrap;
    logic                              wrap_h, wrap_v;
    
    // Instantiate DUTs
    barrel_shifter_2d_simple #(
        .NUM_ROWS(NUM_ROWS),
        .NUM_COLS(NUM_COLS)
    ) uut_simple (
        .data_in(data_in_s),
        .shamt_h(shamt_h_s),
        .shamt_v(shamt_v_s),
        .left_h(left_h_s),
        .up_v(up_v_s),
        .arithmetic(arithmetic_s),
        .data_out(data_out_s)
    );
    
    barrel_shifter_2d_log #(
        .NUM_ROWS(NUM_ROWS),
        .NUM_COLS(NUM_COLS)
    ) uut_log (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .data_in(data_in_s),
        .shamt_h(shamt_h_s),
        .shamt_v(shamt_v_s),
        .left_h(left_h_s),
        .up_v(up_v_s),
        .arithmetic(arithmetic_s),
        .valid_out(valid_out),
        .data_out(data_out_log)
    );
    
    barrel_shifter_2d_wrap #(
        .NUM_ROWS(NUM_ROWS),
        .NUM_COLS(NUM_COLS)
    ) uut_wrap (
        .data_in(data_in_s),
        .shamt_h(shamt_h_s),
        .shamt_v(shamt_v_s),
        .left_h(left_h_s),
        .up_v(up_v_s),
        .wrap_h(wrap_h),
        .wrap_v(wrap_v),
        .arithmetic(arithmetic_s),
        .data_out(data_out_wrap)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Test procedure
    initial begin
        // Initialize
        rst_n = 0;
        valid_in = 0;
        wrap_h = 0;
        wrap_v = 0;
        arithmetic_s = 0;
        left_h_s = 0;
        up_v_s = 0;
        shamt_h_s = 0;
        shamt_v_s = 0;
        data_in_s = '0;
        
        #20 rst_n = 1;
        #10;
        
        //----------------------------------------------------------------------
        // Test 1: Initialize with pattern data
        //----------------------------------------------------------------------
        $display("\n=== Test 1: Basic Pattern ===");
        for (int i = 0; i < NUM_ROWS; i++) begin
            data_in_s[i] = i * 16 + 8'hAA;  // Each row has unique value
        end
        display_data("Input:", data_in_s);
        
        //----------------------------------------------------------------------
        // Test 2: No shift
        //----------------------------------------------------------------------
        $display("\n=== Test 2: No Shift ===");
        shamt_h_s = 0;
        shamt_v_s = 0;
        left_h_s = 0;
        up_v_s = 0;
        valid_in = 1;
        #10;
        valid_in = 0;
        display_data("Output:", data_out_s);
        
        //----------------------------------------------------------------------
        // Test 3: Horizontal right shift by 2
        //----------------------------------------------------------------------
        $display("\n=== Test 3: Horizontal Right Shift by 2 ===");
        shamt_h_s = 2;
        shamt_v_s = 0;
        left_h_s = 0;
        valid_in = 1;
        #10;
        valid_in = 0;
        display_data("Output:", data_out_s);
        
        //----------------------------------------------------------------------
        // Test 4: Horizontal left shift by 3
        //----------------------------------------------------------------------
        $display("\n=== Test 4: Horizontal Left Shift by 3 ===");
        shamt_h_s = 3;
        shamt_v_s = 0;
        left_h_s = 1;
        valid_in = 1;
        #10;
        valid_in = 0;
        display_data("Output:", data_out_s);
        
        //----------------------------------------------------------------------
        // Test 5: Vertical shift down by 1
        //----------------------------------------------------------------------
        $display("\n=== Test 5: Vertical Shift Down by 1 ===");
        shamt_h_s = 0;
        shamt_v_s = 1;
        up_v_s = 0;
        valid_in = 1;
        #10;
        valid_in = 0;
        display_data("Output:", data_out_s);
        
        //----------------------------------------------------------------------
        // Test 6: Vertical shift up by 2
        //----------------------------------------------------------------------
        $display("\n=== Test 6: Vertical Shift Up by 2 ===");
        shamt_h_s = 0;
        shamt_v_s = 2;
        up_v_s = 1;
        valid_in = 1;
        #10;
        valid_in = 0;
        display_data("Output:", data_out_s);
        
        //----------------------------------------------------------------------
        // Test 7: Combined horizontal and vertical shift
        //----------------------------------------------------------------------
        $display("\n=== Test 7: Combined H-Right by 2, V-Down by 1 ===");
        shamt_h_s = 2;
        shamt_v_s = 1;
        left_h_s = 0;
        up_v_s = 0;
        valid_in = 1;
        #10;
        valid_in = 0;
        display_data("Simple Output:", data_out_s);
        
        //----------------------------------------------------------------------
        // Test 8: Wrap-around horizontal shift
        //----------------------------------------------------------------------
        $display("\n=== Test 8: Wrap Horizontal Left by 4 ===");
        shamt_h_s = 4;
        shamt_v_s = 0;
        left_h_s = 1;
        wrap_h = 1;
        wrap_v = 0;
        valid_in = 1;
        #10;
        valid_in = 0;
        display_data("Wrap Output:", data_out_wrap);
        
        //----------------------------------------------------------------------
        // Test 9: Wrap-around vertical shift
        //----------------------------------------------------------------------
        $display("\n=== Test 9: Wrap Vertical Up by 1 ===");
        shamt_h_s = 0;
        shamt_v_s = 1;
        up_v_s = 1;
        wrap_h = 0;
        wrap_v = 1;
        valid_in = 1;
        #10;
        valid_in = 0;
        display_data("Wrap Output:", data_out_wrap);
        
        //----------------------------------------------------------------------
        // Test 10: Full 2D wrap-around
        //----------------------------------------------------------------------
        $display("\n=== Test 10: Full 2D Wrap (H-Left 3, V-Up 2) ===");
        shamt_h_s = 3;
        shamt_v_s = 2;
        left_h_s = 1;
        up_v_s = 1;
        wrap_h = 1;
        wrap_v = 1;
        valid_in = 1;
        #10;
        valid_in = 0;
        display_data("Wrap Output:", data_out_wrap);
        
        //----------------------------------------------------------------------
        // Test 11: Arithmetic right shift
        //----------------------------------------------------------------------
        $display("\n=== Test 11: Arithmetic Right Shift ===");
        // Set MSB of each row for sign extension test
        for (int i = 0; i < NUM_ROWS; i++) begin
            data_in_s[i] = {1'b1, i[2:0], 4'b1111};
        end
        display_data("Input:", data_in_s);
        shamt_h_s = 3;
        shamt_v_s = 0;
        left_h_s = 0;
        up_v_s = 0;
        wrap_h = 0;
        wrap_v = 0;
        arithmetic_s = 1;
        valid_in = 1;
        #10;
        valid_in = 0;
        display_data("Arithmetic Output:", data_out_s);
        
        //----------------------------------------------------------------------
        // Wait for pipelined output and finish
        //----------------------------------------------------------------------
        #100;
        $display("\n=== Pipelined Output (delayed) ===");
        display_data("Log Output:", data_out_log);
        
        $display("\n=== All Tests Complete ===");
        $finish;
    end
    
    // Helper task to display 2D data
    task display_data(input string label, input [NUM_ROWS-1:0][NUM_COLS-1:0] data);
        $display("%s", label);
        for (int row = 0; row < NUM_ROWS; row++) begin
            $display("  Row %0d: %b (0x%0h)", row, data[row], data[row]);
        end
    endtask
    
    // Waveform dump
    initial begin
        $dumpfile("barrel_shifter_2d.vcd");
        $dumpvars(0, tb_barrel_shifter_2d);
    end

endmodule