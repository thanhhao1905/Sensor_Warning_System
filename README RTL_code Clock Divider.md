````verilog
module clk_divider #(
    parameter   input_clk_freq  = 40_000_000,  // Input clock frequency
    parameter   output_clk_freq = 1_000_000    // Output clock frequency
)(
    input       clk,
    output      clk_1MHz                                   
);
    reg         clk_1MHz_temp = 0;
    integer     count = 0;
    integer     half_cycle = input_clk_freq / output_clk_freq / 2 - 1;
    
    always @(posedge clk) begin
        if (count == half_cycle) begin
            clk_1MHz_temp <= ~clk_1MHz_temp;
            count <= 0;
        end else
            count <= count + 1;
    end

    assign clk_1MHz = clk_1MHz_temp;
endmodule
