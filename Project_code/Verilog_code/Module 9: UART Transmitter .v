
module uart_tx #(
    parameter CLK_FREQ = 40_000_000,
    parameter BAUD_RATE = 9600
)(
    input wire clk,
    input wire rst_n,
    input wire tx_start,
    input wire [7:0] tx_data,
    output reg tx,
    output reg tx_busy
);

    localparam integer BAUD_TICK = CLK_FREQ / BAUD_RATE;
    
    reg [15:0] baud_cnt;
    reg [9:0] tx_shift;
    reg [3:0] bit_index;
    
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            tx <= 1;
            tx_busy <= 0;
            baud_cnt <= 0;
            bit_index <= 0;
            tx_shift <= 10'b11111_11111;
        end else begin
            if (tx_start && !tx_busy) begin
                tx_busy <= 1;
                tx_shift <= {1'b1, tx_data , 1'b0};
                baud_cnt <= 0;
                bit_index <= 0;
            end else begin
                if (tx_busy) begin
                    if (baud_cnt == BAUD_TICK - 1) begin
                        baud_cnt <= 0;
                        tx <= tx_shift[0];
                        tx_shift <= {1'b1, tx_shift[9:1]};
                        bit_index <= bit_index + 1;
                        if (bit_index == 9) tx_busy <= 0;
                    end else begin
                        baud_cnt <= baud_cnt + 1;
                    end
                end else begin
                    tx <= 1;
                end
            end
        end
    end
endmodule
