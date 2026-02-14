module shift_74hc595 #(
    parameter WIDTH = 16,
    parameter HALF_CNT = 20
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire             start,
    input  wire [WIDTH-1:0] data_in,
    output reg              SCLK,
    output reg              RCLK,
    output reg              DIO,
    output reg              busy
);

    // FSM states
    localparam IDLE  = 3'd0;
    localparam LOAD  = 3'd1;
    localparam SHIFT = 3'd2;
    localparam LATCH = 3'd3;
    localparam DONE  = 3'd4;

    reg [2:0] state;
    reg [WIDTH-1:0] shift_reg;
    reg [$clog2(WIDTH):0] bit_idx;
    reg [$clog2(2*HALF_CNT):0] clkcnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            SCLK <= 0;
            RCLK <= 0;
            DIO  <= 0;
            busy <= 0;
            state <= IDLE;
            shift_reg <= 0;
            bit_idx <= 0;
            clkcnt <= 0;
        end else begin
            case (state)
                IDLE: begin
                    SCLK <= 0;
                    RCLK <= 0;
                    busy <= 0;
                    if (start) state <= LOAD;
                end

                LOAD: begin
                    shift_reg <= data_in;
                    bit_idx <= WIDTH-1;
                    clkcnt <= 0;
                    busy <= 1;
                    SCLK <= 0;
                    state <= SHIFT;
                end

                SHIFT: begin
                    if (clkcnt == 0) begin
                        DIO <= shift_reg[bit_idx];
                    end

                    clkcnt <= clkcnt + 1;

                    if (clkcnt == HALF_CNT-1) begin
                        SCLK <= 1;
                    end else if (clkcnt == 2*HALF_CNT-1) begin
                        SCLK <= 0;
                        clkcnt <= 0;
                        if (bit_idx == 0) begin
                            state <= LATCH;
                        end else begin
                            bit_idx <= bit_idx - 1;
                        end
                    end
                end

                LATCH: begin
                    RCLK <= 1;
                    state <= DONE;
                end

                DONE: begin
                    RCLK <= 0;
                    busy <= 0;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule
