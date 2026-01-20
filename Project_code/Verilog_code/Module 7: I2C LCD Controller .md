````verilog
module i2c_writeframe(
    input       clk_1MHz,
    input       rst_n,
    input       en_write,
    input       start_frame,
    input       stop_frame,
    input [7:0] data,
    inout       sda,
    output reg  scl,
    output      done,
    output reg  sda_en
);

    localparam  DELAY       = 10;
    reg [20:0]  cnt;
    reg         cnt_clr;

    // FSM states
    localparam  WaitEn      = 0,
                PreStart    = 1,
                Start       = 2,
                AfterStart  = 3,
                PreWrite    = 4,
                WriteLow    = 5,
                WriteHigh   = 6,
                WriteDone   = 7,
                WaitAck     = 8,
                Ack1        = 9,
                Ack2        = 10,
                AckDone     = 11,
                PreStop     = 12,
                Stop        = 13,
                Done        = 14;
    
    reg [3:0]   state, next_state;
    reg [3:0]   bit_cnt;
    reg         sda_out;
    wire        sda_in;

    assign sda = sda_en ? (~sda_out ? 1'b0 : 1'bz) : 1'bz;
    assign sda_in = sda;

    // Microsecond counter
    always @(posedge clk_1MHz, negedge rst_n) begin
        if (!rst_n)
            cnt <= 21'd0;
        else if (cnt_clr)
            cnt <= 21'd0;
        else
            cnt <= cnt + 1'b1;
    end

    // State machine
    always @(posedge clk_1MHz, negedge rst_n) begin
        if (!rst_n)
            state <= WaitEn;
        else
            state <= next_state;
    end

    // Next state logic
    always @(*) begin
        if (!rst_n)
            next_state <= WaitEn;
        else begin
            case (state)
                WaitEn:     next_state = en_write ? (start_frame ? PreStart : PreWrite) : WaitEn;
                PreStart:   next_state = (cnt == DELAY) ? Start : PreStart;
                Start:      next_state = (cnt == DELAY) ? AfterStart : Start;
                AfterStart: next_state = (cnt == DELAY) ? WriteLow : AfterStart;
                PreWrite:   next_state = (cnt == DELAY) ? WriteLow : PreWrite;
                WriteLow:   next_state = (cnt == DELAY) ? WriteHigh : WriteLow;
                WriteHigh:  next_state = (cnt == DELAY && bit_cnt == 4'd8) ? WriteDone : ((cnt == DELAY) ? WriteLow : WriteHigh);
                WriteDone:  next_state = (cnt == DELAY) ? WaitAck : WriteDone;
                WaitAck:    next_state = (sda_in == 1'b0) ? Ack1 : WaitAck;
                Ack1:       next_state = (cnt == DELAY) ? Ack2 : Ack1;
                Ack2:       next_state = (cnt == DELAY) ? AckDone : Ack2;
                AckDone:    next_state = (cnt == DELAY) ? (stop_frame ? PreStop : Done) : AckDone;
                PreStop:    next_state = (cnt == DELAY) ? Stop : PreStop;
                Stop:       next_state = (cnt == DELAY) ? Done : Stop;
                Done:       next_state = WaitEn;
            endcase
        end
    end   

    // Output logic
    always @(posedge clk_1MHz, negedge rst_n) begin
        if (!rst_n) begin
            sda_en  <= 1'b1;
            cnt_clr <= 1'b1;
        end else begin
            case (state)
                WaitEn: begin
                    sda_en  <= 1'b1;
                    cnt_clr <= 1'b1;
                end
                PreStart: begin
                    sda_out <= 1'b1;
                    scl     <= 1'b1;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                Start: begin
                    sda_out <= 1'b0;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                AfterStart: begin
                    scl     <= 1'b0;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                PreWrite: begin
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                WriteLow: begin
                    scl     <= 1'b0;
                    sda_out <= data[7-(bit_cnt-1)] ? 1'b1 : 1'b0;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                WriteHigh: begin
                    scl     <= 1'b1;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                WriteDone: begin
                    scl     <= 1'b0;
                    sda_en  <= 1'b0;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                WaitAck: begin
                    cnt_clr <= 1'b1;
                end
                Ack1: begin
                    scl     <= 1'b1;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                Ack2: begin
                    scl     <= 1'b0;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                AckDone: begin
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                PreStop: begin
                    scl     <= 1'b1;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                Stop: begin
                    sda_en  <= 1'b1;
                    sda_out <= 1'b1;
                    cnt_clr <= (cnt == DELAY-1) ? 1'b1 : 1'b0;
                end
                Done: begin
                    cnt_clr <= 1'b1;
                end
            endcase
        end
    end

    // Bit counter
    always @(posedge clk_1MHz, negedge rst_n) begin
        if (!rst_n)
            bit_cnt <= 4'd0;
        else begin
            case (state)
                WriteLow:   bit_cnt <= (cnt == 1'b0) ? bit_cnt + 1'b1 : bit_cnt;
                WaitEn:     bit_cnt <= 4'd0;
                Done:       bit_cnt <= 4'd0;
                default:    bit_cnt <= bit_cnt;
            endcase
        end
    end

    assign done = (state == Done);
endmodule
