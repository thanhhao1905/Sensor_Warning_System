module dht_11 #(
    parameter CLK_HZ = 40_000_000
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,           // Start signal
    inout  wire dht_io,          // DHT11 pin
    output reg  [7:0] hum_int,   // Integer humidity
    output reg  [7:0] temp_int,  // Integer temperature
    output reg  valid,           // Pull high when valid value
    output reg  busy,            // High when reading data
    output reg  checksum
);

    // Timing parameters (clock cycles)
    localparam T_START_LOW   = CLK_HZ/1_000*18;        // 18ms
    localparam T_START_REL   = CLK_HZ/1_000_000*30;    // 30us
    localparam T_RESP_MIN    = CLK_HZ/1_000_000*60;    // >= ~60us (80us)
    localparam T_BIT_LOW     = CLK_HZ/1_000_000*48;    // 48us
    localparam T_THRESHOLD   = CLK_HZ/1_000_000*40;    // < 40us: bit0, > 40us: bit1
    localparam TIMEOUT_SHORT = CLK_HZ/1_000_000*200;   // 200us
    
    // IO - 3 states
    reg  oe;            // 1: FPGA control dht_out, 0: release
    reg  dht_out;       // oe = 1, pull low to create start signal
    wire dht_in;
    
    assign dht_io = oe ? dht_out : 1'bz;
    
    // Synchronize input
    reg dht_sync1, dht_sync2;
    always @(posedge clk) begin
        dht_sync1 <= dht_io;
        dht_sync2 <= dht_sync1;
    end
    assign dht_in = dht_sync2;
    
    // Finite State Machine states
    localparam [3:0]
        IDLE           = 4'd0,
        START_L        = 4'd1,
        START_REL_HI   = 4'd2,
        WAIT_RESP_L    = 4'd3,
        WAIT_RESP_H    = 4'd4,
        READ_BIT_L     = 4'd5,
        READ_BIT_H_MEAS= 4'd6,
        STORE_BIT      = 4'd7,
        DONE           = 4'd8;
    
    reg [3:0] state;
    integer   cnt;
    integer   bit_idx;
    reg [39:0] shift; // 40 bit: H_int, H_dec, T_int, T_dec, checksum
    reg        bit_val;
    
    // State and flow control
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= IDLE;
            cnt      <= 0;
            bit_idx  <= 0;
            shift    <= 40'd0;
            bit_val  <= 1'b0;
            hum_int  <= 8'd0;
            temp_int <= 8'd0;
            checksum <= 1'b0;
            valid    <= 1'b0;
            busy     <= 1'b0;
            oe       <= 1'b0;
            dht_out  <= 1'b1;
        end else begin
            valid <= 1'b0;
            
            case (state)
                IDLE: begin
                    busy     <= 1'b0;
                    oe       <= 1'b0;
                    dht_out  <= 1'b1;
                    cnt      <= 0;
                    bit_idx  <= 0;
                    shift    <= 40'd0;
                    checksum <= 1'b0;
                    if (start) begin
                        busy     <= 1'b1;
                        oe       <= 1'b1;
                        dht_out  <= 1'b0;
                        cnt      <= 0;
                        state    <= START_L;
                    end
                end
                
                START_L: begin
                    cnt     <= cnt + 1;
                    dht_out <= 1'b0;
                    if (cnt >= T_START_LOW) begin
                        cnt   <= 0;
                        oe    <= 1'b0;
                        state <= START_REL_HI;
                    end
                end
                
                START_REL_HI: begin
                    cnt <= cnt + 1;
                    if (cnt >= T_START_REL) begin
                        cnt   <= 0;
                        state <= WAIT_RESP_L;
                    end
                end
                
                WAIT_RESP_L: begin
                    if (dht_in == 1'b0) begin
                        cnt <= cnt + 1;
                        if (cnt >= TIMEOUT_SHORT) state <= IDLE;
                    end else begin
                        if (cnt != 0) begin
                            if (cnt >= T_RESP_MIN) begin
                                cnt   <= 0;
                                state <= WAIT_RESP_H;
                            end else begin
                                state <= IDLE;
                            end
                        end
                    end
                end
                
                WAIT_RESP_H: begin
                    if (dht_in == 1'b1) begin
                        cnt <= cnt + 1;
                        if (cnt >= TIMEOUT_SHORT) state <= IDLE;
                    end else begin
                        if (cnt != 0) begin
                            if (cnt >= T_RESP_MIN) begin
                                cnt   <= 0;
                                state <= READ_BIT_L;
                            end else begin
                                state <= IDLE;
                            end
                        end
                    end
                end
                
                READ_BIT_L: begin
                    if (dht_in == 1'b0) begin
                        cnt <= cnt + 1;
                        if (cnt > TIMEOUT_SHORT) state <= IDLE;
                    end else begin
                        if (cnt >= T_BIT_LOW) begin
                            cnt   <= 0;
                            state <= READ_BIT_H_MEAS;
                        end else begin
                            state <= IDLE;
                        end
                    end
                end
                
                READ_BIT_H_MEAS: begin
                    if (dht_in == 1'b1) begin
                        cnt <= cnt + 1;
                        if (cnt > TIMEOUT_SHORT) state <= IDLE;
                    end else begin
                        bit_val <= (cnt >= T_THRESHOLD) ? 1'b1 : 1'b0;
                        cnt    <= 0;
                        state  <= STORE_BIT;
                    end
                end
                
                STORE_BIT: begin
                    shift   <= {shift[38:0], bit_val};
                    bit_idx <= bit_idx + 1;
                    if (bit_idx == 6'd39) begin
                        state <= DONE;
                    end else begin
                        state <= READ_BIT_L;
                    end
                end
                
                DONE: begin
                    hum_int  <= shift[39:32];
                    temp_int <= shift[23:16];
                    checksum <= (((shift[39:32] + shift[31:24] + shift[23:16] + shift[15:8]) & 8'hFF) == shift[7:0]);
                    valid    <= 1'b1;
                    state    <= IDLE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end
endmodule
