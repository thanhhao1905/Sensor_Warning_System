
module fire_alarm_system(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       warning_btn,
    input  wire       esp32_warning,
    input  wire       temp,
    input  wire       hum,
    input  wire       smoke,
    output reg  [7:0] led,
    output reg        buzzer,
    output reg        warning_enabled,
    output reg        warning_led,
    output reg        sim
);

    parameter CLK_FREQ      = 40_000_000;
    parameter CNT_0_1S      = CLK_FREQ / 10 - 1;
    parameter DEBOUNCE_TIME = CLK_FREQ / 100 - 1;

    // Timing and LED control
    reg [31:0] toggle_counter;
    reg [2:0]  led_index;
    
    // Button synchronization and debouncing
    reg        warning_btn_sync;
    reg        warning_btn_prev;
    reg [31:0] debounce_counter;
    reg        btn_pressed;
    
    // ESP32 signal synchronization
    reg        esp32_warning_sync;
    reg        esp32_warning_prev;
    reg        esp32_btn_pressed;
    
    // Control logic
    reg        warning_state;

    // Button debouncing
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            warning_btn_sync <= 1'b0;
            warning_btn_prev <= 1'b0;
            debounce_counter <= 32'd0;
            btn_pressed      <= 1'b0;
        end else begin
            warning_btn_sync <= warning_btn;
            warning_btn_prev <= warning_btn_sync;
            
            if (debounce_counter > 0) begin
                debounce_counter <= debounce_counter - 1'b1;
                btn_pressed      <= 1'b0;
            end else begin
                if (warning_btn_prev && !warning_btn_sync) begin
                    debounce_counter <= DEBOUNCE_TIME;
                    btn_pressed      <= 1'b1;
                end else begin
                    btn_pressed <= 1'b0;
                end
            end
        end
    end

    // ESP32 signal synchronization
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            esp32_warning_sync <= 1'b0;
            esp32_warning_prev <= 1'b0;
            esp32_btn_pressed  <= 1'b0;
        end else begin
            esp32_warning_sync <= esp32_warning;
            esp32_warning_prev <= esp32_warning_sync;
            esp32_btn_pressed <= (esp32_warning_prev != esp32_warning_sync);
        end
    end

    // Warning mode management
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            warning_state   <= 1'b1;
            warning_enabled <= 1'b1;
            warning_led     <= 1'b1;
        end else begin
            if (btn_pressed || esp32_btn_pressed) begin
                warning_state <= ~warning_state;
            end
            
            warning_enabled <= warning_state;
            warning_led     <= warning_state;
        end
    end

    // Alarm detection
    wire sensor_active   = (temp | hum | smoke);
    wire sensor_sim = (temp | smoke);
    wire alarm_condition = warning_enabled && sensor_active;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            toggle_counter <= 32'd0;
            led_index      <= 3'd0;
            buzzer         <= 1'b0;
            led            <= 8'hFF;
            sim            <= 1'b0;
        end else begin
            if (!warning_enabled) begin
                toggle_counter <= 32'd0;
                led_index      <= 3'd0;
                buzzer         <= 1'b0;
                led            <= 8'hFF;
                sim            <= 1'b0;
            end else if (!alarm_condition) begin
                toggle_counter <= 32'd0;
                led_index      <= 3'd0;
                buzzer         <= 1'b0;
                led            <= 8'hFF;
                sim            <= 1'b0;
            end else begin
                sim <= sensor_sim ? 1 : 0;
                
                if (toggle_counter >= CNT_0_1S) begin
                    toggle_counter <= 32'd0;
                    buzzer         <= ~buzzer;
                    led_index <= (led_index == 3'd7) ? 3'd0 : led_index + 1'b1;
                end else begin
                    toggle_counter <= toggle_counter + 1'b1;
                end
                
                case (led_index)
                    3'd0: led <= 8'b11111110;
                    3'd1: led <= 8'b11111101;
                    3'd2: led <= 8'b11111011;
                    3'd3: led <= 8'b11110111;
                    3'd4: led <= 8'b11101111;
                    3'd5: led <= 8'b11011111;
                    3'd6: led <= 8'b10111111;
                    3'd7: led <= 8'b01111111;
                    default: led <= 8'hFF;
                endcase
            end
        end
    end
endmodule
