````verilog
module top_complete #(
    parameter CLK_HZ = 40_000_000,
    parameter BAUD_RATE = 9600
)(
    input clk,
    input rst_n,
    
    // DHT11
    inout dht_pin,
    
    // LCD I2C
    output scl,
    inout sda,
    
    // UART TX to ESP32
    output uart_tx,
    
    // UART RX from ESP32
    input rx,
    
    // 7-segment display via HC595
    output SCLK,
    output RCLK,
    output DIO,
    
    // Fire alarm system
    input warning_btn,
    output [7:0] led,
    output buzzer,
    output warning_enabled,
    output warning_led,
    output sim
);

    wire [3:0] d0, d1, d2, d3;
    wire temp_sensor, hum_sensor, smoke_sensor, esp32_warning_sig;

    // Main system instance
    top_dht11_lcd_uart #(
        .CLK_HZ(CLK_HZ),
        .BAUD_RATE(BAUD_RATE)
    ) u_dht11_lcd_uart (
        .clk(clk),
        .rst_n(rst_n),
        .dht_pin(dht_pin),
        .scl(scl),
        .sda(sda),
        .uart_tx(uart_tx)
    );

    // UART display system
    top_uart_display #(
        .CLK_FREQ(CLK_HZ),
        .BAUD_RATE(BAUD_RATE)
    ) u_uart_display (
        .clk(clk),
        .rst_n(rst_n),
        .rx(rx),
        .SCLK(SCLK),
        .RCLK(RCLK),
        .DIO(DIO),
        .temp(temp_sensor),
        .hum(hum_sensor),
        .smoke(smoke_sensor),
        .esp32_warning(esp32_warning_sig)
    );

    // Fire alarm system
    fire_alarm_system u_fire_alarm (
        .clk(clk),
        .rst_n(rst_n),
        .warning_btn(warning_btn),
        .esp32_warning(esp32_warning_sig),
        .temp(temp_sensor),
        .hum(hum_sensor),
        .smoke(smoke_sensor),
        .led(led),
        .buzzer(buzzer),
        .warning_enabled(warning_enabled),
        .warning_led(warning_led),
        .sim(sim)
    );

endmodule
