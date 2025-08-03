`timescale 1ns / 1ps
module home_automation (
    input clk, 
    input rst, 
    output trig,
    input echo,
    output scl,
    inout sda,
    input [3:0] sw,
    output [2:0] led
);
    parameter CLK_FREQ = 100_000_000;
    parameter DISTANCE_THRESHOLD = 50; 
    parameter TEMP_THRESHOLD = 25; 
    parameter CORRECT_CODE = 4'b1010; 
    parameter BME280_ADDR = 8'b11101110; 
    parameter TEMP_REG = 8'hFA; 
    parameter CALIB_00 = 8'h88; 
    parameter [2:0] IDLE       = 3'b000;
    parameter [2:0] START      = 3'b001;
    parameter [2:0] SEND_ADDR  = 3'b010;
    parameter [2:0] SEND_REG   = 3'b011;
    parameter [2:0] REPEAT_START = 3'b100;
    parameter [2:0] READ_ADDR  = 3'b101;
    parameter [2:0] READ_DATA  = 3'b110;
    parameter [2:0] STOP       = 3'b111;
    reg trigger_pulse = 0;
    reg [31:0] echo_duration = 0;
    reg [31:0] distance_cm = 0;
    reg distance_valid = 0;
    reg [31:0] i2c_counter = 0;
    reg [7:0] temperature = 0;
    reg [19:0] temperature_raw = 0;
    reg [15:0] dig_T1;
    reg [16:0] dig_T2;
    reg [16:0] dig_T3;
    reg [7:0] calib_data [0:23];
    reg calib_read_done = 0;
    reg [2:0] state = IDLE;
    reg [3:0] bit_count = 0;
    reg [7:0] i2c_data = 0;
    reg scl_out = 1;
    reg sda_out = 1;
    reg ack_received = 0;
    reg [31:0] delay_counter = 0;
    reg [2:0] read_phase = 0;
    reg [7:0] temp_data [0:2]; 
    integer var_temp; 
    reg code_correct = 0;
    reg [31:0] blink_counter = 0;
    reg blink_led = 0;
    reg light_on = 0;
    reg fan_on = 0;
    assign scl = scl_out;
    assign sda = sda_out ? 1'bz : 1'b0;
    reg [31:0] trigger_counter = 0;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            trigger_pulse <= 0;
            trigger_counter <= 0;
        end else begin
            if (trigger_counter < 1000) begin 
                trigger_pulse <= 1;
                trigger_counter <= trigger_counter + 1;
            end else if (trigger_counter < CLK_FREQ) begin
                trigger_pulse <= 0;
                trigger_counter <= trigger_counter + 1;
            end else begin
                trigger_counter <= 0;
            end
        end
    end
    assign trig = trigger_pulse;
    reg measuring = 0;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            echo_duration <= 0;
            distance_cm <= 0;
            distance_valid <= 0;
            measuring <= 0;
        end else begin
            if (echo && !measuring) begin
                measuring <= 1;
                echo_duration <= 0;
            end else if (echo && measuring) begin
                echo_duration <= echo_duration + 1;
            end else if (!echo && measuring) begin
                measuring <= 0;
                distance_valid <= 1;
                distance_cm <= (echo_duration * 343) / (2 * (CLK_FREQ / 100));
            end
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            scl_out <= 1;
            sda_out <= 1;
            i2c_counter <= 0;
            bit_count <= 0;
            ack_received <= 0;
            delay_counter <= 0;
            read_phase <= 0;
            calib_read_done <= 0;
            temperature <= 0;
            dig_T1 <= 0;
            dig_T2 <= 0;
            dig_T3 <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (delay_counter < CLK_FREQ/10) begin 
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 0;
                        if (!calib_read_done) begin
                            state <= START;
                            i2c_data <= {BME280_ADDR, 1'b0}; 
                        end else begin
                            if (i2c_counter < CLK_FREQ) begin 
                                i2c_counter <= i2c_counter + 1;
                            end else begin
                                i2c_counter <= 0;
                                state <= START;
                                i2c_data <= {BME280_ADDR, 1'b0}; 
                            end
                        end
                    end
                end
                START: begin
                    if (delay_counter < CLK_FREQ/1000) begin 
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 0;
                        sda_out <= 0;
                        state <= SEND_ADDR;
                        bit_count <= 7;
                    end
                end
                SEND_ADDR: begin
                    if (delay_counter < CLK_FREQ/10000) begin 
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 0;
                        scl_out <= ~scl_out;
                        
                        if (scl_out) begin
                            sda_out <= i2c_data[bit_count];
                        end else begin
                            if (bit_count == 0) begin
                                scl_out <= 1;
                                state <= SEND_REG;
                                if (!calib_read_done) begin
                                    i2c_data <= CALIB_00; 
                                end else begin
                                    i2c_data <= TEMP_REG; 
                                end
                                bit_count <= 7;
                            end else begin
                                bit_count <= bit_count - 1;
                            end
                        end
                    end
                end
                SEND_REG: begin
                    if (delay_counter < CLK_FREQ/10000) begin 
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 0;
                        scl_out <= ~scl_out;
                        
                        if (scl_out) begin
                            sda_out <= i2c_data[bit_count];
                        end else begin
                            if (bit_count == 0) begin
                                scl_out <= 1;
                                state <= REPEAT_START;
                            end else begin
                                bit_count <= bit_count - 1;
                            end
                        end
                    end
                end
                REPEAT_START: begin
                    if (delay_counter < CLK_FREQ/1000) begin 
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 0;
                        sda_out <= 1;
                        scl_out <= 1;
                        state <= READ_ADDR;
                        i2c_data <= {BME280_ADDR, 1'b1}; 
                        bit_count <= 7;
                    end
                end
                READ_ADDR: begin
                    if (delay_counter < CLK_FREQ/10000) begin 
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 0;
                        scl_out <= ~scl_out;
                        
                        if (scl_out) begin
                            ack_received <= ~sda;
                        end else begin
                            if (bit_count == 0) begin
                                scl_out <= 1;
                                state <= READ_DATA;
                                bit_count <= 7;
                                if (!calib_read_done) begin
                                    read_phase <= 0;
                                end else begin
                                    read_phase <= 3;
                                end
                            end else begin
                                bit_count <= bit_count - 1;
                            end
                        end
                    end
                end
                READ_DATA: begin
                    if (delay_counter < CLK_FREQ/10000) begin 
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 0;
                        scl_out <= ~scl_out;
                        if (scl_out) begin
                            if (!calib_read_done) begin
                                calib_data[{read_phase, bit_count}] <= sda;
                            end else if (read_phase > 0) begin
                                temp_data[read_phase-1][bit_count] <= sda;
                            end
                        end else begin
                            if (bit_count == 0) begin
                                if (read_phase == 0) begin
                                    dig_T1 <= {calib_data[1], calib_data[0]};
                                    dig_T2 <= {calib_data[3], calib_data[2]};
                                    dig_T3 <= {calib_data[5], calib_data[4]};
                                    if (read_phase < 23) begin
                                        read_phase <= read_phase + 1;
                                        bit_count <= 7;
                                    end else begin
                                        calib_read_done <= 1;
                                        state <= STOP;
                                    end
                                end else if (read_phase > 1) begin
                                    read_phase <= read_phase - 1;
                                    bit_count <= 7;
                                end else begin
                                    temperature_raw <= {temp_data[0], temp_data[1], temp_data[2][7:4]};
                                    state <= STOP;
                                    var_temp = (temperature_raw / 16) - (dig_T1 * 2);
                                    var_temp = (var_temp * dig_T2) / 2048;
                                    var_temp = (var_temp + (dig_T3 * var_temp) / 4096) / 1024;
                                    temperature <= var_temp / 100; 
                                end
                            end else begin
                                bit_count <= bit_count - 1;
                            end
                        end
                    end
                end
                STOP: begin
                    if (delay_counter < CLK_FREQ/1000) begin // 1us delay
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 0;
                        sda_out <= 0;
                        scl_out <= 0;
                        state <= IDLE;
                    end
                end
            endcase
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            code_correct <= 0;
        end else begin
            code_correct <= (sw == CORRECT_CODE);
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            blink_counter <= 0;
            blink_led <= 0;
        end else if (!code_correct) begin
            if (blink_counter < CLK_FREQ / 2) begin
                blink_counter <= blink_counter + 1;
                blink_led <= 1;
            end else if (blink_counter < CLK_FREQ) begin
                blink_counter <= blink_counter + 1;
                blink_led <= 0;
            end else begin
                blink_counter <= 0;
            end
        end else begin
            blink_led <= 0;
        end
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            light_on <= 0;
            fan_on <= 0;
        end else begin
            if (distance_valid && distance_cm < DISTANCE_THRESHOLD && code_correct) begin
                light_on <= 1;
                fan_on <= 1;
            end else begin
                light_on <= 0;
            end
            if (temperature > TEMP_THRESHOLD && code_correct) begin
                fan_on <= 1;
            end else if (distance_cm >= DISTANCE_THRESHOLD) begin
                fan_on <= 0;
            end
        end
    end
    assign led[0] = light_on; 
    assign led[1] = fan_on; 
    assign led[2] = blink_led; 
endmodule