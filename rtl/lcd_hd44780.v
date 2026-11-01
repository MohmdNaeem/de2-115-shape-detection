// ============================================================================
// lcd_hd44780.v
// ----------------------------------------------------------------------------
// Write-only driver for the 16x2 HD44780-compatible character LCD on the
// DE2-115 (8-bit bus, RW tied low).
//
// After the power-on initialisation the controller continuously refreshes
// both lines from a 32-character source:
//     cmd 0x80, chars 0..15 (line 1), cmd 0xC0, chars 16..31 (line 2)
// The character to show at position `char_idx` is supplied combinationally
// on `char_code` (see lcd_text.v), so the display follows the system state
// with no extra handshaking.  One full refresh takes ~1.8 ms.
//
// Timing (at 50 MHz): E pulse 1 us, 50 us after each write, 2 ms after
// clear, 20 ms power-on wait.  TIME_DIV divides all waits (simulation only).
// ============================================================================
module lcd_hd44780 #(
    parameter CLK_HZ   = 50_000_000,
    parameter TIME_DIV = 1                // >1 shortens delays for simulation
)(
    input  wire       clk,
    input  wire       rst,
    output wire [4:0] char_idx,           // 0..31 character being fetched
    input  wire [7:0] char_code,          // ASCII code for char_idx
    output reg  [7:0] lcd_data,
    output reg        lcd_rs,
    output wire       lcd_rw,
    output reg        lcd_en,
    output wire       lcd_on,
    output wire       lcd_blon
);
    assign lcd_rw   = 1'b0;               // write only
    assign lcd_on   = 1'b1;               // LCD power
    assign lcd_blon = 1'b1;               // backlight (not fitted on all boards)

    localparam CYC_US   = CLK_HZ / 1_000_000;
    localparam T_PWRON  = (CYC_US * 20_000) / TIME_DIV;   // 20 ms
    localparam T_LONG   = (CYC_US * 2_000)  / TIME_DIV;   // 2 ms (clear / home)
    localparam T_INIT   = (CYC_US * 5_000)  / TIME_DIV;   // 5 ms (first function set)
    localparam T_SHORT  = (CYC_US * 50)     / TIME_DIV;   // 50 us (normal command/data)
    localparam T_SETUP  = 4;                              // RS/data setup before E (80 ns)
    localparam T_EPULSE = CYC_US;                         // E high 1 us

    // init command list
    function [7:0] init_cmd;
        input [2:0] i;
        begin
            case (i)
                3'd0: init_cmd = 8'h38;   // function set: 8-bit, 2 lines, 5x8
                3'd1: init_cmd = 8'h38;
                3'd2: init_cmd = 8'h38;
                3'd3: init_cmd = 8'h0C;   // display on, cursor off, blink off
                3'd4: init_cmd = 8'h01;   // clear display
                default: init_cmd = 8'h06;// entry mode: increment, no shift
            endcase
        end
    endfunction

    localparam S_PWRON = 3'd0, S_LOAD = 3'd1, S_SETUP = 3'd2, S_EHIGH = 3'd3,
               S_WAIT  = 3'd4, S_NEXT = 3'd5;

    reg [2:0]  st;
    reg [31:0] cnt;
    reg [31:0] wait_cyc;
    reg        init_phase;
    reg [2:0]  init_i;
    reg [5:0]  pos;                       // refresh position 0..33

    // character index of the current refresh position
    // pos 1..16 -> chars 0..15 (line 1), pos 18..33 -> chars 16..31 (line 2)
    assign char_idx = (pos <= 6'd16) ? (pos[4:0] - 5'd1) : (pos[4:0] - 5'd2);

    always @(posedge clk) begin
        if (rst) begin
            st <= S_PWRON; cnt <= 32'd0; wait_cyc <= T_SHORT;
            init_phase <= 1'b1; init_i <= 3'd0; pos <= 6'd0;
            lcd_data <= 8'h00; lcd_rs <= 1'b0; lcd_en <= 1'b0;
        end else begin
            case (st)
            S_PWRON: begin
                lcd_en <= 1'b0;
                if (cnt >= T_PWRON) begin cnt <= 32'd0; st <= S_LOAD; end
                else cnt <= cnt + 32'd1;
            end
            // choose the next byte to write
            S_LOAD: begin
                cnt <= 32'd0;
                if (init_phase) begin
                    lcd_rs   <= 1'b0;
                    lcd_data <= init_cmd(init_i);
                    wait_cyc <= (init_i == 3'd0) ? T_INIT :
                                (init_i == 3'd4) ? T_LONG : T_SHORT;
                end else if (pos == 6'd0 || pos == 6'd17) begin
                    lcd_rs   <= 1'b0;                          // set DDRAM address
                    lcd_data <= (pos == 6'd0) ? 8'h80 : 8'hC0;
                    wait_cyc <= T_SHORT;
                end else begin
                    lcd_rs   <= 1'b1;                          // character
                    lcd_data <= char_code;
                    wait_cyc <= T_SHORT;
                end
                st <= S_SETUP;
            end
            S_SETUP: begin
                if (cnt >= T_SETUP) begin cnt <= 32'd0; lcd_en <= 1'b1; st <= S_EHIGH; end
                else cnt <= cnt + 32'd1;
            end
            S_EHIGH: begin                                     // data latched on E falling
                if (cnt >= T_EPULSE) begin cnt <= 32'd0; lcd_en <= 1'b0; st <= S_WAIT; end
                else cnt <= cnt + 32'd1;
            end
            S_WAIT: begin
                if (cnt >= wait_cyc) begin cnt <= 32'd0; st <= S_NEXT; end
                else cnt <= cnt + 32'd1;
            end
            S_NEXT: begin
                if (init_phase) begin
                    if (init_i == 3'd5) init_phase <= 1'b0;
                    init_i <= init_i + 3'd1;
                end else begin
                    pos <= (pos == 6'd33) ? 6'd0 : pos + 6'd1;
                end
                st <= S_LOAD;
            end
            default: st <= S_PWRON;
            endcase
        end
    end
endmodule
