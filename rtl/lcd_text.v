// ============================================================================
// lcd_text.v
// ----------------------------------------------------------------------------
// Builds the 32 characters shown on the 16x2 LCD from the system state.
//
//   in range, shape found     in range, nothing found     out of range
//   +----------------+        +----------------+          +----------------+
//   |SHAPE: TRIANGLE |        |SHAPE: SEARCHING|          |OUT OF RANGE    |
//   |DIST: 042cm NEAR|        |DIST: 060cm NEAR|          |DIST: 120cm  FAR|
//   +----------------+        +----------------+          +----------------+
//
// No echo -> "DIST: ---cm  FAR".  Gate bypassed (SW) -> status "BYPS".
// Purely combinational: char_idx in, ASCII out.
// ============================================================================
module lcd_text (
    input  wire [4:0] char_idx,
    input  wire [3:0] shape_class,     // 0 none, 1..8 (see shape_analyzer)
    input  wire       in_range,        // distance gate open (or bypassed)
    input  wire       bypass,          // distance gate bypass switch
    input  wire [11:0] dist_bcd,       // distance as 3 BCD digits (registered)
    input  wire       dist_valid,
    output reg  [7:0] char_code
);
    wire [11:0] bcd = dist_bcd;

    // ---- shape names, 9 characters, space padded ----
    function [7:0] name_char;
        input [3:0] c;
        input [3:0] p;
        reg [71:0] s;
        begin
            case (c)
                4'd1:    s = "CIRCLE   ";
                4'd2:    s = "SQUARE   ";
                4'd3:    s = "RECTANGLE";
                4'd4:    s = "TRIANGLE ";
                4'd5:    s = "PENTAGON ";
                4'd6:    s = "HEXAGON  ";
                4'd7:    s = "ELLIPSE  ";
                4'd8:    s = "UNKNOWN  ";
                default: s = "SEARCHING";
            endcase
            name_char = s[71 - p*8 -: 8];
        end
    endfunction

    // ---- fixed strings ----
    localparam [127:0] L1_FAR   = "OUT OF RANGE    ";
    localparam [55:0]  L1_SHAPE = "SHAPE: ";
    localparam [47:0]  L2_DIST  = "DIST: ";

    wire [3:0] p1 = char_idx[3:0];           // column within the line
    wire [31:0] status = bypass ? "BYPS" : (in_range ? "NEAR" : " FAR");

    always @(*) begin
        if (!char_idx[4]) begin
            // ---------------- line 1 ----------------
            if (!in_range)
                char_code = L1_FAR[127 - p1*8 -: 8];
            else if (p1 < 4'd7)
                char_code = L1_SHAPE[55 - p1*8 -: 8];
            else
                char_code = name_char(shape_class, p1 - 4'd7);
        end else begin
            // ---------------- line 2 ----------------
            case (p1)
                4'd0, 4'd1, 4'd2, 4'd3, 4'd4, 4'd5:
                         char_code = L2_DIST[47 - p1*8 -: 8];
                4'd6:    char_code = dist_valid ? {4'h3, bcd[11:8]} : "-";
                4'd7:    char_code = dist_valid ? {4'h3, bcd[7:4]}  : "-";
                4'd8:    char_code = dist_valid ? {4'h3, bcd[3:0]}  : "-";
                4'd9:    char_code = "c";
                4'd10:   char_code = "m";
                4'd11:   char_code = " ";
                4'd12:   char_code = status[31:24];
                4'd13:   char_code = status[23:16];
                4'd14:   char_code = status[15:8];
                default: char_code = status[7:0];
            endcase
        end
    end
endmodule
