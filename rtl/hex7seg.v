// ============================================================================
// hex7seg.v  -  hexadecimal digit to 7-segment pattern (DE2-115: active low)
//               segment order {g,f,e,d,c,b,a} = HEX[6:0]
// ============================================================================
module hex7seg (
    input  wire [3:0] hex,
    input  wire       blank,
    output reg  [6:0] seg
);
    always @(*) begin
        if (blank) seg = 7'h7F;
        else case (hex)
            4'h0: seg = 7'h40;  4'h1: seg = 7'h79;  4'h2: seg = 7'h24;  4'h3: seg = 7'h30;
            4'h4: seg = 7'h19;  4'h5: seg = 7'h12;  4'h6: seg = 7'h02;  4'h7: seg = 7'h78;
            4'h8: seg = 7'h00;  4'h9: seg = 7'h10;  4'hA: seg = 7'h08;  4'hB: seg = 7'h03;
            4'hC: seg = 7'h46;  4'hD: seg = 7'h21;  4'hE: seg = 7'h06;  default: seg = 7'h0E;
        endcase
    end
endmodule

// ============================================================================
// bin2bcd10  -  10-bit binary (0..999, saturating) to 3 BCD digits
// ============================================================================
module bin2bcd10 (
    input  wire [9:0]  bin,
    output reg  [11:0] bcd
);
    wire [9:0] b = (bin > 10'd999) ? 10'd999 : bin;
    integer i;
    always @(*) begin
        bcd = 12'd0;
        for (i = 9; i >= 0; i = i - 1) begin
            if (bcd[3:0]  >= 4'd5) bcd[3:0]  = bcd[3:0]  + 4'd3;
            if (bcd[7:4]  >= 4'd5) bcd[7:4]  = bcd[7:4]  + 4'd3;
            if (bcd[11:8] >= 4'd5) bcd[11:8] = bcd[11:8] + 4'd3;
            bcd = {bcd[10:0], b[i]};
        end
    end
endmodule
