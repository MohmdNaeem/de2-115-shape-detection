// ============================================================================
// bayer_to_rgb.v
// ----------------------------------------------------------------------------
// Converts the 640x480 raw Bayer stream to a 320x240 RGB stream by collapsing
// every 2x2 Bayer quad into one RGB pixel:
//
//        even row :  TL  TR          R = R
//        odd  row :  BL  BR          G = (G1 + G2) / 2
//                                    B = B
//
// One line buffer (320 x 24 bit) holds the even-row pair of each quad.  The
// RGB pixel is produced while the odd row streams in.  The 2x reduction in
// resolution also acts as a 2x2 box filter, which lowers sensor noise.
//
// BAYER selects which colour sits in each corner of the quad:
//   0 = GRBG (TL=G, TR=R, BL=B, BR=G)  <- MT9P001 default readout
//   1 = RGGB   2 = BGGR   3 = GBRG
// If red and blue look swapped on screen, change this parameter.
// ============================================================================
module bayer_to_rgb #(
    parameter BAYER   = 0,
    parameter IN_W    = 640,
    parameter IN_H    = 480
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [11:0] in_pix,
    input  wire        in_valid,
    input  wire [11:0] in_x,
    input  wire [11:0] in_y,
    output reg  [7:0]  r,
    output reg  [7:0]  g,
    output reg  [7:0]  b,
    output reg         out_valid,
    output reg  [8:0]  out_x,         // 0..319
    output reg  [7:0]  out_y          // 0..239
);
    // even-row pair buffer: {left pixel, right pixel}
    reg [23:0] lbuf [0:(IN_W/2)-1];
    reg [23:0] lb_q;
    reg [11:0] prev;                   // previous pixel on the current row

    wire in_range = in_valid && (in_x < IN_W) && (in_y < IN_H);
    wire odd_x    = in_x[0];
    wire odd_y    = in_y[0];

    integer k;
    initial begin
        for (k = 0; k < IN_W/2; k = k + 1) lbuf[k] = 24'd0;
    end

    // line buffer write: even row, odd column -> store {left, right}
    always @(posedge clk) begin
        if (in_range && !odd_y && odd_x)
            lbuf[in_x[9:1]] <= {prev, in_pix};
    end
    // line buffer read: odd row, even column -> fetch pair for this quad
    always @(posedge clk) begin
        if (in_range && odd_y && !odd_x)
            lb_q <= lbuf[in_x[9:1]];
    end

    // quad corners at odd row / odd column
    wire [11:0] tl = lb_q[23:12];
    wire [11:0] tr = lb_q[11:0];
    wire [11:0] bl = prev;
    wire [11:0] br = in_pix;

    reg [11:0] cr, cb;
    reg [12:0] gsum;
    always @(*) begin
        case (BAYER)
            1:       begin cr = tl; cb = br; gsum = tr + bl; end   // RGGB
            2:       begin cr = br; cb = tl; gsum = tr + bl; end   // BGGR
            3:       begin cr = bl; cb = tr; gsum = tl + br; end   // GBRG
            default: begin cr = tr; cb = bl; gsum = tl + br; end   // GRBG
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            prev <= 12'd0;
            out_valid <= 1'b0;
            r <= 8'd0; g <= 8'd0; b <= 8'd0;
            out_x <= 9'd0; out_y <= 8'd0;
        end else begin
            if (in_range) prev <= in_pix;
            out_valid <= in_range && odd_y && odd_x;
            if (in_range && odd_y && odd_x) begin
                r     <= cr[11:4];
                g     <= gsum[12:5];
                b     <= cb[11:4];
                out_x <= in_x[9:1];
                out_y <= in_y[8:1];
            end
        end
    end
endmodule
