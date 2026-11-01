// ============================================================================
// morph3x3.v
// ----------------------------------------------------------------------------
// Streaming 3x3 binary morphology (erosion or dilation) on a raster-scanned
// mask.  Two 1-bit line buffers (stored together as 2 bits per column) give a
// 3x3 window.  The output pixel is the window centre, so the output stream is
// delayed by one row and one column and carries its own (x, y):
//
//      input (x, y)  ->  output centre (x-1, y-1)
//
// Pixels outside the image are treated as 0.  Each stage therefore drops the
// last row and column (border pixels only).
//
//   OP = 0 : erosion  (AND of 9 pixels)  - removes specks and thin noise
//   OP = 1 : dilation (OR  of 9 pixels)  - fills pin-holes and small gaps
//
// Erode->Dilate = opening, Dilate->Erode = closing; the top level chains
// opening followed by closing to clean the mask.
// ============================================================================
module morph3x3 #(
    parameter OP     = 0,
    parameter XW     = 9,
    parameter YW     = 8,
    parameter LINE_W = 320
)(
    input  wire          clk,
    input  wire          rst,
    input  wire          in_valid,
    input  wire          in_bit,
    input  wire [XW-1:0] in_x,
    input  wire [YW-1:0] in_y,
    output reg           out_valid,
    output reg           out_bit,
    output reg  [XW-1:0] out_x,
    output reg  [YW-1:0] out_y
);
    // line buffer: bit1 = row y-2, bit0 = row y-1 (for the column x)
    reg [1:0] lb [0:LINE_W-1];
    reg [1:0] lb_q;

    integer k;
    initial begin
        for (k = 0; k < LINE_W; k = k + 1) lb[k] = 2'b00;
    end

    // stage A : register input, read line buffer
    reg          a_v, a_bit;
    reg [XW-1:0] a_x;
    reg [YW-1:0] a_y;
    always @(posedge clk) begin
        if (in_valid) lb_q <= lb[in_x];
    end
    always @(posedge clk) begin
        if (rst) begin
            a_v <= 1'b0; a_bit <= 1'b0; a_x <= {XW{1'b0}}; a_y <= {YW{1'b0}};
        end else begin
            a_v   <= in_valid;
            a_bit <= in_bit;
            a_x   <= in_x;
            a_y   <= in_y;
        end
    end

    // stage B : build window column, write line buffer, compute output
    wire       top_ok = (a_y >= 2);                       // row y-2 exists
    wire [2:0] col_n  = {top_ok ? lb_q[1] : 1'b0, lb_q[0], a_bit};  // {top, mid, bottom}
    reg  [2:0] c0, c1;                                    // columns x-2, x-1
    wire [2:0] c0_m   = (a_x >= 2) ? c0 : 3'b000;         // column x-2 exists
    wire [8:0] win    = {c0_m, c1, col_n};
    wire       res    = (OP == 0) ? (&win) : (|win);

    always @(posedge clk) begin
        if (a_v) lb[a_x] <= {lb_q[0], a_bit};
    end

    always @(posedge clk) begin
        if (rst) begin
            c0 <= 3'b000; c1 <= 3'b000;
            out_valid <= 1'b0; out_bit <= 1'b0;
            out_x <= {XW{1'b0}}; out_y <= {YW{1'b0}};
        end else begin
            out_valid <= 1'b0;
            if (a_v) begin
                c0 <= c1;
                c1 <= col_n;
                if (a_x >= 1 && a_y >= 1) begin
                    out_valid <= 1'b1;
                    out_bit   <= res;
                    out_x     <= a_x - 1'b1;
                    out_y     <= a_y - 1'b1;
                end
            end
        end
    end
endmodule
