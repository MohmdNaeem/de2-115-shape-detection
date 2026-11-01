// ============================================================================
// vga_display.v
// ----------------------------------------------------------------------------
// 640x480 @ 60 Hz VGA output for the DE2-115's ADV7123 video DAC.
//
// * Runs on CLOCK_50 with a 25 MHz pixel enable; VGA_CLK is generated so its
//   rising edge falls in the middle of each pixel's data window.
// * Reads the 320x240 frame buffer and scales it 2x in both directions.
// * Overlays a green bounding box (4 screen pixels thick) around the detected
//   object, and a 24x24 status square in the top-left corner:
//   green = object within range (gate open), red = out of range.
// * Text overlay at the bottom of the screen: the same two lines as the LCD
//   (shape name and distance), 5x7 font scaled x4.  txt_idx = character
//   position 0..31, txt_char = its ASCII code (from lcd_text).
// * show_mask = 1 displays the binary segmentation mask instead of the camera
//   image (useful when tuning the threshold mode / lighting).
// ============================================================================
module vga_display (
    input  wire        clk,             // 50 MHz
    input  wire        rst,
    // frame buffer read port (1-cycle latency)
    output reg  [16:0] fb_raddr,
    input  wire [14:0] fb_rgb,          // RGB 5:5:5
    input  wire        fb_mask,
    // overlay
    input  wire        show_mask,
    input  wire        box_valid,
    input  wire [8:0]  box_x0,
    input  wire [8:0]  box_x1,
    input  wire [7:0]  box_y0,
    input  wire [7:0]  box_y1,
    input  wire        gate_open,
    // text overlay (character generator outside: lcd_text)
    output wire [4:0]  txt_idx,
    input  wire [7:0]  txt_char,
    // VGA
    output reg  [7:0]  vga_r,
    output reg  [7:0]  vga_g,
    output reg  [7:0]  vga_b,
    output reg         vga_hs,
    output reg         vga_vs,
    output reg         vga_blank_n,
    output wire        vga_sync_n,
    output reg         vga_clk
);
    // 640x480@60 timing (pixel clock 25 MHz)
    localparam H_VIS = 640, H_FP = 16, H_SYNC = 96, H_BP = 48, H_TOT = 800;
    localparam V_VIS = 480, V_FP = 10, V_SYNC = 2,  V_BP = 33, V_TOT = 525;

    assign vga_sync_n = 1'b0;           // no sync-on-green

    reg pix_en;
    always @(posedge clk) begin
        if (rst) begin pix_en <= 1'b0; vga_clk <= 1'b0; end
        else begin
            pix_en  <= ~pix_en;
            vga_clk <= ~pix_en;         // rises one 50 MHz cycle after data changes
        end
    end

    reg [9:0] hc, vc;
    always @(posedge clk) begin
        if (rst) begin hc <= 10'd0; vc <= 10'd0; end
        else if (pix_en) begin
            if (hc == H_TOT - 1) begin
                hc <= 10'd0;
                vc <= (vc == V_TOT - 1) ? 10'd0 : vc + 10'd1;
            end else
                hc <= hc + 10'd1;
        end
    end

    // ---- stage 1 : address frame buffer, compute syncs ----
    wire [8:0] sx = hc[9:1];            // 0..319
    wire [7:0] sy = vc[8:1];            // 0..239
    reg        de1, hs1, vs1, ind1, box1, txt1, tbg1;
`include "font5x7.vh"
    // text area: 16 characters x 2 lines, each cell 32x32 screen pixels
    // (5x7 glyph, 4x4 screen pixels per font pixel), at x 64..575, y 400..463
    localparam [9:0] TX0 = 10'd64, TY0 = 10'd400;
    wire [9:0] tx = hc - TX0;
    wire [9:0] ty = vc - TY0;
    wire       t_in  = (hc >= TX0) && (hc < TX0 + 10'd512) && (vc >= TY0) && (vc < TY0 + 10'd64);
    assign     txt_idx = {ty[5], tx[8:5]};               // line, column
    wire [2:0] gx = tx[4:2];                             // font column 0..7 (0..4 used)
    wire [2:0] gy = ty[4:2];                             // font row    0..7 (0..6 used)
    wire [34:0] gl = glyph(txt_char);
    wire [5:0] gbit = 6'd34 - (gy * 6'd5 + gx);
    wire       t_on = t_in && (gx < 3'd5) && (gy < 3'd7) && gl[gbit];
    // dark background strip behind the text
    wire       t_bg = (hc >= TX0 - 10'd8) && (hc < TX0 + 10'd520) && (vc >= TY0 - 10'd8) && (vc < TY0 + 10'd68);

    // box edges (2 frame-buffer pixels thick, just outside the object)
    wire in_x  = ({1'b0, sx} + 10'd2 >= {1'b0, box_x0}) && (sx <= box_x1 + 9'd2);
    wire in_y  = ({1'b0, sy} + 9'd2  >= {1'b0, box_y0}) && (sy <= box_y1 + 8'd2);
    wire on_vx = ({1'b0, sx} + 10'd2 == {1'b0, box_x0}) || ({1'b0, sx} + 10'd1 == {1'b0, box_x0}) ||
                 (sx == box_x1 + 9'd1) || (sx == box_x1 + 9'd2);
    wire on_hy = ({1'b0, sy} + 9'd2  == {1'b0, box_y0}) || ({1'b0, sy} + 9'd1  == {1'b0, box_y0}) ||
                 (sy == box_y1 + 8'd1) || (sy == box_y1 + 8'd2);
    wire on_box = box_valid && ((on_vx && in_y) || (on_hy && in_x));

    always @(posedge clk) begin
        if (rst) begin
            fb_raddr <= 17'd0; de1 <= 1'b0; hs1 <= 1'b1; vs1 <= 1'b1; ind1 <= 1'b0; box1 <= 1'b0;
            txt1 <= 1'b0; tbg1 <= 1'b0;
        end else if (pix_en) begin
            fb_raddr <= {sy, 8'd0} + {sy, 6'd0} + sx;          // sy*320 + sx
            de1  <= (hc < H_VIS) && (vc < V_VIS);
            hs1  <= ~((hc >= H_VIS + H_FP) && (hc < H_VIS + H_FP + H_SYNC));
            vs1  <= ~((vc >= V_VIS + V_FP) && (vc < V_VIS + V_FP + V_SYNC));
            ind1 <= (hc < 24) && (vc < 24);
            box1 <= on_box;
            txt1 <= t_on;
            tbg1 <= t_bg;
        end
    end

    // ---- stage 2 : colour, overlay, output registers ----
    wire [7:0] cam_r = {fb_rgb[14:10], fb_rgb[14:12]};
    wire [7:0] cam_g = {fb_rgb[9:5],   fb_rgb[9:7]};
    wire [7:0] cam_b = {fb_rgb[4:0],   fb_rgb[4:2]};

    always @(posedge clk) begin
        if (rst) begin
            vga_r <= 8'd0; vga_g <= 8'd0; vga_b <= 8'd0;
            vga_hs <= 1'b1; vga_vs <= 1'b1; vga_blank_n <= 1'b0;
        end else if (pix_en) begin
            vga_hs      <= hs1;
            vga_vs      <= vs1;
            vga_blank_n <= de1;
            if (!de1) begin
                vga_r <= 8'd0; vga_g <= 8'd0; vga_b <= 8'd0;
            end else if (txt1) begin
                vga_r <= 8'd255; vga_g <= 8'd255; vga_b <= 8'd0;       // yellow text
            end else if (tbg1) begin
                vga_r <= 8'd0;   vga_g <= 8'd0;   vga_b <= 8'd0;       // black strip
            end else if (box1) begin
                vga_r <= 8'd0; vga_g <= 8'd255; vga_b <= 8'd0;        // green box
            end else if (ind1) begin
                vga_r <= gate_open ? 8'd0 : 8'd255;
                vga_g <= gate_open ? 8'd255 : 8'd0;
                vga_b <= 8'd0;
            end else if (show_mask) begin
                vga_r <= {8{fb_mask}}; vga_g <= {8{fb_mask}}; vga_b <= {8{fb_mask}};
            end else begin
                vga_r <= cam_r; vga_g <= cam_g; vga_b <= cam_b;
            end
        end
    end
endmodule
