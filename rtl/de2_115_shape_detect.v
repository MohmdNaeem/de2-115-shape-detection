// ============================================================================
// de2_115_shape_detect.v  -  TOP LEVEL
// ----------------------------------------------------------------------------
// Distance-gated geometric shape recognition on the Terasic DE2-115.
//
//   TRDB-D5M camera (GPIO header) --> 640x480 Bayer --> 320x240 RGB
//        |                                   |
//        |                                   +--> frame buffer (on-chip) --> VGA + green box
//        |                                   |
//        |                                   +--> segmentation (auto threshold)
//        |                                          --> 3x3 opening + closing
//        |                                          --> shape analyzer (moments + profiles)
//        |                                          --> class + bounding box
//   HC-SR04 ultrasonic (EX_IO) --> distance --> gate (<= 75 cm)
//   16x2 LCD  <-- shape name + distance
//
// Port names match Terasic's DE2-115 pin-assignment file
// (DE2_115_pin_assignments.csv from the DE2-115 System CD): import it with
// Assignments > Import Assignments, or use the provided .qsf.
//
// ---------------------------------------------------------------------------
// Controls
//   KEY[0]     reset (active low)
//   SW[1:0]    segmentation feature: 00 dark object on light background
//                                    01 light object on dark background
//                                    10 coloured object (chroma)
//   SW[2]      1 = manual threshold (SW[10:3]), 0 = automatic (ISODATA)
//   SW[10:3]   manual threshold value
//   SW[11]     VGA shows the binary mask instead of the camera image
//   SW[12]     bypass the distance gate (bench testing without the sensor)
//   SW[16:13]  camera exposure (shutter = 256 + 64 * SW[16:13] row times)
// Indicators
//   LCD        line 1: shape name / "OUT OF RANGE", line 2: distance + status
//   HEX2..0    distance in cm          HEX5..4  current threshold (hex)
//   HEX6       class code (0 none ... 8 unknown)
//   LEDG[0]    object within 75 cm     LEDG[1]  object detected
//   LEDG[2]    scene contrast OK       LEDG[7]  camera I2C NACK (error)
//   LEDG[8]    camera configured       LEDR[8:0] one-hot class, LEDR[17] frame tick
//
// Wiring
//   TRDB-D5M on the 40-pin GPIO header (JP5), header I/O set to 3.3 V.
//   HC-SR04: TRIG <- EX_IO[0], ECHO -> EX_IO[1] through a 5V->3.3V divider
//            (e.g. 1 kOhm series, 2 kOhm to GND), VCC = 5 V, common GND.
// ============================================================================
module de2_115_shape_detect #(
    parameter SIM          = 0,        // 1 = shortened delays for simulation
    parameter BAYER        = 0,        // Bayer phase, see bayer_to_rgb.v
    parameter NEAR_CM      = 75        // distance gate threshold (cm)
)(
    input  wire        CLOCK_50,
    input  wire [3:0]  KEY,
    input  wire [17:0] SW,
    output wire [8:0]  LEDG,
    output wire [17:0] LEDR,
    output wire [6:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7,
    // character LCD
    output wire [7:0]  LCD_DATA,
    output wire        LCD_EN,
    output wire        LCD_RS,
    output wire        LCD_RW,
    output wire        LCD_ON,
    output wire        LCD_BLON,
    // VGA (ADV7123)
    output wire [7:0]  VGA_R,
    output wire [7:0]  VGA_G,
    output wire [7:0]  VGA_B,
    output wire        VGA_CLK,
    output wire        VGA_BLANK_N,
    output wire        VGA_SYNC_N,
    output wire        VGA_HS,
    output wire        VGA_VS,
    // TRDB-D5M on GPIO header
    inout  wire [35:0] GPIO,
    // ultrasonic sensor on the 14-pin expansion header
    inout  wire [6:0]  EX_IO
);
    // =========================================================================
    // 1. Pin mapping
    // =========================================================================
    // TRDB-D5M signals on the GPIO header (same mapping as Terasic's
    // DE2-115 camera reference design)
    wire [11:0] D5M_D;
    wire        D5M_FVAL, D5M_LVAL, D5M_PIXLCLK;
    wire        d5m_xclkin, d5m_reset_n, d5m_sclk, d5m_sda_oe;

    assign D5M_PIXLCLK = GPIO[0];
    assign D5M_D[11]   = GPIO[1];
    assign D5M_D[10]   = GPIO[3];
    assign D5M_D[9]    = GPIO[4];
    assign D5M_D[8]    = GPIO[5];
    assign D5M_D[7]    = GPIO[6];
    assign D5M_D[6]    = GPIO[7];
    assign D5M_D[5]    = GPIO[8];
    assign D5M_D[4]    = GPIO[9];
    assign D5M_D[3]    = GPIO[10];
    assign D5M_D[2]    = GPIO[11];
    assign D5M_D[1]    = GPIO[12];
    assign D5M_D[0]    = GPIO[13];
    assign D5M_LVAL    = GPIO[21];
    assign D5M_FVAL    = GPIO[22];

    assign GPIO[16]    = d5m_xclkin;               // XCLKIN  (25 MHz)
    assign GPIO[17]    = d5m_reset_n;              // RESET_N
    assign GPIO[19]    = 1'b1;                     // TRIGGER (unused, keep high)
    assign GPIO[23]    = d5m_sda_oe ? 1'b0 : 1'bz; // SDATA   (open drain)
    assign GPIO[24]    = d5m_sclk;                 // SCLK

    // all other GPIO pins are inputs / unused
    assign GPIO[15:0]  = 16'hzzzz;
    assign GPIO[18]    = 1'bz;
    assign GPIO[22:20] = 3'bzzz;
    assign GPIO[35:25] = 11'bzzzzzzzzzzz;

    // HC-SR04
    wire us_trig;
    wire us_echo = EX_IO[1];
    assign EX_IO[0]   = us_trig;
    assign EX_IO[6:1] = 6'bzzzzzz;

    // =========================================================================
    // 2. Clocks and resets
    // =========================================================================
    wire clk  = CLOCK_50;                  // system clock (VGA, LCD, sensor, I2C)
    wire pclk = D5M_PIXLCLK;               // camera pixel clock (25 MHz)

    wire rst;                              // CLOCK_50 domain
    reset_sync u_rst_sys (.clk(clk), .arst(~KEY[0]), .rst(rst));

    wire prst;                             // pixel-clock domain
    reset_sync u_rst_pix (.clk(pclk), .arst(rst), .rst(prst));

    // 25 MHz camera master clock
    reg xclk_r;
    always @(posedge clk) xclk_r <= rst ? 1'b0 : ~xclk_r;
    assign d5m_xclkin = xclk_r;

    // camera hardware reset: low during system reset and 1 ms afterwards
    localparam CAM_RST_CYC = SIM ? 100 : 50_000;
    reg [16:0] cam_rst_cnt;
    always @(posedge clk) begin
        if (rst)                           cam_rst_cnt <= 17'd0;
        else if (cam_rst_cnt < CAM_RST_CYC) cam_rst_cnt <= cam_rst_cnt + 17'd1;
    end
    assign d5m_reset_n = (cam_rst_cnt >= CAM_RST_CYC);

    // switches into each domain
    wire [17:0] sw_s;                      // CLOCK_50 domain
    sync_bits #(.W(18)) u_sw_sys (.clk(clk), .d(SW), .q(sw_s));
    wire [17:0] sw_p;                      // pixel domain
    sync_bits #(.W(18)) u_sw_pix (.clk(pclk), .d(SW), .q(sw_p));

    // =========================================================================
    // 3. Camera configuration (I2C)
    // =========================================================================
    wire        cam_cfg_done, cam_ack_err;
    wire [15:0] exposure = 16'd256 + {6'd0, sw_s[16:13], 6'd0};

    d5m_i2c_config #(
        .CLK_HZ  (50_000_000),
        .I2C_HZ  (SIM ? 1_000_000 : 50_000),
        .WAIT_US (SIM ? 5 : 2_000)
    ) u_cam_cfg (
        .clk(clk), .rst(rst | ~d5m_reset_n),
        .exposure(exposure),
        .sclk(d5m_sclk), .sda_oe(d5m_sda_oe), .sda_in(GPIO[23]),
        .config_done(cam_cfg_done), .ack_error(cam_ack_err)
    );

    // =========================================================================
    // 4. Ultrasonic distance gate (CLOCK_50 domain)
    // =========================================================================
    wire [9:0] dist_cm;
    wire       dist_valid, near, us_strobe;

    ultrasonic_hcsr04 #(
        .CLK_HZ    (50_000_000),
        .PERIOD_MS (SIM ? 12 : 60),
        .TIMEOUT_MS(SIM ? 10 : 30),
        .NEAR_CM   (NEAR_CM)
    ) u_us (
        .clk(clk), .rst(rst), .echo(us_echo), .trig(us_trig),
        .dist_cm(dist_cm), .dist_valid(dist_valid), .near(near),
        .sample_strobe(us_strobe)
    );

    wire gate_bypass = sw_s[12];
    wire gate_open   = near | gate_bypass;          // CLOCK_50 domain
    wire gate_p;                                    // pixel domain
    sync_bits #(.W(1)) u_gate_sync (.clk(pclk), .d(gate_open), .q(gate_p));

    // =========================================================================
    // 5. Camera capture and pre-processing (pixel-clock domain)
    // =========================================================================
    wire [11:0] cap_pix, cap_x, cap_y;
    wire        cap_valid, frame_start, frame_end;
    wire [15:0] frame_count;

    d5m_capture u_cap (
        .pclk(pclk), .rst(prst),
        .d5m_d(D5M_D), .d5m_fval(D5M_FVAL), .d5m_lval(D5M_LVAL),
        .pix(cap_pix), .pix_valid(cap_valid), .x(cap_x), .y(cap_y),
        .frame_start(frame_start), .frame_end(frame_end), .frame_count(frame_count)
    );

    // frame_done: frame_end delayed until every pipeline stage has drained
    reg [63:0] fe_dly;
    always @(posedge pclk) fe_dly <= prst ? 64'd0 : {fe_dly[62:0], frame_end};
    wire frame_done = fe_dly[63];

    wire [7:0] rgb_r, rgb_g, rgb_b;
    wire       rgb_valid;
    wire [8:0] rgb_x;
    wire [7:0] rgb_y;

    bayer_to_rgb #(.BAYER(BAYER)) u_bayer (
        .clk(pclk), .rst(prst),
        .in_pix(cap_pix), .in_valid(cap_valid), .in_x(cap_x), .in_y(cap_y),
        .r(rgb_r), .g(rgb_g), .b(rgb_b),
        .out_valid(rgb_valid), .out_x(rgb_x), .out_y(rgb_y)
    );

    // ---- segmentation ----
    wire       seg_valid, seg_bit, frame_enabled, contrast_ok;
    wire [8:0] seg_x;
    wire [7:0] seg_y, threshold;

    pixel_segment u_seg (
        .clk(pclk), .rst(prst),
        .in_valid(rgb_valid), .in_x(rgb_x), .in_y(rgb_y),
        .r(rgb_r), .g(rgb_g), .b(rgb_b),
        .frame_start(frame_start), .frame_done(frame_done),
        .mode(sw_p[1:0]), .manual_en(sw_p[2]), .manual_th(sw_p[10:3]),
        .gate(gate_p),
        .out_valid(seg_valid), .out_bit(seg_bit), .out_x(seg_x), .out_y(seg_y),
        .threshold(threshold), .contrast_ok(contrast_ok), .frame_enabled(frame_enabled)
    );

    // ---- morphology: opening (erode, dilate) then closing (dilate, erode) ----
    wire       m1_v, m1_b, m2_v, m2_b, m3_v, m3_b, m4_v, m4_b;
    wire [8:0] m1_x, m2_x, m3_x, m4_x;
    wire [7:0] m1_y, m2_y, m3_y, m4_y;

    morph3x3 #(.OP(0)) u_m1 (.clk(pclk), .rst(prst), .in_valid(seg_valid), .in_bit(seg_bit),
        .in_x(seg_x), .in_y(seg_y), .out_valid(m1_v), .out_bit(m1_b), .out_x(m1_x), .out_y(m1_y));
    morph3x3 #(.OP(1)) u_m2 (.clk(pclk), .rst(prst), .in_valid(m1_v), .in_bit(m1_b),
        .in_x(m1_x), .in_y(m1_y), .out_valid(m2_v), .out_bit(m2_b), .out_x(m2_x), .out_y(m2_y));
    morph3x3 #(.OP(1)) u_m3 (.clk(pclk), .rst(prst), .in_valid(m2_v), .in_bit(m2_b),
        .in_x(m2_x), .in_y(m2_y), .out_valid(m3_v), .out_bit(m3_b), .out_x(m3_x), .out_y(m3_y));
    morph3x3 #(.OP(0)) u_m4 (.clk(pclk), .rst(prst), .in_valid(m3_v), .in_bit(m3_b),
        .in_x(m3_x), .in_y(m3_y), .out_valid(m4_v), .out_bit(m4_b), .out_x(m4_x), .out_y(m4_y));

    // ---- shape analysis ----
    wire        res_strobe, res_obj;
    wire [3:0]  res_class, res_raw;
    wire [8:0]  res_xmin, res_xmax;
    wire [7:0]  res_ymin, res_ymax;
    wire [16:0] res_area;

    shape_analyzer u_shape (
        .clk(pclk), .rst(prst),
        .frame_start(frame_start), .frame_done(frame_done),
        .in_valid(m4_v), .in_bit(m4_b), .in_x(m4_x), .in_y(m4_y),
        .res_strobe(res_strobe), .res_class(res_class), .res_raw(res_raw), .res_obj(res_obj),
        .res_xmin(res_xmin), .res_xmax(res_xmax), .res_ymin(res_ymin), .res_ymax(res_ymax),
        .res_area(res_area)
    );

    // =========================================================================
    // 6. Frame buffer (camera image + mask), written in the pixel domain
    // =========================================================================
    wire [16:0] fb_waddr = {rgb_y, 8'd0} + {rgb_y, 6'd0} + rgb_x;     // y*320 + x
    wire [16:0] mk_waddr = {m4_y, 8'd0}  + {m4_y, 6'd0}  + m4_x;
    wire [16:0] fb_raddr;
    wire [14:0] fb_rgb;
    wire        fb_mask;

    dual_clock_ram #(.DW(15), .AW(17), .DEPTH(76800)) u_fb_rgb (
        .wclk(pclk), .we(rgb_valid), .waddr(fb_waddr),
        .wdata({rgb_r[7:3], rgb_g[7:3], rgb_b[7:3]}),
        .rclk(clk), .raddr(fb_raddr), .rdata(fb_rgb)
    );
    dual_clock_ram #(.DW(1), .AW(17), .DEPTH(76800)) u_fb_mask (
        .wclk(pclk), .we(m4_v), .waddr(mk_waddr), .wdata(m4_b),
        .rclk(clk), .raddr(fb_raddr), .rdata(fb_mask)
    );

    // =========================================================================
    // 7. Results: pixel domain -> CLOCK_50 domain
    // =========================================================================
    wire [47:0] res_bus_p = {res_class, res_obj, res_xmin, res_xmax, res_ymin, res_ymax,
                             threshold, contrast_ok};
    wire [47:0] res_bus;
    wire        res_strobe_s;
    cdc_bus #(.W(48)) u_res_cdc (
        .src_clk(pclk), .src_rst(prst), .src_strobe(res_strobe), .src_data(res_bus_p),
        .dst_clk(clk), .dst_rst(rst), .dst_data(res_bus), .dst_strobe(res_strobe_s)
    );
    wire [3:0] s_class    = res_bus[47:44];
    wire       s_obj      = res_bus[43];
    wire [8:0] s_xmin     = res_bus[42:34];
    wire [8:0] s_xmax     = res_bus[33:25];
    wire [7:0] s_ymin     = res_bus[24:17];
    wire [7:0] s_ymax     = res_bus[16:9];
    wire [7:0] s_thresh   = res_bus[8:1];
    wire       s_contrast = res_bus[0];

    // what is shown: nothing while the gate is closed
    wire [3:0] show_class = gate_open ? s_class : 4'd0;
    wire       show_box   = gate_open & s_obj;

    // =========================================================================
    // 8. VGA output with bounding-box overlay
    // =========================================================================
    wire [4:0] vga_tidx;                   // VGA text overlay character position
    wire [7:0] vga_tchar;
    vga_display u_vga (
        .clk(clk), .rst(rst),
        .fb_raddr(fb_raddr), .fb_rgb(fb_rgb), .fb_mask(fb_mask),
        .show_mask(sw_s[11]),
        .box_valid(show_box), .box_x0(s_xmin), .box_x1(s_xmax), .box_y0(s_ymin), .box_y1(s_ymax),
        .gate_open(gate_open),
        .txt_idx(vga_tidx), .txt_char(vga_tchar),
        .vga_r(VGA_R), .vga_g(VGA_G), .vga_b(VGA_B),
        .vga_hs(VGA_HS), .vga_vs(VGA_VS), .vga_blank_n(VGA_BLANK_N),
        .vga_sync_n(VGA_SYNC_N), .vga_clk(VGA_CLK)
    );

    // =========================================================================
    // 9. Character LCD
    // =========================================================================
    wire [4:0]  lcd_idx;
    wire [7:0]  lcd_char;
    wire [11:0] dist_bcd;                  // distance in BCD (also used by HEX2..0)
    reg  [11:0] dist_bcd_r;
    bin2bcd10 u_bcd (.bin(dist_cm), .bcd(dist_bcd));
    always @(posedge clk) dist_bcd_r <= dist_bcd;

    // same text on the VGA screen (second character generator)
    lcd_text u_vga_text (
        .char_idx(vga_tidx), .shape_class(show_class), .in_range(gate_open),
        .bypass(gate_bypass), .dist_bcd(dist_bcd_r), .dist_valid(dist_valid),
        .char_code(vga_tchar)
    );

    lcd_text u_lcd_text (
        .char_idx(lcd_idx), .shape_class(show_class), .in_range(gate_open),
        .bypass(gate_bypass), .dist_bcd(dist_bcd_r), .dist_valid(dist_valid),
        .char_code(lcd_char)
    );

    lcd_hd44780 #(.CLK_HZ(50_000_000), .TIME_DIV(SIM ? 50 : 1)) u_lcd (
        .clk(clk), .rst(rst),
        .char_idx(lcd_idx), .char_code(lcd_char),
        .lcd_data(LCD_DATA), .lcd_rs(LCD_RS), .lcd_rw(LCD_RW), .lcd_en(LCD_EN),
        .lcd_on(LCD_ON), .lcd_blon(LCD_BLON)
    );

    // =========================================================================
    // 10. 7-segment displays and LEDs
    // =========================================================================
    hex7seg u_h0 (.hex(dist_bcd_r[3:0]),  .blank(~dist_valid), .seg(HEX0));
    hex7seg u_h1 (.hex(dist_bcd_r[7:4]),  .blank(~dist_valid), .seg(HEX1));
    hex7seg u_h2 (.hex(dist_bcd_r[11:8]), .blank(~dist_valid), .seg(HEX2));
    hex7seg u_h3 (.hex(4'h0),           .blank(1'b1),        .seg(HEX3));
    hex7seg u_h4 (.hex(s_thresh[3:0]),  .blank(1'b0),        .seg(HEX4));
    hex7seg u_h5 (.hex(s_thresh[7:4]),  .blank(1'b0),        .seg(HEX5));
    hex7seg u_h6 (.hex(show_class),     .blank(1'b0),        .seg(HEX6));
    hex7seg u_h7 (.hex(4'h0),           .blank(1'b1),        .seg(HEX7));

    reg frame_tick;
    always @(posedge clk) frame_tick <= rst ? 1'b0 : (frame_tick ^ res_strobe_s);

    assign LEDG[0]    = gate_open;
    assign LEDG[1]    = show_box;
    assign LEDG[2]    = s_contrast;
    assign LEDG[6:3]  = 4'd0;
    assign LEDG[7]    = cam_ack_err;
    assign LEDG[8]    = cam_cfg_done;

    genvar gi;
    generate
        for (gi = 0; gi < 9; gi = gi + 1) begin : g_cls_led
            assign LEDR[gi] = (show_class == gi);
        end
    endgenerate
    assign LEDR[16:9] = 8'd0;
    assign LEDR[17]   = frame_tick;
endmodule
