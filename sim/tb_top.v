// ============================================================================
// tb_top.v  -  system testbench for de2_115_shape_detect
// ----------------------------------------------------------------------------
// Models every external device of the design:
//   * TRDB-D5M : streams 640x480 12-bit Bayer frames read from img_<n>.hex
//                (FVAL/LVAL/D on its own 25 MHz pixel clock)
//   * MT9P001 serial interface : acknowledges and logs every register write
//   * HC-SR04  : returns an echo whose width matches the scheduled distance
//   * LCD      : decodes the HD44780 writes into a 16x2 text buffer
//   * VGA      : captures one complete 640x480 frame per test into a PPM file
// The schedule (image, segmentation mode, distance, frame count) is read from
// tests.txt, produced by gen_frames.py.  Per-frame results are printed as
// "RES ..." lines that check_results.py compares bit-exactly with the Python
// reference model.
// ============================================================================
`timescale 1ns/1ps
module tb_top;
    // ---------------- clocks ----------------
    reg clk50 = 1'b0;
    always #10 clk50 = ~clk50;            // 50 MHz
    reg pclk = 1'b0;
    always #20 pclk = ~pclk;              // 25 MHz camera pixel clock

    // ---------------- DUT ----------------
    reg  [3:0]  KEY = 4'hE;               // KEY[0] pressed = reset
    reg  [17:0] SW  = 18'd0;
    wire [8:0]  LEDG;
    wire [17:0] LEDR;
    wire [6:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7;
    wire [7:0]  LCD_DATA;
    wire        LCD_EN, LCD_RS, LCD_RW, LCD_ON, LCD_BLON;
    wire [7:0]  VGA_R, VGA_G, VGA_B;
    wire        VGA_CLK, VGA_BLANK_N, VGA_SYNC_N, VGA_HS, VGA_VS;
    wire [35:0] GPIO;
    wire [6:0]  EX_IO;

    de2_115_shape_detect #(.SIM(1)) dut (
        .CLOCK_50(clk50), .KEY(KEY), .SW(SW), .LEDG(LEDG), .LEDR(LEDR),
        .HEX0(HEX0), .HEX1(HEX1), .HEX2(HEX2), .HEX3(HEX3),
        .HEX4(HEX4), .HEX5(HEX5), .HEX6(HEX6), .HEX7(HEX7),
        .LCD_DATA(LCD_DATA), .LCD_EN(LCD_EN), .LCD_RS(LCD_RS), .LCD_RW(LCD_RW),
        .LCD_ON(LCD_ON), .LCD_BLON(LCD_BLON),
        .VGA_R(VGA_R), .VGA_G(VGA_G), .VGA_B(VGA_B), .VGA_CLK(VGA_CLK),
        .VGA_BLANK_N(VGA_BLANK_N), .VGA_SYNC_N(VGA_SYNC_N), .VGA_HS(VGA_HS), .VGA_VS(VGA_VS),
        .GPIO(GPIO), .EX_IO(EX_IO)
    );

    // ---------------- camera model ----------------
    reg [11:0] cam_d = 12'd0;
    reg        cam_fval = 1'b0, cam_lval = 1'b0;
    assign GPIO[0]  = pclk;
    assign GPIO[1]  = cam_d[11];
    assign GPIO[3]  = cam_d[10];
    assign GPIO[4]  = cam_d[9];
    assign GPIO[5]  = cam_d[8];
    assign GPIO[6]  = cam_d[7];
    assign GPIO[7]  = cam_d[6];
    assign GPIO[8]  = cam_d[5];
    assign GPIO[9]  = cam_d[4];
    assign GPIO[10] = cam_d[3];
    assign GPIO[11] = cam_d[2];
    assign GPIO[12] = cam_d[1];
    assign GPIO[13] = cam_d[0];
    assign GPIO[21] = cam_lval;
    assign GPIO[22] = cam_fval;

    localparam HB = 100;                  // line blanking (pixel clocks)
    localparam VB = 4000;                 // frame blanking (pixel clocks)
    reg [11:0] frame_mem [0:640*480-1];
    integer ln, px;

    task send_frame;
        begin
            @(negedge pclk) cam_fval <= 1'b1;
            repeat (20) @(negedge pclk);
            for (ln = 0; ln < 480; ln = ln + 1) begin
                for (px = 0; px < 640; px = px + 1) begin
                    @(negedge pclk);
                    cam_lval <= 1'b1;
                    cam_d    <= frame_mem[ln*640 + px];
                end
                @(negedge pclk) cam_lval <= 1'b0;
                repeat (HB - 1) @(negedge pclk);
            end
            repeat (10) @(negedge pclk);
            cam_fval <= 1'b0;
            repeat (VB) @(negedge pclk);
        end
    endtask

    // ---------------- MT9P001 serial-interface model ----------------
    pullup (GPIO[23]);
    reg       i2c_ack = 1'b0;
    assign GPIO[23] = i2c_ack ? 1'b0 : 1'bz;
    wire      sda = GPIO[23];
    wire      scl = GPIO[24];
    reg       i2c_busy = 1'b0;
    reg [7:0] i2c_sh;
    reg [7:0] i2c_bytes [0:3];
    integer   i2c_bit = 0, i2c_nb = 0, i2c_writes = 0;

    always @(negedge sda) if (scl === 1'b1) begin     // START
        i2c_busy = 1'b1; i2c_bit = 0; i2c_nb = 0;
    end
    always @(posedge sda) if (scl === 1'b1 && i2c_busy) begin   // STOP
        i2c_busy = 1'b0;
        if (i2c_nb == 4) begin
            i2c_writes = i2c_writes + 1;
            $display("[%0t] I2C  dev=%02h reg=%02h data=%02h%02h", $time,
                     i2c_bytes[0], i2c_bytes[1], i2c_bytes[2], i2c_bytes[3]);
        end else
            $display("[%0t] I2C  ERROR: incomplete transfer (%0d bytes)", $time, i2c_nb);
    end
    always @(posedge scl) if (i2c_busy) begin
        if (i2c_bit < 8) i2c_sh = {i2c_sh[6:0], sda};
        i2c_bit = i2c_bit + 1;
    end
    always @(negedge scl) if (i2c_busy) begin
        if (i2c_bit == 8) begin
            if (i2c_nb < 4) i2c_bytes[i2c_nb] = i2c_sh;
            i2c_nb  = i2c_nb + 1;
            #50 i2c_ack = 1'b1;                       // acknowledge
        end else if (i2c_bit == 9) begin
            #50 i2c_ack = 1'b0;
            i2c_bit = 0;
        end
    end

    // ---------------- HC-SR04 model ----------------
    reg     echo = 1'b0;
    integer dist_cm = 45;
    assign  EX_IO[1] = echo;
    always @(negedge EX_IO[0]) begin                  // end of TRIG pulse
        if (dist_cm > 0) begin
            #500_000;                                 // 0.5 ms burst time
            echo = 1'b1;
            #(dist_cm * 58_300);                      // 58.3 us per cm
            echo = 1'b0;
        end
    end

    // ---------------- LCD model ----------------
    reg [7:0] lcd_mem [0:31];
    integer   lcd_addr = 0, li;
    initial for (li = 0; li < 32; li = li + 1) lcd_mem[li] = " ";
    always @(negedge LCD_EN) begin
        if (!LCD_RS) begin
            if (LCD_DATA[7]) lcd_addr = (LCD_DATA[6] ? 16 : 0) + LCD_DATA[3:0];
        end else begin
            if (lcd_addr < 32) lcd_mem[lcd_addr] = LCD_DATA;
            lcd_addr = lcd_addr + 1;
        end
    end
    task show_lcd;
        begin
            $write("LCD  |");
            for (li = 0; li < 16; li = li + 1) $write("%s", lcd_mem[li]);
            $write("|  |");
            for (li = 16; li < 32; li = li + 1) $write("%s", lcd_mem[li]);
            $write("|\n");
        end
    endtask

    // ---------------- VGA capture ----------------
    reg [23:0] vga_img [0:640*480-1];
    integer vx = 0, vy = -1, vga_lines = 0;
    reg     blank_d = 1'b0, vs_d = 1'b1;
    always @(posedge VGA_CLK) begin
        if (!VGA_VS && vs_d) vy = -1;                 // vertical sync: new frame
        if (VGA_BLANK_N && !blank_d) begin vx = 0; vy = vy + 1; end
        if (VGA_BLANK_N && vy >= 0 && vy < 480 && vx < 640) begin
            vga_img[vy*640 + vx] = {VGA_R, VGA_G, VGA_B};
            vx = vx + 1;
        end
        blank_d = VGA_BLANK_N;
        vs_d    = VGA_VS;
    end
    integer fd, pi;
    task dump_vga;                                    // capture one full frame
        input [8*64-1:0] fname;
        begin
            @(negedge VGA_VS);
            @(negedge VGA_VS);
            fd = $fopen(fname, "w");
            $fwrite(fd, "P3\n640 480\n255\n");
            for (pi = 0; pi < 640*480; pi = pi + 1)
                $fwrite(fd, "%0d %0d %0d\n", vga_img[pi][23:16], vga_img[pi][15:8], vga_img[pi][7:0]);
            $fclose(fd);
        end
    endtask

    // ---------------- result monitor ----------------
    integer cur_test = -1, frame_no = 0;
    always @(posedge pclk) begin
        if (dut.u_shape.res_strobe)
            $display("RES test=%0d frame=%0d gate=%0d T=%0d raw=%0d class=%0d obj=%0d box=%0d,%0d,%0d,%0d area=%0d I1=%0d",
                cur_test, frame_no, dut.u_seg.gate_l, dut.u_seg.threshold,
                dut.u_shape.res_raw, dut.u_shape.res_class, dut.u_shape.res_obj,
                dut.u_shape.res_xmin, dut.u_shape.res_ymin, dut.u_shape.res_xmax, dut.u_shape.res_ymax,
                dut.u_shape.res_area, dut.u_shape.I1);
    end

    // ---------------- test sequence ----------------
    integer tf, rc, t_id, t_mode, t_dist, t_nfr, f;
    reg [8*16-1:0] t_name, t_exp;
    reg [8*64-1:0] fname;
    initial begin
        $display("=== DE2-115 shape detection testbench ===");
        #2000 KEY = 4'hF;                             // release reset
        wait (dut.cam_cfg_done === 1'b1);
        $display("[%0t] camera configured, %0d register writes, ack_error=%0b",
                 $time, i2c_writes, dut.cam_ack_err);

        tf = $fopen("tests.txt", "r");
        while (!$feof(tf)) begin
            rc = $fscanf(tf, "%d %s %d %d %s %d\n", t_id, t_name, t_mode, t_dist, t_exp, t_nfr);
            if (rc == 6) begin
                $sformat(fname, "img_%0d.hex", t_id);
                $readmemh(fname, frame_mem);
                SW[1:0]  = t_mode[1:0];
                dist_cm  = t_dist;
                cur_test = t_id;
                $display("[%0t] ---- test %0d: %0s  mode=%0d  distance=%0d cm  expect %0s",
                         $time, t_id, t_name, t_mode, t_dist, t_exp);
                for (f = 0; f < t_nfr; f = f + 1) begin
                    send_frame;
                    frame_no = frame_no + 1;
                end
                // camera idles while the VGA frame and LCD are sampled
                $sformat(fname, "vga_%0d.ppm", t_id);
                dump_vga(fname);
                show_lcd;
                $display("FINAL test=%0d expect=%0s class=%0d lcd_class_leds=%b dist=%0d near=%0b",
                         t_id, t_exp, dut.show_class, LEDR[8:0], dut.dist_cm, dut.near);
            end
        end
        $fclose(tf);
        $display("=== done ===");
        $finish;
    end
endmodule
