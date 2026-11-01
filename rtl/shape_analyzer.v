// ============================================================================
// shape_analyzer.v
// ----------------------------------------------------------------------------
// Measures the object in the cleaned binary mask and classifies its shape.
//
// A) While the frame streams in (one pass, no frame buffer needed):
//      * area A and bounding box (xmin, xmax, ymin, ymax)
//      * raw moments  Sx, Sy, Sxx, Syy, Sxy
//      * row profile  : number of object pixels in every row    (240 x 9 bit RAM)
//      * column profile: number of object pixels in every column (320 x 8 bit RAM)
//    Profile RAM entries carry a frame-parity tag, so they never need to be
//    cleared: an entry written in an older frame simply reads as zero.
//
// B) During vertical blanking a small sequencer computes
//      * the affine moment invariant
//            I1 = (mu20*mu02 - mu11^2) / mu00^4          (Q32 fixed point)
//        which depends only on the shape family, NOT on size, position,
//        rotation, aspect ratio or (mild) perspective:
//            ellipse/circle 0.00633, hexagon 0.00643, pentagon 0.00655,
//            any rectangle 0.00694, any triangle 0.00926
//      * the elongation ((mu20-mu02)^2 + 4 mu11^2) / (mu20+mu02)^2
//        (0 for square/circle, 0.36 for a 2:1 rectangle) - rotation invariant
//      * 14 normalised profile samples (row widths at 1/8..7/8 of the height
//        divided by the box width, and the same for columns)
//
//    Decision:
//      I1 > REJ_TH                      -> UNKNOWN (stars, several blobs ...)
//      I1 > TRI_TH                      -> TRIANGLE
//      I1 > QUAD_TH                     -> SQUARE, or RECTANGLE if elongated
//      elongated                        -> ELLIPSE
//      otherwise nearest template (profile SAD + weighted |I1 - I1_t|)
//                                       -> CIRCLE / PENTAGON / HEXAGON
//
// C) Temporal filter: the reported class changes only after the same raw
//    result is seen in two consecutive frames.
//
// Class codes: 0 NONE, 1 CIRCLE, 2 SQUARE, 3 RECTANGLE, 4 TRIANGLE,
//              5 PENTAGON, 6 HEXAGON, 7 ELLIPSE, 8 UNKNOWN
// ============================================================================
module shape_analyzer #(
    parameter IMG_W    = 320,
    parameter IMG_H    = 240,
    parameter MIN_AREA = 300,          // smaller blobs are ignored
    parameter MIN_DIM  = 16            // minimum box width / height
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        frame_start,    // start of camera frame
    input  wire        frame_done,     // all mask pixels of the frame delivered
    input  wire        in_valid,
    input  wire        in_bit,
    input  wire [8:0]  in_x,
    input  wire [7:0]  in_y,
    output reg         res_strobe,     // one pulse per analysed frame
    output reg  [3:0]  res_class,      // temporally filtered class
    output reg  [3:0]  res_raw,        // class of this frame
    output reg         res_obj,        // object present (bounding box valid)
    output reg  [8:0]  res_xmin,
    output reg  [8:0]  res_xmax,
    output reg  [7:0]  res_ymin,
    output reg  [7:0]  res_ymax,
    output reg  [16:0] res_area
);
    `include "shape_templates.vh"

    localparam C_NONE = 4'd0, C_CIRCLE = 4'd1, C_SQUARE = 4'd2, C_RECT = 4'd3,
               C_TRI  = 4'd4, C_PENT   = 4'd5, C_HEX    = 4'd6, C_ELLIPSE = 4'd7,
               C_UNKNOWN = 4'd8;

    // ==================================================================
    // A) streaming accumulation
    // ==================================================================
    reg        par;                     // frame parity tag
    reg        fsm_rd;                  // sequencer owns the RAM read ports

    // stage 1 : register pixel, multiply coordinates
    reg        p_v, p_b;
    reg [8:0]  p_x;
    reg [7:0]  p_y;
    reg [16:0] p_xx, p_yy, p_xy;
    always @(posedge clk) begin
        if (rst) begin
            p_v <= 1'b0; p_b <= 1'b0; p_x <= 9'd0; p_y <= 8'd0;
            p_xx <= 17'd0; p_yy <= 17'd0; p_xy <= 17'd0;
        end else begin
            p_v  <= in_valid;
            p_b  <= in_bit;
            p_x  <= in_x;
            p_y  <= in_y;
            p_xx <= in_x * in_x;
            p_yy <= in_y * in_y;
            p_xy <= in_x * in_y;
        end
    end

    // stage 2 : accumulate
    reg [16:0] acc_a;
    reg [25:0] acc_sx, acc_sy;
    reg [33:0] acc_sxx, acc_syy, acc_sxy;
    reg [8:0]  acc_xmin, acc_xmax;
    reg [7:0]  acc_ymin, acc_ymax;
    wire       fg = p_v & p_b;

    always @(posedge clk) begin
        if (rst) begin
            par <= 1'b0;
            acc_a <= 17'd0; acc_sx <= 26'd0; acc_sy <= 26'd0;
            acc_sxx <= 34'd0; acc_syy <= 34'd0; acc_sxy <= 34'd0;
            acc_xmin <= 9'h1FF; acc_xmax <= 9'd0; acc_ymin <= 8'hFF; acc_ymax <= 8'd0;
        end else if (frame_start) begin
            par <= ~par;
            acc_a <= 17'd0; acc_sx <= 26'd0; acc_sy <= 26'd0;
            acc_sxx <= 34'd0; acc_syy <= 34'd0; acc_sxy <= 34'd0;
            acc_xmin <= 9'h1FF; acc_xmax <= 9'd0; acc_ymin <= 8'hFF; acc_ymax <= 8'd0;
        end else if (fg) begin
            acc_a   <= acc_a + 17'd1;
            acc_sx  <= acc_sx + p_x;
            acc_sy  <= acc_sy + p_y;
            acc_sxx <= acc_sxx + p_xx;
            acc_syy <= acc_syy + p_yy;
            acc_sxy <= acc_sxy + p_xy;
            if (p_x < acc_xmin) acc_xmin <= p_x;
            if (p_x > acc_xmax) acc_xmax <= p_x;
            if (p_y < acc_ymin) acc_ymin <= p_y;
            if (p_y > acc_ymax) acc_ymax <= p_y;
        end
    end

    // ---- row profile RAM : {tag, count[8:0]} ----
    reg [9:0]  row_ram [0:IMG_H-1];
    reg [9:0]  row_q;
    reg [7:0]  row_raddr;
    reg        row_we;
    reg [7:0]  row_waddr;
    reg [9:0]  row_wdata;
    reg [8:0]  rc;                      // count of current row
    reg [7:0]  ry;                      // current row index
    reg        row_open;

    // ---- column profile RAM : {tag, count[7:0]} ----
    reg [8:0]  col_ram [0:IMG_W-1];
    reg [8:0]  col_q;
    reg [8:0]  col_raddr_fsm;
    wire [8:0] col_raddr = fsm_rd ? col_raddr_fsm : in_x;

    integer k;
    initial begin
        for (k = 0; k < IMG_H; k = k + 1) row_ram[k] = 10'd0;
        for (k = 0; k < IMG_W; k = k + 1) col_ram[k] = 9'd0;
    end

    always @(posedge clk) begin
        if (row_we) row_ram[row_waddr] <= row_wdata;
        row_q <= row_ram[row_raddr];
    end

    // column read-modify-write: read at input stage, write at stage 1
    wire [7:0] col_cnt  = (col_q[8] == par) ? col_q[7:0] : 8'd0;
    wire [7:0] col_next = (col_cnt == 8'hFF) ? 8'hFF : col_cnt + 8'd1;
    always @(posedge clk) begin
        col_q <= col_ram[col_raddr];
        if (fg) col_ram[p_x] <= {par, col_next};
    end

    // row counting: write the finished row when the row index changes
    reg flush_row;
    always @(posedge clk) begin
        if (rst) begin
            rc <= 9'd0; ry <= 8'd0; row_open <= 1'b0;
            row_we <= 1'b0; row_waddr <= 8'd0; row_wdata <= 10'd0;
        end else begin
            row_we <= 1'b0;
            if (frame_start) begin
                row_open <= 1'b0;
            end else if (p_v) begin
                if (!row_open || p_y != ry) begin
                    if (row_open) begin
                        row_we    <= 1'b1;
                        row_waddr <= ry;
                        row_wdata <= {par, rc};
                    end
                    ry       <= p_y;
                    rc       <= {8'd0, p_b};
                    row_open <= 1'b1;
                end else begin
                    rc <= rc + p_b;
                end
            end else if (flush_row && row_open) begin
                row_we    <= 1'b1;
                row_waddr <= ry;
                row_wdata <= {par, rc};
                row_open  <= 1'b0;
            end
        end
    end

    // ==================================================================
    // B) analysis sequencer (runs in vertical blanking)
    // ==================================================================
    // shared multiplier (registered)
    reg  [34:0] mul_a, mul_b;
    reg  [69:0] mul_p;
    always @(posedge clk) mul_p <= mul_a * mul_b;

    // shared divider
    reg         dv_start;
    reg  [63:0] dv_num;
    reg  [49:0] dv_den;
    wire [63:0] dv_quo;
    wire        dv_busy, dv_done;
    seq_divider #(.NW(64), .DW(50)) u_div (
        .clk(clk), .rst(rst), .start(dv_start), .num(dv_num), .den(dv_den),
        .quo(dv_quo), .busy(dv_busy), .done(dv_done)
    );

    localparam S_IDLE   = 5'd0,  S_FLUSH  = 5'd1,  S_LATCH  = 5'd2,  S_CHECK  = 5'd3,
               S_PADDR  = 5'd4,  S_PWAIT  = 5'd5,  S_PDATA  = 5'd6,  S_PDIV   = 5'd7,
               S_PDIVW  = 5'd8,  S_MUL    = 5'd9,  S_MULW   = 5'd10, S_MULR   = 5'd11,
               S_MDIV   = 5'd12, S_MDIVW  = 5'd13, S_DECIDE = 5'd14, S_SCORE  = 5'd15,
               S_SCEND  = 5'd16, S_RESULT = 5'd17;

    reg [4:0]  st;
    reg [3:0]  fcnt;                    // flush delay counter

    // latched measurements
    reg [16:0] A;
    reg [25:0] Sx, Sy;
    reg [33:0] Sxx, Syy, Sxy;
    reg [8:0]  xmin, xmax, wbox;
    reg [7:0]  ymin, ymax, hbox;

    // profile sampling
    reg        pcol;                    // 0 = rows, 1 = columns
    reg [2:0]  pk;                      // sample 1..7
    reg [1:0]  pj;                      // -1, 0, +1 neighbour
    reg [10:0] s3;                      // sum of 3 adjacent profile entries
    reg [7:0]  fv [0:13];               // normalised profile features

    wire [11:0] row_ctr = ymin + ((hbox * pk) >> 3);
    wire [11:0] col_ctr = xmin + ((wbox * pk) >> 3);

    // moments
    reg [3:0]  mstep;
    reg [1:0]  dstep;
    reg [69:0] t1;
    reg [51:0] c20, c02;
    reg signed [52:0] c11;
    wire       [52:0] c11_abs = c11[52] ? (~c11 + 53'd1) : c11;
    reg [49:0] a3;
    reg [19:0] n20, n02, n11;
    reg [41:0] p20x02;
    reg [41:0] n11sq;
    reg signed [42:0] i1s;
    reg [40:0] I1;
    reg [43:0] num_e, den_e;

    // template scoring
    reg [3:0]  ti;                      // template index
    reg [3:0]  fi;                      // feature index
    reg [31:0] acc_sc;
    reg [31:0] best_sc;
    reg [3:0]  best_c;
    reg [3:0]  cls;
    reg [3:0]  last_raw;

    wire [7:0]  tf     = tmpl_feat(ti, fi);
    wire [7:0]  fcur   = fv[fi];
    wire [7:0]  adiff  = (fcur > tf) ? (fcur - tf) : (tf - fcur);
    wire [40:0] ti1    = tmpl_i1(ti);
    wire [40:0] i1diff = (I1 > ti1) ? (I1 - ti1) : (ti1 - I1);
    wire [31:0] tot_sc = acc_sc + i1diff[40:10];
    wire        elong  = ({num_e, 4'b0} + {num_e, 2'b0}) > {4'b0, den_e};   // num*20 > den

    // clamp a 64-bit quotient to 20 bits
    function [19:0] clamp20;
        input [63:0] q;
        begin
            clamp20 = (q[63:20] != 0) ? 20'hFFFFF : q[19:0];
        end
    endfunction

    integer m;
    always @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE; fsm_rd <= 1'b0; flush_row <= 1'b0; fcnt <= 4'd0;
            res_strobe <= 1'b0; res_class <= C_NONE; res_raw <= C_NONE; res_obj <= 1'b0;
            res_xmin <= 9'd0; res_xmax <= 9'd0; res_ymin <= 8'd0; res_ymax <= 8'd0; res_area <= 17'd0;
            dv_start <= 1'b0; dv_num <= 64'd0; dv_den <= 50'd1;
            mul_a <= 35'd0; mul_b <= 35'd0;
            A <= 17'd0; Sx <= 26'd0; Sy <= 26'd0; Sxx <= 34'd0; Syy <= 34'd0; Sxy <= 34'd0;
            xmin <= 9'd0; xmax <= 9'd0; ymin <= 8'd0; ymax <= 8'd0; wbox <= 9'd0; hbox <= 8'd0;
            pcol <= 1'b0; pk <= 3'd1; pj <= 2'd0; s3 <= 11'd0;
            row_raddr <= 8'd0; col_raddr_fsm <= 9'd0;
            mstep <= 4'd0; dstep <= 2'd0; t1 <= 70'd0;
            c20 <= 52'd0; c02 <= 52'd0; c11 <= 53'sd0; a3 <= 50'd0;
            n20 <= 20'd0; n02 <= 20'd0; n11 <= 20'd0;
            p20x02 <= 42'd0; n11sq <= 42'd0; i1s <= 43'sd0; I1 <= 41'd0;
            num_e <= 44'd0; den_e <= 44'd0;
            ti <= 4'd0; fi <= 4'd0; acc_sc <= 32'd0; best_sc <= 32'hFFFFFFFF; best_c <= C_UNKNOWN;
            cls <= C_NONE; last_raw <= C_NONE;
            for (m = 0; m < 14; m = m + 1) fv[m] <= 8'd0;
        end else begin
            res_strobe <= 1'b0;
            dv_start   <= 1'b0;
            flush_row  <= 1'b0;
            case (st)
            // ----------------------------------------------------------
            S_IDLE: if (frame_done) begin
                flush_row <= 1'b1;          // write the last row
                fcnt      <= 4'd0;
                st        <= S_FLUSH;
            end
            S_FLUSH: begin                  // let the last RAM writes settle
                fcnt <= fcnt + 4'd1;
                if (fcnt == 4'd3) st <= S_LATCH;
            end
            // ----------------------------------------------------------
            S_LATCH: begin
                fsm_rd <= 1'b1;
                A   <= acc_a;   Sx  <= acc_sx;  Sy  <= acc_sy;
                Sxx <= acc_sxx; Syy <= acc_syy; Sxy <= acc_sxy;
                xmin <= acc_xmin; xmax <= acc_xmax; ymin <= acc_ymin; ymax <= acc_ymax;
                wbox <= acc_xmax - acc_xmin + 9'd1;
                hbox <= acc_ymax - acc_ymin + 8'd1;
                st  <= S_CHECK;
            end
            S_CHECK: begin
                if (A < MIN_AREA || wbox < MIN_DIM || hbox < MIN_DIM) begin
                    cls <= C_NONE;
                    st  <= S_RESULT;
                end else begin
                    pcol <= 1'b0; pk <= 3'd1; pj <= 2'd0; s3 <= 11'd0;
                    st   <= S_PADDR;
                end
            end
            // ----------------------------------------------------------
            // profile samples: sum of 3 adjacent rows/columns around
            // the 1/8 .. 7/8 positions
            S_PADDR: begin
                if (!pcol) row_raddr     <= row_ctr[7:0] + {6'd0, pj} - 8'd1;
                else       col_raddr_fsm <= col_ctr[8:0] + {7'd0, pj} - 9'd1;
                st <= S_PWAIT;
            end
            S_PWAIT: st <= S_PDATA;         // RAM read latency
            S_PDATA: begin
                if (!pcol) s3 <= s3 + ((row_q[9] == par) ? {2'b0, row_q[8:0]} : 11'd0);
                else       s3 <= s3 + ((col_q[8] == par) ? {3'b0, col_q[7:0]} : 11'd0);
                if (pj == 2'd2) st <= S_PDIV;
                else begin pj <= pj + 2'd1; st <= S_PADDR; end
            end
            S_PDIV: begin                   // feature = s3 * 85 / dimension (255 = full)
                dv_num   <= s3 * 64'd85;
                dv_den   <= pcol ? {42'd0, hbox} : {41'd0, wbox};
                dv_start <= 1'b1;
                st       <= S_PDIVW;
            end
            S_PDIVW: if (dv_done) begin
                fv[{1'b0, pk} - 4'd1 + (pcol ? 4'd7 : 4'd0)] <=
                    (dv_quo > 64'd255) ? 8'd255 : dv_quo[7:0];
                s3 <= 11'd0;
                pj <= 2'd0;
                if (pk == 3'd7) begin
                    pk <= 3'd1;
                    if (pcol) begin
                        mstep <= 4'd0;
                        st    <= S_MUL;
                    end else begin
                        pcol <= 1'b1;
                        st   <= S_PADDR;
                    end
                end else begin
                    pk <= pk + 3'd1;
                    st <= S_PADDR;
                end
            end
            // ----------------------------------------------------------
            // central moments (scaled by A) and invariants
            S_MUL: begin
                case (mstep)
                    4'd0:  begin mul_a <= A;          mul_b <= Sxx;       end
                    4'd1:  begin mul_a <= Sx;         mul_b <= Sx;        end
                    4'd2:  begin mul_a <= A;          mul_b <= Syy;       end
                    4'd3:  begin mul_a <= Sy;         mul_b <= Sy;        end
                    4'd4:  begin mul_a <= A;          mul_b <= Sxy;       end
                    4'd5:  begin mul_a <= Sx;         mul_b <= Sy;        end
                    4'd6:  begin mul_a <= A;          mul_b <= A;         end
                    4'd7:  begin mul_a <= t1[34:0];   mul_b <= A;         end
                    4'd8:  begin mul_a <= n20;        mul_b <= n02;       end
                    4'd9:  begin mul_a <= n11;        mul_b <= n11;       end
                    4'd10: begin
                        mul_a <= (n20 > n02) ? (n20 - n02) : (n02 - n20);
                        mul_b <= (n20 > n02) ? (n20 - n02) : (n02 - n20);
                    end
                    default: begin mul_a <= n20 + n02; mul_b <= n20 + n02; end
                endcase
                st <= S_MULW;
            end
            S_MULW: st <= S_MULR;           // product registered
            S_MULR: begin
                case (mstep)
                    4'd0:  t1  <= mul_p;
                    4'd1:  c20 <= t1[51:0] - mul_p[51:0];
                    4'd2:  t1  <= mul_p;
                    4'd3:  c02 <= t1[51:0] - mul_p[51:0];
                    4'd4:  t1  <= mul_p;
                    4'd5:  c11 <= $signed({1'b0, t1[51:0]}) - $signed({1'b0, mul_p[51:0]});
                    4'd6:  t1  <= mul_p;
                    4'd7:  a3  <= mul_p[49:0];
                    4'd8:  p20x02 <= mul_p[41:0];
                    4'd9:  begin
                        n11sq <= mul_p[41:0];
                        i1s   <= $signed({1'b0, p20x02}) - $signed({1'b0, mul_p[41:0]});
                    end
                    4'd10: num_e <= {2'b0, mul_p[41:0]} + {n11sq, 2'b00};
                    default: den_e <= mul_p[43:0];
                endcase
                if (mstep == 4'd7) begin
                    dstep <= 2'd0;
                    st    <= S_MDIV;
                end else if (mstep == 4'd11) begin
                    st <= S_DECIDE;
                end else begin
                    st <= S_MUL;
                end
                mstep <= mstep + 4'd1;
            end
            // normalised second moments n = C << 16 / A^3  (Q16)
            S_MDIV: begin
                case (dstep)
                    2'd0:    dv_num <= {c20[47:0], 16'd0};
                    2'd1:    dv_num <= {c02[47:0], 16'd0};
                    default: dv_num <= {c11_abs[47:0], 16'd0};
                endcase
                dv_den   <= a3;
                dv_start <= 1'b1;
                st       <= S_MDIVW;
            end
            S_MDIVW: if (dv_done) begin
                case (dstep)
                    2'd0:    n20 <= clamp20(dv_quo);
                    2'd1:    n02 <= clamp20(dv_quo);
                    default: n11 <= clamp20(dv_quo);
                endcase
                if (dstep == 2'd2) st <= S_MUL;     // mstep is already 8
                else begin dstep <= dstep + 2'd1; st <= S_MDIV; end
            end
            // ----------------------------------------------------------
            S_DECIDE: begin
                if (i1s[42])            I1 <= 41'd0;
                else                    I1 <= i1s[40:0];
                if (!i1s[42] && i1s[40:0] > REJ_TH) begin
                    cls <= C_UNKNOWN; st <= S_RESULT;
                end else if (!i1s[42] && i1s[40:0] > TRI_TH) begin
                    cls <= C_TRI; st <= S_RESULT;
                end else if (!i1s[42] && i1s[40:0] > QUAD_TH) begin
                    cls <= elong ? C_RECT : C_SQUARE; st <= S_RESULT;
                end else if (elong) begin
                    cls <= C_ELLIPSE; st <= S_RESULT;
                end else begin
                    ti <= 4'd0; fi <= 4'd0; acc_sc <= 32'd0;
                    best_sc <= 32'hFFFFFFFF; best_c <= C_UNKNOWN;
                    st <= S_SCORE;
                end
            end
            // nearest round-family template
            S_SCORE: begin
                acc_sc <= acc_sc + adiff;
                if (fi == 4'd13) st <= S_SCEND;
                else             fi <= fi + 4'd1;
            end
            S_SCEND: begin
                if (tot_sc < best_sc) begin
                    best_sc <= tot_sc;
                    best_c  <= tmpl_class(ti);
                end
                if (ti == N_TMPL - 1) begin
                    st <= S_RESULT;
                    cls <= (tot_sc < best_sc) ?
                               ((tot_sc <= REJECT_SCORE) ? tmpl_class(ti) : C_UNKNOWN) :
                               ((best_sc <= REJECT_SCORE) ? best_c        : C_UNKNOWN);
                end else begin
                    ti <= ti + 4'd1;
                    fi <= 4'd0;
                    acc_sc <= 32'd0;
                    st <= S_SCORE;
                end
            end
            // ----------------------------------------------------------
            S_RESULT: begin
                fsm_rd     <= 1'b0;
                res_raw    <= cls;
                res_obj    <= (cls != C_NONE);
                res_xmin   <= xmin; res_xmax <= xmax;
                res_ymin   <= ymin; res_ymax <= ymax;
                res_area   <= A;
                last_raw   <= cls;
                if (cls == last_raw) res_class <= cls;    // 2-frame agreement
                res_strobe <= 1'b1;
                st         <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
