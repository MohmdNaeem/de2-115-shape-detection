// ============================================================================
// pixel_segment.v
// ----------------------------------------------------------------------------
// Turns the RGB stream into a binary "object" mask with an automatically
// adapting threshold.
//
// 1. Feature value per pixel (selected by `mode`):
//      0 : darkness   = 255 - Y   (dark shape on a light background)
//      1 : brightness = Y         (light shape on a dark background)
//      2 : chroma     = max(R,G,B) - min(R,G,B)
//                       (coloured shape on a white/grey/black background;
//                        insensitive to illumination level)
//      3 : same as 0
//    Y = (77 R + 150 G + 29 B) / 256  (ITU-R BT.601 luma)
//
// 2. Threshold: iterative ISODATA (Ridler-Calvard), one iteration per frame:
//      T(n+1) = ( mean(f | f > T(n)) + mean(f | f <= T(n)) ) / 2
//    This tracks lighting changes automatically - the main weakness of a
//    fixed luminance threshold.  If the two class means are closer than
//    MIN_CONTRAST the scene is treated as empty (no object pixels).
//    `manual_en` replaces the automatic threshold with `manual_th`.
//
// 3. mask = (f > T) AND gate, where `gate` (distance <= 75 cm) is sampled at
//    the start of each frame so a frame is processed entirely or not at all.
// ============================================================================
module pixel_segment #(
    parameter [7:0] T_INIT       = 8'd128,
    parameter [7:0] T_MIN        = 8'd16,
    parameter [7:0] T_MAX        = 8'd240,
    parameter [7:0] MIN_CONTRAST = 8'd24
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       in_valid,
    input  wire [8:0] in_x,
    input  wire [7:0] in_y,
    input  wire [7:0] r,
    input  wire [7:0] g,
    input  wire [7:0] b,
    input  wire       frame_start,
    input  wire       frame_done,      // all pixels of the frame have passed
    input  wire [1:0] mode,
    input  wire       manual_en,
    input  wire [7:0] manual_th,
    input  wire       gate,            // 1 = object within range, process frame
    output reg        out_valid,
    output reg        out_bit,
    output reg  [8:0] out_x,
    output reg  [7:0] out_y,
    output reg  [7:0] threshold,       // threshold in use this frame
    output reg        contrast_ok,     // scene has two distinct populations
    output reg        frame_enabled    // this frame is being processed (gate)
);
    // ------------------------------------------------------------------
    // stage 1 : feature value
    // ------------------------------------------------------------------
    wire [15:0] y_mul = 16'd77 * r + 16'd150 * g + 16'd29 * b;
    wire [7:0]  luma  = y_mul[15:8];
    wire [7:0]  mx    = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
    wire [7:0]  mn    = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);

    reg  [7:0] f1;
    reg        v1;
    reg  [8:0] x1;
    reg  [7:0] y1;
    always @(posedge clk) begin
        if (rst) begin
            v1 <= 1'b0; f1 <= 8'd0; x1 <= 9'd0; y1 <= 8'd0;
        end else begin
            v1 <= in_valid;
            x1 <= in_x;
            y1 <= in_y;
            case (mode)
                2'd1:    f1 <= luma;
                2'd2:    f1 <= mx - mn;
                default: f1 <= 8'd255 - luma;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // stage 2 : threshold + frame statistics
    // ------------------------------------------------------------------
    reg [7:0]  t_auto;
    reg        ok_auto;
    reg        gate_l;
    reg [16:0] cnt_hi, cnt_lo;
    reg [24:0] sum_hi, sum_lo;

    wire hi = (f1 > threshold);

    always @(posedge clk) begin
        if (rst) begin
            out_valid <= 1'b0; out_bit <= 1'b0; out_x <= 9'd0; out_y <= 8'd0;
            threshold <= T_INIT; gate_l <= 1'b0; frame_enabled <= 1'b0;
            cnt_hi <= 17'd0; cnt_lo <= 17'd0; sum_hi <= 25'd0; sum_lo <= 25'd0;
        end else begin
            if (frame_start) begin
                threshold     <= manual_en ? manual_th : t_auto;
                gate_l        <= gate & (manual_en | ok_auto);
                frame_enabled <= gate;
                cnt_hi <= 17'd0; cnt_lo <= 17'd0;
                sum_hi <= 25'd0; sum_lo <= 25'd0;
            end else if (v1) begin
                if (hi) begin
                    cnt_hi <= cnt_hi + 17'd1;
                    sum_hi <= sum_hi + f1;
                end else begin
                    cnt_lo <= cnt_lo + 17'd1;
                    sum_lo <= sum_lo + f1;
                end
            end
            out_valid <= v1;
            out_bit   <= v1 & hi & gate_l;
            out_x     <= x1;
            out_y     <= y1;
        end
    end

    // ------------------------------------------------------------------
    // end of frame : compute class means and next threshold
    // ------------------------------------------------------------------
    reg         dstart;
    reg  [24:0] dnum;
    reg  [16:0] dden;
    wire [24:0] dquo;
    wire        dbusy, ddone;

    seq_divider #(.NW(25), .DW(17)) u_div (
        .clk(clk), .rst(rst), .start(dstart), .num(dnum), .den(dden),
        .quo(dquo), .busy(dbusy), .done(ddone)
    );

    localparam S_IDLE = 3'd0, S_HI = 3'd1, S_HIW = 3'd2, S_LO = 3'd3, S_LOW = 3'd4, S_UPD = 3'd5;
    reg [2:0] st;
    reg [7:0] mean_hi, mean_lo;
    wire [8:0] tsum = {1'b0, mean_hi} + {1'b0, mean_lo} + 9'd1;

    always @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE; dstart <= 1'b0; dnum <= 25'd0; dden <= 17'd1;
            mean_hi <= 8'd255; mean_lo <= 8'd0;
            t_auto <= T_INIT; ok_auto <= 1'b1; contrast_ok <= 1'b1;
        end else begin
            dstart <= 1'b0;
            case (st)
                S_IDLE: if (frame_done) st <= S_HI;
                S_HI: begin
                    if (cnt_hi == 17'd0) begin mean_hi <= 8'd255; st <= S_LO; end
                    else begin dnum <= sum_hi; dden <= cnt_hi; dstart <= 1'b1; st <= S_HIW; end
                end
                S_HIW: if (ddone) begin
                    mean_hi <= (dquo > 25'd255) ? 8'd255 : dquo[7:0];
                    st <= S_LO;
                end
                S_LO: begin
                    if (cnt_lo == 17'd0) begin mean_lo <= 8'd0; st <= S_UPD; end
                    else begin dnum <= sum_lo; dden <= cnt_lo; dstart <= 1'b1; st <= S_LOW; end
                end
                S_LOW: if (ddone) begin
                    mean_lo <= (dquo > 25'd255) ? 8'd255 : dquo[7:0];
                    st <= S_UPD;
                end
                S_UPD: begin
                    if (tsum[8:1] < T_MIN)      t_auto <= T_MIN;
                    else if (tsum[8:1] > T_MAX) t_auto <= T_MAX;
                    else                        t_auto <= tsum[8:1];
                    ok_auto     <= ({1'b0, mean_hi} >= {1'b0, mean_lo} + {1'b0, MIN_CONTRAST});
                    contrast_ok <= ({1'b0, mean_hi} >= {1'b0, mean_lo} + {1'b0, MIN_CONTRAST});
                    st <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
