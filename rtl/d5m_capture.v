// ============================================================================
// d5m_capture.v
// ----------------------------------------------------------------------------
// Captures the raw 12-bit Bayer stream of the TRDB-D5M on its pixel clock.
//
//   FVAL = frame valid, LVAL = line valid, D[11:0] = pixel data.
//
// Outputs every valid pixel with its (x, y) position inside the frame, and
// single-cycle frame_start / frame_end pulses.  Capture begins at the first
// complete frame after reset so that a partial frame is never processed.
// All inputs are registered first (these registers can be packed into the
// I/O cells by Quartus).
// ============================================================================
module d5m_capture (
    input  wire        pclk,
    input  wire        rst,
    input  wire [11:0] d5m_d,
    input  wire        d5m_fval,
    input  wire        d5m_lval,
    output reg  [11:0] pix,
    output reg         pix_valid,
    output reg  [11:0] x,
    output reg  [11:0] y,
    output reg         frame_start,
    output reg         frame_end,
    output reg  [15:0] frame_count
);
    reg [11:0] d_r;
    reg        fv_r, lv_r, fv_d, lv_d;
    reg        active;
    reg [11:0] xcnt, ycnt;

    always @(posedge pclk) begin
        if (rst) begin
            d_r <= 12'd0; fv_r <= 1'b0; lv_r <= 1'b0; fv_d <= 1'b0; lv_d <= 1'b0;
            active <= 1'b0; xcnt <= 12'd0; ycnt <= 12'd0;
            pix <= 12'd0; pix_valid <= 1'b0; x <= 12'd0; y <= 12'd0;
            frame_start <= 1'b0; frame_end <= 1'b0; frame_count <= 16'd0;
        end else begin
            // stage 0: input registers
            d_r  <= d5m_d;
            fv_r <= d5m_fval;
            lv_r <= d5m_lval;
            fv_d <= fv_r;
            lv_d <= lv_r;

            // frame boundaries
            frame_start <= fv_r & ~fv_d;
            frame_end   <= ~fv_r & fv_d & active;
            if (fv_r & ~fv_d) begin
                active <= 1'b1;
                ycnt   <= 12'd0;
            end
            if (~fv_r & fv_d & active)
                frame_count <= frame_count + 16'd1;

            // line / pixel counters
            if (fv_r & lv_d & ~lv_r)          // end of a line
                ycnt <= ycnt + 12'd1;
            if (fv_r & lv_r)
                xcnt <= xcnt + 12'd1;
            else
                xcnt <= 12'd0;

            // stage 1: output
            pix       <= d_r;
            pix_valid <= fv_r & lv_r & active & ~(fv_r & ~fv_d);
            x         <= xcnt;
            y         <= ycnt;
        end
    end
endmodule
