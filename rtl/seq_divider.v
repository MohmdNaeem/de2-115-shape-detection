// ============================================================================
// seq_divider.v
// ----------------------------------------------------------------------------
// Unsigned sequential (restoring) divider: quo = num / den.
// One quotient bit per clock -> NW clock cycles per division.
// Used by the threshold estimator and the shape classifier, both of which run
// during the camera's vertical blanking, so throughput is not critical and a
// sequential divider is far smaller than a combinational one.
//
//   * Pulse `start` for one cycle while `busy` is low.
//   * `done` pulses for one cycle when `quo` is valid.
//   * Division by zero returns all ones (callers guard against it).
// ============================================================================
module seq_divider #(
    parameter NW = 32,          // numerator / quotient width
    parameter DW = 16           // denominator width
)(
    input  wire           clk,
    input  wire           rst,
    input  wire           start,
    input  wire [NW-1:0]  num,
    input  wire [DW-1:0]  den,
    output reg  [NW-1:0]  quo,
    output reg            busy,
    output reg            done
);
    reg [DW-1:0] rem;            // partial remainder (always < den)
    reg [NW-1:0] n;              // numerator shift register
    reg [DW-1:0] d;              // latched denominator
    reg [7:0]    cnt;            // bits remaining (NW <= 255)

    // shift next numerator bit into remainder and trial-subtract
    wire [DW:0]   sh   = {rem, n[NW-1]};
    wire [DW+1:0] diff = {1'b0, sh} - {2'b00, d};

    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0;
            done <= 1'b0;
            quo  <= {NW{1'b0}};
            rem  <= {DW{1'b0}};
            n    <= {NW{1'b0}};
            d    <= {DW{1'b0}};
            cnt  <= 8'd0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                rem  <= {DW{1'b0}};
                n    <= num;
                d    <= den;
                quo  <= {NW{1'b0}};
                cnt  <= NW[7:0];
                busy <= 1'b1;
            end else if (busy) begin
                n <= {n[NW-2:0], 1'b0};
                if (!diff[DW+1]) begin             // sh >= d
                    rem <= diff[DW-1:0];
                    quo <= {quo[NW-2:0], 1'b1};
                end else begin
                    rem <= sh[DW-1:0];
                    quo <= {quo[NW-2:0], 1'b0};
                end
                cnt <= cnt - 8'd1;
                if (cnt == 8'd1) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            end
        end
    end
endmodule
