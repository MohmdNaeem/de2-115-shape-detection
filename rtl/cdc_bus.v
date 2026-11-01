// ============================================================================
// cdc_bus.v
// ----------------------------------------------------------------------------
// Safe transfer of a multi-bit value from one clock domain to another using a
// toggle handshake:
//   source : on `src_strobe` the bus is captured into a holding register and
//            a toggle bit flips.
//   dest   : the toggle is synchronised (3 flip-flops); when it changes, the
//            holding register - stable for a long time by then - is copied.
// Suitable for slowly updated data such as per-frame results.
// ============================================================================
module cdc_bus #(
    parameter W = 32
)(
    input  wire         src_clk,
    input  wire         src_rst,
    input  wire         src_strobe,
    input  wire [W-1:0] src_data,
    input  wire         dst_clk,
    input  wire         dst_rst,
    output reg  [W-1:0] dst_data,
    output reg          dst_strobe
);
    reg [W-1:0] hold;
    reg         tog;
    always @(posedge src_clk) begin
        if (src_rst) begin
            hold <= {W{1'b0}};
            tog  <= 1'b0;
        end else if (src_strobe) begin
            hold <= src_data;
            tog  <= ~tog;
        end
    end

    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
    reg [2:0] tog_s;
    always @(posedge dst_clk) begin
        if (dst_rst) begin
            tog_s      <= 3'b000;
            dst_data   <= {W{1'b0}};
            dst_strobe <= 1'b0;
        end else begin
            tog_s      <= {tog_s[1:0], tog};
            dst_strobe <= tog_s[2] ^ tog_s[1];
            if (tog_s[2] ^ tog_s[1])
                dst_data <= hold;
        end
    end
endmodule

// ----------------------------------------------------------------------------
// Two-flip-flop synchroniser for single, slowly changing control bits.
// ----------------------------------------------------------------------------
module sync_bits #(
    parameter W = 1
)(
    input  wire         clk,
    input  wire [W-1:0] d,
    output wire [W-1:0] q
);
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
    reg [W-1:0] s1, s2;
    always @(posedge clk) begin
        s1 <= d;
        s2 <= s1;
    end
    assign q = s2;
endmodule

// ----------------------------------------------------------------------------
// Reset synchroniser: asynchronous assert, synchronous de-assert.
// ----------------------------------------------------------------------------
module reset_sync (
    input  wire clk,
    input  wire arst,          // active-high asynchronous reset
    output wire rst            // active-high reset, synchronous release
);
    reg [2:0] r;
    always @(posedge clk or posedge arst) begin
        if (arst) r <= 3'b111;
        else      r <= {r[1:0], 1'b0};
    end
    assign rst = r[2];
endmodule
