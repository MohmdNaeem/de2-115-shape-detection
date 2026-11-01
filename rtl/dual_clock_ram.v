// ============================================================================
// dual_clock_ram.v
// ----------------------------------------------------------------------------
// Simple dual-port RAM with independent write and read clocks, written in the
// form Quartus recognises for inference into M9K blocks.  Used as the video
// frame buffer: written by the camera pixel clock, read by the VGA clock.
// Read latency: 1 clock (registered output).
// ============================================================================
module dual_clock_ram #(
    parameter DW    = 16,
    parameter AW    = 17,
    parameter DEPTH = 76800
)(
    input  wire          wclk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire          rclk,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);
    (* ramstyle = "M9K" *) reg [DW-1:0] mem [0:DEPTH-1];

    always @(posedge wclk) begin
        if (we) mem[waddr] <= wdata;
    end

    always @(posedge rclk) begin
        rdata <= mem[raddr];
    end
endmodule
