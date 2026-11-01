// ============================================================================
// d5m_i2c_config.v
// ----------------------------------------------------------------------------
// Configures the Micron/Aptina MT9P001 sensor on the Terasic TRDB-D5M through
// its two-wire serial interface (I2C compatible, write address 0xBA).
//
// Each register write is: START, 0xBA, REG, DATA[15:8], DATA[7:0], STOP.
//
// Sensor set-up used by this project
//   * 1280 x 960 window centred on the 2592 x 1944 array
//   * 2x binning + 2x skipping in rows and columns  -> 640 x 480 Bayer output
//   * PLL bypassed: PIXCLK = XCLKIN (25 MHz from the FPGA)
//   * Exposure (shutter width, in row times) comes from the `exposure` input
//     and is re-written automatically whenever that input changes.
//
// SDA is open-drain (driven low or released); SCL is driven push-pull, as in
// Terasic's reference design (the MT9P001 never stretches the clock).
// ============================================================================
module d5m_i2c_config #(
    parameter CLK_HZ   = 50_000_000,
    parameter I2C_HZ   = 50_000,           // SCL frequency
    parameter WAIT_US  = 2_000,            // delay after reset before first write
    parameter [7:0] DEV_ADDR = 8'hBA,
    parameter [15:0] READ_MODE2 = 16'h0000 // 0x20: bit15 mirror rows, bit14 mirror columns
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] exposure,           // shutter width lower (reg 0x09)
    output reg         sclk,
    output reg         sda_oe,             // 1 = drive SDA low, 0 = release (pulled high)
    input  wire        sda_in,
    output reg         config_done,
    output reg         ack_error           // sticky: a write was not acknowledged
);
    // ------------------------------------------------------------------
    // register table {reg[7:0], data[15:0]}
    // ------------------------------------------------------------------
    localparam N_REGS  = 17;
    localparam EXP_IDX = 10;

    function [23:0] lut;
        input [4:0] i;
        begin
            case (i)
                5'd0 : lut = 24'h0D_0001;            // soft reset (assert)
                5'd1 : lut = 24'h0D_0000;            // soft reset (release)
                5'd2 : lut = 24'h01_0220;            // row start    = 544  (multiple of 4 for 2x bin)
                5'd3 : lut = 24'h02_02A0;            // column start = 672  (multiple of 4 for 2x bin)
                5'd4 : lut = 24'h03_03BF;            // row size     = 959  -> 960 rows
                5'd5 : lut = 24'h04_04FF;            // column size  = 1279 -> 1280 columns
                5'd6 : lut = 24'h22_0011;            // row mode: bin 2x, skip 2x  -> 480 rows
                5'd7 : lut = 24'h23_0011;            // col mode: bin 2x, skip 2x  -> 640 columns
                5'd8 : lut = 24'h05_0000;            // horizontal blank (sensor enforces minimum)
                5'd9 : lut = 24'h06_0019;            // vertical blank = 25 rows
                5'd10: lut = {8'h09, exposure};      // shutter width lower (exposure)
                5'd11: lut = 24'h0A_8000;            // invert PIXCLK: data stable on rising edge
                5'd12: lut = {8'h20, READ_MODE2};    // read mode 2 (mirroring)
                5'd13: lut = 24'h2B_0010;            // green1 gain (2x)
                5'd14: lut = 24'h2C_0012;            // blue gain
                5'd15: lut = 24'h2D_0012;            // red gain
                5'd16: lut = 24'h2E_0010;            // green2 gain (2x)
                default: lut = 24'h0D_0000;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // quarter-bit tick generator
    // ------------------------------------------------------------------
    localparam QDIV = (CLK_HZ / (I2C_HZ * 4)) < 2 ? 2 : (CLK_HZ / (I2C_HZ * 4));
    reg [15:0] qcnt;
    wire       qtick = (qcnt == QDIV - 1);
    always @(posedge clk) begin
        if (rst || qtick) qcnt <= 16'd0;
        else              qcnt <= qcnt + 16'd1;
    end

    // ------------------------------------------------------------------
    // byte engine + sequencer
    // ------------------------------------------------------------------
    localparam ST_WAIT  = 3'd0,   // power-up / inter-write wait
               ST_LOAD  = 3'd1,   // load next table entry
               ST_START = 3'd2,   // START condition
               ST_BITS  = 3'd3,   // 4 bytes x (8 data + 1 ack)
               ST_STOP  = 3'd4,   // STOP condition
               ST_NEXT  = 3'd5,   // advance table index
               ST_IDLE  = 3'd6;   // configured; watch exposure

    localparam WAIT_CYC = (CLK_HZ / 1_000_000) * WAIT_US;

    reg  [2:0]  state;
    reg  [1:0]  phase;            // quarter of the current bit / condition
    reg  [31:0] sh;               // {dev, reg, data_hi, data_lo}
    reg  [5:0]  bitn;             // 0..35
    reg  [3:0]  bpos;             // 0..8 within a byte (8 = ACK slot)
    reg  [4:0]  idx;
    reg         single;           // write only one entry (exposure update)
    reg  [31:0] wcnt;
    reg  [15:0] exp_written;

    always @(posedge clk) begin
        if (rst) begin
            state       <= ST_WAIT;
            phase       <= 2'd0;
            sclk        <= 1'b1;
            sda_oe      <= 1'b0;
            sh          <= 32'd0;
            bitn        <= 6'd0;
            bpos        <= 4'd0;
            idx         <= 5'd0;
            single      <= 1'b0;
            wcnt        <= 32'd0;
            config_done <= 1'b0;
            ack_error   <= 1'b0;
            exp_written <= 16'd0;
        end else begin
            case (state)
            // ----------------------------------------------------------
            ST_WAIT: begin
                sclk   <= 1'b1;
                sda_oe <= 1'b0;
                if (wcnt >= WAIT_CYC) begin
                    wcnt  <= 32'd0;
                    state <= ST_LOAD;
                end else
                    wcnt <= wcnt + 32'd1;
            end
            // ----------------------------------------------------------
            ST_LOAD: begin
                sh    <= {DEV_ADDR, lut(idx)};
                if (idx == EXP_IDX) exp_written <= exposure;
                bitn  <= 6'd0;
                bpos  <= 4'd0;
                phase <= 2'd0;
                state <= ST_START;
            end
            // ----------------------------------------------------------
            // START: SDA falls while SCL is high, then SCL goes low
            ST_START: if (qtick) begin
                phase <= phase + 2'd1;
                case (phase)
                    2'd0: begin sclk <= 1'b1; sda_oe <= 1'b0; end
                    2'd1: begin sclk <= 1'b1; sda_oe <= 1'b1; end
                    2'd2: begin sclk <= 1'b0; sda_oe <= 1'b1; end
                    2'd3: begin sclk <= 1'b0; state <= ST_BITS; end
                endcase
            end
            // ----------------------------------------------------------
            // data bits: q0 set SDA (SCL low), q1/q2 SCL high, q3 SCL low
            ST_BITS: if (qtick) begin
                phase <= phase + 2'd1;
                case (phase)
                    2'd0: begin
                        sclk <= 1'b0;
                        if (bpos == 4'd8) sda_oe <= 1'b0;          // release for ACK
                        else              sda_oe <= ~sh[31];       // MSB first
                    end
                    2'd1: sclk <= 1'b1;
                    2'd2: begin
                        sclk <= 1'b1;
                        if (bpos == 4'd8 && sda_in) ack_error <= 1'b1;
                    end
                    2'd3: begin
                        sclk <= 1'b0;
                        if (bpos != 4'd8) sh <= {sh[30:0], 1'b0};
                        bpos <= (bpos == 4'd8) ? 4'd0 : bpos + 4'd1;
                        bitn <= bitn + 6'd1;
                        if (bitn == 6'd35) state <= ST_STOP;
                    end
                endcase
            end
            // ----------------------------------------------------------
            // STOP: SDA rises while SCL is high
            ST_STOP: if (qtick) begin
                phase <= phase + 2'd1;
                case (phase)
                    2'd0: begin sclk <= 1'b0; sda_oe <= 1'b1; end
                    2'd1: begin sclk <= 1'b1; sda_oe <= 1'b1; end
                    2'd2: begin sclk <= 1'b1; sda_oe <= 1'b0; end
                    2'd3: state <= ST_NEXT;
                endcase
            end
            // ----------------------------------------------------------
            ST_NEXT: begin
                if (single || idx == N_REGS - 1) begin
                    single      <= 1'b0;
                    config_done <= 1'b1;
                    state       <= ST_IDLE;
                end else begin
                    idx   <= idx + 5'd1;
                    state <= ST_LOAD;
                end
            end
            // ----------------------------------------------------------
            ST_IDLE: begin
                sclk   <= 1'b1;
                sda_oe <= 1'b0;
                if (exposure != exp_written) begin   // exposure switches moved
                    idx    <= EXP_IDX;
                    single <= 1'b1;
                    state  <= ST_LOAD;
                end
            end
            default: state <= ST_WAIT;
            endcase
        end
    end
endmodule
