// ============================================================================
// ultrasonic_hcsr04.v
// ----------------------------------------------------------------------------
// HC-SR04 (or compatible) ultrasonic range finder driver + distance gate.
//
//   1. TRIG high for 10 us
//   2. wait for ECHO to rise (timeout -> no echo)
//   3. measure ECHO high time; the distance counter advances by 1 cm every
//      CYC_PER_CM clocks:  1 cm (round trip) = 2 / 34300 m/s = 58.3 us
//      -> 2915 clocks at 50 MHz.  No divider needed.  The prescaler starts
//      at half a centimetre so the result is rounded to the nearest cm.
//   4. wait for the rest of the measurement period (>= 60 ms recommended so
//      old echoes die out)
//
// `near` = object within NEAR_CM (75 cm).  It only changes after
// CONFIRM consecutive measurements agree, which rejects single spurious
// echoes.  A missing echo or an out-of-range echo counts as "far".
//
// ECHO is a 5 V signal on the classic HC-SR04: use a resistor divider
// (e.g. 1 k / 2 k) or a level shifter before the 3.3 V FPGA pin.
// ============================================================================
module ultrasonic_hcsr04 #(
    parameter CLK_HZ     = 50_000_000,
    parameter CYC_PER_CM = 2915,         // clocks per cm of distance
    parameter PERIOD_MS  = 60,           // measurement period
    parameter TIMEOUT_MS = 30,           // max echo wait / echo length
    parameter NEAR_CM    = 75,           // gate threshold (inclusive)
    parameter MIN_CM     = 2,            // below this the reading is invalid
    parameter MAX_CM     = 400,          // sensor range
    parameter CONFIRM    = 2             // consecutive agreeing readings
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       echo,              // asynchronous input from sensor
    output reg        trig,
    output reg  [9:0] dist_cm,           // last valid distance (cm)
    output reg        dist_valid,        // last measurement produced an echo in range
    output reg        near,              // debounced: distance <= NEAR_CM
    output reg        sample_strobe      // pulses after each measurement
);
    localparam TRIG_CYC    = CLK_HZ / 100_000;              // 10 us
    localparam PERIOD_CYC  = (CLK_HZ / 1000) * PERIOD_MS;
    localparam TIMEOUT_CYC = (CLK_HZ / 1000) * TIMEOUT_MS;

    // synchronise ECHO
    reg [2:0] echo_s;
    always @(posedge clk) echo_s <= {echo_s[1:0], echo};
    wire echo_i = echo_s[2];

    localparam S_TRIG = 2'd0, S_WAITR = 2'd1, S_MEAS = 2'd2, S_IDLE = 2'd3;
    reg [1:0]  st;
    reg [31:0] tcnt;                     // time since trigger
    reg [15:0] pcnt;                     // prescaler (clocks per cm)
    reg [9:0]  cm;                       // cm counter while echo high
    reg [3:0]  agree;                    // consecutive readings disagreeing with `near`

    // result of the measurement that just finished (evaluated one cycle later)
    reg        m_done, m_valid;
    reg [9:0]  m_dist;
    wire       n_raw = m_valid && (m_dist <= NEAR_CM);

    always @(posedge clk) begin
        if (rst) begin
            st <= S_TRIG; tcnt <= 32'd0; pcnt <= 16'd0; cm <= 10'd0;
            trig <= 1'b0; dist_cm <= 10'd0; dist_valid <= 1'b0; near <= 1'b0;
            sample_strobe <= 1'b0; agree <= 4'd0;
            m_done <= 1'b0; m_valid <= 1'b0; m_dist <= 10'd0;
        end else begin
            sample_strobe <= 1'b0;
            m_done        <= 1'b0;
            tcnt <= tcnt + 32'd1;

            // evaluate the finished measurement: update distance and the
            // debounced near flag (changes after CONFIRM agreeing readings)
            if (m_done) begin
                dist_valid    <= m_valid;
                if (m_valid) dist_cm <= m_dist;
                sample_strobe <= 1'b1;
                if (n_raw == near) agree <= 4'd0;
                else if (agree + 4'd1 >= CONFIRM) begin
                    near  <= n_raw;
                    agree <= 4'd0;
                end else
                    agree <= agree + 4'd1;
            end

            case (st)
            S_TRIG: begin
                trig <= 1'b1;
                if (tcnt >= TRIG_CYC) begin
                    trig <= 1'b0;
                    st   <= S_WAITR;
                end
            end
            S_WAITR: begin
                if (echo_i) begin
                    pcnt <= CYC_PER_CM / 2;      // start half-way: rounds to nearest cm
                    cm   <= 10'd0;
                    st   <= S_MEAS;
                end else if (tcnt >= TIMEOUT_CYC) begin
                    m_done <= 1'b1; m_valid <= 1'b0;          // no echo
                    st <= S_IDLE;
                end
            end
            S_MEAS: begin
                if (!echo_i) begin               // falling edge: done
                    m_done  <= 1'b1;
                    m_valid <= (cm >= MIN_CM) && (cm <= MAX_CM);
                    m_dist  <= cm;
                    st <= S_IDLE;
                end else if (tcnt >= TIMEOUT_CYC + TRIG_CYC) begin
                    m_done <= 1'b1; m_valid <= 1'b0;          // echo too long: no target
                    st <= S_IDLE;
                end else if (pcnt == CYC_PER_CM - 1) begin
                    pcnt <= 16'd0;
                    if (cm != 10'h3FF) cm <= cm + 10'd1;
                end else
                    pcnt <= pcnt + 16'd1;
            end
            S_IDLE: begin
                if (tcnt >= PERIOD_CYC) begin
                    tcnt <= 32'd0;
                    st   <= S_TRIG;
                end
            end
            endcase
        end
    end
endmodule
