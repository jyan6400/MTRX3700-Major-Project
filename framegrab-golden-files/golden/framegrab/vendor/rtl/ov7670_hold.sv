/*
 *  ov7670_hold.sv -- hold the camera's automatic exposure, gain and white balance, and let them go again.
 *
 *  The course's register list leaves three automatic loops running in the camera (COM8 = 0xE7): automatic
 *  exposure (AEC: how long each row collects light), automatic gain (AGC: how much the signal is amplified) and
 *  automatic white balance (AWB: how much red and blue are scaled so that white looks white). They keep the
 *  picture looking right as the light changes, but they also change the colours a design measures: a white
 *  sheet coming into view darkens everything else, and a large patch of one colour pulls the white balance.
 *  A colour threshold that worked a minute ago can stop working with nothing about the object changed.
 *
 *  Holding them: one SCCB write, COM8 = 0xE0, turns the three loops off, and the camera keeps the exposure,
 *  gain and white-balance gains it had at that moment. COM8 = 0xE7 (the course's value) lets them run again.
 *  Hold once the picture has settled in the room you are in, and the colours then stay where they are whatever
 *  else comes into view; they change only if the room's light itself changes, which a held camera does not
 *  follow (then release, let it settle, and hold again).
 *
 *  This module sits beside ov7670_config, which is unchanged. ov7670_config sends the course's list and then
 *  leaves the SCCB bus idle (`done`). From then on this module has the bus: the top level switches SIO_C over
 *  to it while `cfg_done` is high, and SIO_D is open drain, so both modules sit on it:
 *
 *      assign GPIO_1[25] = cfg_done ? hold_sioc : cfg_sioc;     // SIO_C
 *      // GPIO_1[24] is SIO_D: connect it to both modules' siod
 *
 *  `hold` is any signal (a switch, a button, your design): high asks for the hold, low for the loops to run.
 *  The course's top_level.sv uses SW9.
 *  A hold asked for during the first SETTLE_MS after configuration waits until then, so that the camera has
 *  adapted to the room first. HOLD_AFTER_MS > 0 also holds by itself that long after configuration (and
 *  `hold` then only matters until that time). After a reset the camera is configured again, its loops run
 *  again (the list sends 0xE7), and the hold is sent again once allowed.
 *
 *  Runs on the SCCB bit clock (100 kHz), like ov7670_config; a write takes 32 of its clocks, 320 us. A write
 *  the camera does not acknowledge is sent again. A reset in the middle of a write lets it finish: it is over
 *  long before ov7670_config's first write, which waits 3 ms (RESET# low, then the camera's settling time).
 *
 *  The camera model in simulation has no automatic loops, so there the hold changes nothing in the picture:
 *  the model only notes each change in its log. It matters on the board.
 */
module ov7670_hold #(
    parameter int SCCB_CLK_HZ   = 100_000,  // the clock this module and its I2C master run on
    parameter int SETTLE_MS     = 2000,     // a hold waits at least this long after configuration
    parameter int HOLD_AFTER_MS = 0         // 0: hold only when `hold` asks; N: also hold by itself N ms after configuration
) (
    input  logic clk,          // SCCB bit clock
    input  logic reset,        // synchronous, active high (as ov7670_config's)
    input  logic cfg_done,     // ov7670_config's done: the camera has the list (loops running) and the bus is free
    input  logic hold,         // any clock domain: 1 = hold exposure, gain and white balance; 0 = let them run
    output logic sioc,         // SIO_C: the top level uses this one while cfg_done is high
    inout  wire  siod,         // SIO_D, open drain, shared with ov7670_config
    output logic held,         // the camera is holding: its last COM8 write from here was 0xE0, acknowledged
    output logic busy,         // a write is being sent
    output logic error = 1'b0  // the last write was not acknowledged (it is being sent again)
);
    localparam logic [15:0] COM8_HOLD = 16'h13E0;   // AEC, AGC and AWB off; fast AEC, unlimited step, banding filter on
    localparam logic [15:0] COM8_RUN  = 16'h13E7;   // the course's value: all three on
    localparam int T_SETTLE = SETTLE_MS     * (SCCB_CLK_HZ / 1000);
    localparam int T_AUTO   = HOLD_AFTER_MS * (SCCB_CLK_HZ / 1000);
    localparam int T_MAX    = (T_AUTO > T_SETTLE) ? T_AUTO : T_SETTLE;

    // `hold`, through two flip-flops
    logic [1:0] hold_s = '0;
    always_ff @(posedge clk) hold_s <= {hold_s[0], hold};

    // time since configuration, up to T_MAX
    logic [$clog2(T_MAX + 2)-1:0] since = '0;
    always_ff @(posedge clk)
        if (reset || !cfg_done) since <= '0;
        else if (since != T_MAX[$bits(since)-1:0]) since <= since + 1'b1;
    logic settled, auto;
    assign settled = (since >= T_SETTLE[$bits(since)-1:0]);
    assign auto    = (HOLD_AFTER_MS > 0) && (since >= T_AUTO[$bits(since)-1:0]);
    logic want;
    assign want = settled && (hold_s[1] || auto);

    // the I2C master from Lesson 3, as ov7670_config uses it
    logic [15:0] cmd = COM8_RUN;
    logic        target = 1'b0;      // the COM8 state being written: 1 = hold
    logic        valid, ready, m_error;
    i2c_master u_master (
        .clk(clk), .i2c_scl(sioc), .i2c_sda(siod),
        .slav_addr(7'h21), .read_not_write(1'b0), .reg_addr(cmd[15:8]),
        .write_data(cmd[7:0]), .write_valid(valid), .write_ready(ready),
        .read_data(), .read_valid(), .read_ready(1'b1), .error(m_error));

    // IDLE -> OFFER (the master takes the write at the next edge) -> SENDING (until the master is idle again: in
    // that first idle cycle `m_error` is the verdict on the write, as in ov7670_config) -> IDLE.
    enum logic [1:0] {IDLE, OFFER, SENDING} state = IDLE;
    logic applied = 1'b0;            // what the camera has: 0 = loops running (the list's 0xE7), 1 = held
    assign valid = (state == OFFER);
    assign busy  = (state != IDLE);
    assign held  = applied;

    always_ff @(posedge clk) begin
        if (reset || !cfg_done) begin
            // the camera is being configured again: the list sends COM8 = 0xE7, so its loops will be running
            applied <= 1'b0;
            if (state == IDLE) error <= 1'b0;
        end
        case (state)
            IDLE:
                if (cfg_done && !reset && ready && want != applied) begin
                    cmd    <= want ? COM8_HOLD : COM8_RUN;
                    target <= want;
                    state  <= OFFER;
                end
            OFFER:
                if (ready) state <= SENDING;
            SENDING:
                if (ready) begin
                    error <= m_error;
                    if (!m_error && cfg_done && !reset) applied <= target;   // refused: IDLE sends it again
                    state <= IDLE;
                end
            default: state <= IDLE;
        endcase
    end
endmodule
