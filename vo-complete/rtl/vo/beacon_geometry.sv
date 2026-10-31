`timescale 1ns/1ps
`default_nettype none

// Group 26 - beacon visual odometry (Wilson, M2): pixels to millimetres.
//
// For each LED seen (beacon_detect's Q4 centroid), its position in the robot frame from its
// elevation: the LEDs are DH_MM above the camera lens, the camera is tilted up by the pitch.
//     U = x - CX,   V = CY - y                         (Q4 pixels; image y grows downward)
//     up = V cos + F sin,   fw = F cos - V sin          (Q18: Q4 x Q14)
//     forward = DH * fw / up,   lateral = DH * U * 2^14 / up      (mm, lateral right positive)
// No trigonometry at run time: the pitch comes in as its sine and cosine (Q14) from calibration.
// An LED is dropped when up <= 0 or fw <= 0 (impossible geometry) or forward > MAX_RANGE_MM.
//
// With two or more LEDs: the widest pair (i < j; red, green, blue left to right, SPACING_MM apart)
//     pair_ok    = |dF^2 + dL^2 - S^2| <= S^2 >> PAIR_TOL_SHIFT   (the two positions are the right
//                  distance apart: the calibration and the detection agree)
//     heading_q12 = -dF * 4096 / dL = tan(heading) * 4096, positive turned left of square-on
//
// One shared divider: up to 7 divisions a frame, about 330 clocks in all. Signed division
// truncates toward zero (magnitudes divided, sign restored). Golden model: vo_model.geometry.
module beacon_geometry #(
    parameter int F_Q4           = 8960,    // focal length, 1/16 px
    parameter int CX_Q4          = 5112,
    parameter int CY_Q4          = 3832,
    parameter int SIN_Q14        = 3406,    // camera pitch up 12 degrees
    parameter int COS_Q14        = 16026,
    parameter int DH_MM          = 150,
    parameter int MAX_RANGE_MM   = 4000,
    parameter int SPACING_MM     = 300,
    parameter int PAIR_TOL_SHIFT = 2
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,              // one clock: a detection result
    input  wire  [2:0]  det_valid,
    input  wire  [41:0] det_x_q4,           // 3 x 14
    input  wire  [41:0] det_y_q4,

    output logic        done,               // one clock
    output logic [2:0]  led_valid,
    output logic [47:0] forward_mm,         // 3 x 16, unsigned
    output logic [47:0] lateral_mm,         // 3 x 16, signed
    output logic        pair_valid,
    output logic        pair_ok,
    output logic [3:0]  pair,               // {i, j}
    output logic signed [15:0] heading_q12,
    output logic        busy
);
    localparam int NW = 44;
    localparam int DW = 32;

    typedef enum logic [2:0] {S_IDLE, S_LED, S_FWD, S_LAT, S_PAIR, S_HEAD, S_DONE} state_t;
    state_t state;
    logic [1:0] k;

    logic [2:0]  dv;
    logic [41:0] dx, dy;
    logic [2:0]  lv;
    logic [15:0] fwd [3];
    logic signed [15:0] lat [3];

    // the LED being worked on
    logic signed [15:0] U, V;
    logic signed [47:0] up_w, fw_w;
    always_comb begin
        U    = 16'(signed'({2'b00, dx[k*14 +: 14]}) - 16'(CX_Q4));
        V    = 16'(16'(CY_Q4) - signed'({2'b00, dy[k*14 +: 14]}));
        up_w = 48'(V) * 48'(COS_Q14) + 48'(F_Q4) * 48'(SIN_Q14);
        fw_w = 48'(F_Q4) * 48'(COS_Q14) - 48'(V) * 48'(SIN_Q14);
    end
    logic signed [47:0] up_q, fw_q;
    logic signed [15:0] U_q;

    // the divider
    logic          div_start, div_busy, div_done;
    logic [NW-1:0] div_num, div_quo;
    logic [DW-1:0] div_den, div_rem;
    logic          neg;
    vo_divider #(.NW(NW), .DW(DW)) u_div (
        .clk(clk), .rst_n(rst_n), .start(div_start), .num(div_num), .den(div_den),
        .busy(div_busy), .done(div_done), .quo(div_quo), .rem(div_rem));
    logic waiting;

    // pair
    logic [1:0] pi, pj;
    logic signed [17:0] dF, dL;
    logic signed [36:0] dF37, dL37;
    logic [36:0] d2, s2;
    logic [11:0] span;
    always_comb begin
        dF   = signed'({2'b00, fwd[pj]}) - signed'({2'b00, fwd[pi]});
        dL   = 18'(lat[pj]) - 18'(lat[pi]);
        dF37 = 37'(dF);
        dL37 = 37'(dL);
        d2   = 37'(dF37 * dF37 + dL37 * dL37);
        span = 12'(SPACING_MM) * 12'({2'b00, pj} - {2'b00, pi});
        s2   = 37'(span) * 37'(span);
    end
    logic [36:0] d2_err;
    assign d2_err = (d2 >= s2) ? d2 - s2 : s2 - d2;

    wire signed [NW-1:0] q_signed = neg ? -signed'(div_quo) : signed'(div_quo);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; k <= '0; dv <= '0; dx <= '0; dy <= '0; lv <= '0;
            for (int i = 0; i < 3; i++) begin fwd[i] <= '0; lat[i] <= '0; end
            up_q <= '0; fw_q <= '0; U_q <= '0;
            div_start <= 1'b0; div_num <= '0; div_den <= '0; neg <= 1'b0; waiting <= 1'b0;
            pi <= '0; pj <= '0;
            done <= 1'b0; led_valid <= '0; forward_mm <= '0; lateral_mm <= '0;
            pair_valid <= 1'b0; pair_ok <= 1'b0; pair <= '0; heading_q12 <= '0;
        end else begin
            done      <= 1'b0;
            div_start <= 1'b0;
            case (state)
                S_IDLE: if (start) begin
                    dv <= det_valid; dx <= det_x_q4; dy <= det_y_q4;
                    lv <= '0;
                    for (int i = 0; i < 3; i++) begin fwd[i] <= '0; lat[i] <= '0; end
                    k <= '0;
                    state <= S_LED;
                end
                S_LED: begin                                   // LED k: check, then forward
                    if (!dv[k] || up_w <= 0 || fw_w <= 0) begin
                        if (k == 2'd2) state <= S_PAIR; else k <= k + 2'd1;
                    end else begin
                        up_q <= up_w; fw_q <= fw_w; U_q <= U;
                        div_num   <= NW'(48'(DH_MM) * fw_w);
                        div_den   <= DW'(up_w);
                        neg       <= 1'b0;
                        div_start <= 1'b1;
                        waiting   <= 1'b1;
                        state     <= S_FWD;
                    end
                end
                S_FWD: if (waiting && div_done) begin
                    waiting <= 1'b0;
                    if (div_quo > NW'(MAX_RANGE_MM)) begin             // out of range: drop this LED
                        if (k == 2'd2) state <= S_PAIR; else begin k <= k + 2'd1; state <= S_LED; end
                    end else begin
                        fwd[k]    <= 16'(div_quo);
                        div_num   <= NW'(48'(DH_MM) * 48'(U_q < 0 ? -U_q : U_q) * 48'(16384));
                        div_den   <= DW'(up_q);
                        neg       <= (U_q < 0);
                        div_start <= 1'b1;
                        waiting   <= 1'b1;
                        state     <= S_LAT;
                    end
                end
                S_LAT: if (waiting && div_done) begin
                    waiting <= 1'b0;
                    lv[k]   <= 1'b1;
                    if (q_signed > 32767)       lat[k] <= 16'sd32767;
                    else if (q_signed < -32767) lat[k] <= -16'sd32767;
                    else                        lat[k] <= 16'(q_signed);
                    if (k == 2'd2) state <= S_PAIR; else begin k <= k + 2'd1; state <= S_LED; end
                end
                S_PAIR: begin                                  // the widest pair of valid LEDs
                    pi <= lv[0] ? 2'd0 : (lv[1] ? 2'd1 : 2'd2);
                    pj <= lv[2] ? 2'd2 : (lv[1] ? 2'd1 : 2'd0);
                    state <= S_HEAD;
                end
                S_HEAD: begin
                    if (!waiting) begin
                        led_valid <= lv;
                        for (int i = 0; i < 3; i++) begin
                            forward_mm[i*16 +: 16] <= fwd[i];
                            lateral_mm[i*16 +: 16] <= lat[i];
                        end
                        if (lv[0] + lv[1] + lv[2] < 2) begin
                            pair_valid <= 1'b0; pair_ok <= 1'b0; pair <= '0; heading_q12 <= '0;
                            state <= S_DONE;
                        end else begin
                            pair_valid <= 1'b1;
                            pair       <= {pi, pj};
                            if (dL <= 0) begin
                                pair_ok <= 1'b0; heading_q12 <= '0;
                                state <= S_DONE;
                            end else begin
                                pair_ok   <= (d2_err <= (s2 >> PAIR_TOL_SHIFT));
                                div_num   <= NW'((dF < 0 ? -dF : dF)) << 12;
                                div_den   <= DW'(dL);
                                neg       <= (dF > 0);              // heading = -dF / dL
                                div_start <= 1'b1;
                                waiting   <= 1'b1;
                            end
                        end
                    end else if (div_done) begin
                        waiting <= 1'b0;
                        if (q_signed > 32767)       heading_q12 <= 16'sd32767;
                        else if (q_signed < -32767) heading_q12 <= -16'sd32767;
                        else                        heading_q12 <= 16'(q_signed);
                        state <= S_DONE;
                    end
                end
                S_DONE: begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
    assign busy = (state != S_IDLE);
endmodule

`default_nettype wire
