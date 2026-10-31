`timescale 1ns/1ps
`default_nettype none

// Group 26 - beacon visual odometry (Wilson, M2): the VO on the DE1-SoC's six 7-segment displays,
// so the tutor can read range against a tape measure.
//
//   HEX5      the LED being tracked: r, G or b
//   HEX4      blank; '-' when the shown velocity is negative (closing on the LED)
//   HEX3..0   show_vel = 0: range in mm (0 to 9999);  show_vel = 1: |velocity| in mm/s
//             "----" when there is no track
// Segments are active low, [6:0] = g f e d c b a (the DE1-SoC's wiring). Values are latched on
// `updated`, so the display changes once a frame and never shows a half-updated number.
module vo_hex_display (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        updated,
    input  wire [1:0]  sel,
    input  wire        track_valid,
    input  wire signed [15:0] range_mm,
    input  wire signed [15:0] vel_mm_s,
    input  wire        show_vel,
    output logic [6:0] hex0, hex1, hex2, hex3, hex4, hex5
);
    localparam logic [6:0] BLANK = 7'b1111111, DASH = 7'b0111111;

    function automatic logic [6:0] seg(input logic [3:0] d);
        case (d)
            4'd0: seg = 7'b1000000; 4'd1: seg = 7'b1111001; 4'd2: seg = 7'b0100100; 4'd3: seg = 7'b0110000;
            4'd4: seg = 7'b0011001; 4'd5: seg = 7'b0010010; 4'd6: seg = 7'b0000010; 4'd7: seg = 7'b1111000;
            4'd8: seg = 7'b0000000; 4'd9: seg = 7'b0010000; default: seg = DASH;
        endcase
    endfunction

    // binary (0..9999) to four BCD digits, double dabble
    function automatic logic [15:0] bcd4(input logic [13:0] b);
        logic [29:0] s;
        s = {16'd0, b};
        for (int i = 0; i < 14; i++) begin
            for (int d = 0; d < 4; d++)
                if (s[14 + 4*d +: 4] >= 4'd5) s[14 + 4*d +: 4] = s[14 + 4*d +: 4] + 4'd3;
            s = s << 1;
        end
        return s[29:14];
    endfunction

    logic        tv_q, neg_q;
    logic [1:0]  sel_q;
    logic [13:0] mag_q;
    wire signed [15:0] shown = show_vel ? vel_mm_s : range_mm;
    wire [15:0] mag = (shown < 0) ? 16'(-shown) : 16'(shown);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tv_q <= 1'b0; neg_q <= 1'b0; sel_q <= '0; mag_q <= '0;
        end else if (updated) begin
            tv_q  <= track_valid;
            neg_q <= (shown < 0);
            sel_q <= sel;
            mag_q <= (mag > 16'd9999) ? 14'd9999 : 14'(mag);
        end
    end

    wire [15:0] d = bcd4(mag_q);
    always_comb begin
        case (sel_q)
            2'd0:    hex5 = 7'b0101111;      // r
            2'd1:    hex5 = 7'b1000010;      // G
            2'd2:    hex5 = 7'b0000011;      // b
            default: hex5 = DASH;
        endcase
        hex4 = (tv_q && neg_q) ? DASH : BLANK;
        if (tv_q) begin
            hex3 = seg(d[15:12]); hex2 = seg(d[11:8]); hex1 = seg(d[7:4]); hex0 = seg(d[3:0]);
        end else begin
            hex3 = DASH; hex2 = DASH; hex1 = DASH; hex0 = DASH;
        end
    end
endmodule

`default_nettype wire
