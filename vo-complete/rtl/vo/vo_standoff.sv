`timescale 1ns/1ps
`default_nettype none

// Group 26 - beacon visual odometry (Wilson, M2): V4 "position itself a set distance from the LED".
//
// While `enable` is high (the sequencer is reversing away from, or approaching, the bay's LED),
// `reached` rises once the tracked range has been at or past target_mm for CONFIRM consecutive
// tracker updates: range >= target when reversing away (approaching = 0), range <= target when
// approaching (approaching = 1). It holds until enable drops. `lost` is high while enabled with no
// track: the sequencer then falls back to its timed reverse.
//
// The bypass (SW9) is not here: the sequencer chooses between `reached` and its own timer, so the
// timed V3 reverse stays exactly as it was (README_VO.md, integration). Golden model: vo_model.Standoff.
module vo_standoff #(
    parameter int CONFIRM = 2
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        approaching,
    input  wire [15:0] target_mm,
    input  wire        updated,             // beacon_tracker's updated
    input  wire        track_valid,
    input  wire signed [15:0] range_mm,
    output logic       reached,
    output logic       lost
);
    logic [2:0] n;
    wire signed [16:0] tgt = signed'({1'b0, target_mm});
    wire hit = track_valid && (approaching ? (17'(range_mm) <= tgt) : (17'(range_mm) >= tgt));

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            n <= '0; reached <= 1'b0;
        end else if (!enable) begin
            n <= '0; reached <= 1'b0;
        end else if (updated) begin
            if (hit) begin
                if (n != 3'd7) n <= n + 3'd1;
                if (n + 3'd1 >= 3'(CONFIRM)) reached <= 1'b1;
            end else begin
                n <= '0;
            end
        end
    end
    assign lost = enable && !track_valid;
endmodule

`default_nettype wire
