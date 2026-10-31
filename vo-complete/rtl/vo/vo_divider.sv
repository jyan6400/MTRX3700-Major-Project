`timescale 1ns/1ps
`default_nettype none

// Group 26 - beacon visual odometry (Wilson, M2).
// Unsigned restoring divider, one quotient bit per clock: quotient = floor(num / den).
// `start` loads num and den (ignored while busy); `done` pulses for one clock NW clocks later,
// with `quo` and `rem` valid from then until the next start. den = 0 gives quo = all ones.
// Shared by beacon_detect (centroids) and beacon_geometry (ranges), so each needs one divider.
module vo_divider #(
    parameter int NW = 32,           // numerator and quotient width
    parameter int DW = 32            // denominator width
) (
    input  wire           clk,
    input  wire           rst_n,
    input  wire           start,
    input  wire [NW-1:0]  num,
    input  wire [DW-1:0]  den,
    output logic          busy,
    output logic          done,
    output logic [NW-1:0] quo,
    output logic [DW-1:0] rem
);
    localparam int CW = $clog2(NW + 1);
    logic [CW-1:0] n_left;
    logic [DW-1:0] d_q;
    logic [DW:0]   r_shift;
    assign r_shift = {rem, quo[NW-1]};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy   <= 1'b0;
            done   <= 1'b0;
            quo    <= '0;
            rem    <= '0;
            d_q    <= '0;
            n_left <= '0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                if (start) begin
                    busy   <= 1'b1;
                    quo    <= num;
                    rem    <= '0;
                    d_q    <= den;
                    n_left <= CW'(NW);
                end
            end else begin
                if (r_shift >= {1'b0, d_q}) begin
                    rem <= DW'(r_shift - {1'b0, d_q});
                    quo <= {quo[NW-2:0], 1'b1};
                end else begin
                    rem <= r_shift[DW-1:0];
                    quo <= {quo[NW-2:0], 1'b0};
                end
                n_left <= n_left - 1'b1;
                if (n_left == CW'(1)) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            end
        end
    end
endmodule

`default_nettype wire
