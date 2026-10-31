`timescale 1ns/1ps
`default_nettype none

// Group 26 - beacon visual odometry (Wilson, M2).
// One W-bit result from the source clock to the destination clock, whole: a toggle request and
// a toggle acknowledge, each through two flops. The source holds the data from src_valid until the
// acknowledge comes back, so the destination copies a word that is not changing. A src_valid while
// a word is still in flight is dropped (src_overrun pulses): results here come once a frame, about
// 20 ms apart, and the round trip takes a few clocks of each side.
module vo_mailbox_cdc #(
    parameter int W = 160
) (
    input  wire          src_clk,
    input  wire          src_rst_n,
    input  wire          src_valid,
    input  wire [W-1:0]  src_data,
    output logic         src_overrun,

    input  wire          dst_clk,
    input  wire          dst_rst_n,
    output logic         dst_valid,      // one dst clock
    output logic [W-1:0] dst_data        // holds until the next dst_valid
);
    logic         req, busy;
    logic [W-1:0] hold;
    (* async_reg = "true" *) logic [1:0] ack_sync;
    logic         ack;
    (* async_reg = "true" *) logic [1:0] req_sync;
    logic         req_seen;

    // source side
    always_ff @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            req <= 1'b0; hold <= '0; ack_sync <= '0; src_overrun <= 1'b0;
        end else begin
            ack_sync    <= {ack_sync[0], ack};
            src_overrun <= 1'b0;
            if (src_valid) begin
                if (busy) src_overrun <= 1'b1;
                else begin
                    hold <= src_data;
                    req  <= ~req;
                end
            end
        end
    end
    assign busy = (req != ack_sync[1]);

    // destination side
    always_ff @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            req_sync <= '0; req_seen <= 1'b0; ack <= 1'b0; dst_valid <= 1'b0; dst_data <= '0;
        end else begin
            req_sync  <= {req_sync[0], req};
            dst_valid <= 1'b0;
            if (req_sync[1] != req_seen) begin
                req_seen  <= req_sync[1];
                dst_data  <= hold;               // stable: the source changes it only after the acknowledge
                dst_valid <= 1'b1;
                ack       <= req_sync[1];
            end
        end
    end
endmodule

`default_nettype wire
