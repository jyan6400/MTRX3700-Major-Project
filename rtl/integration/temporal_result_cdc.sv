`default_nettype none
// Single-entry coherent result mailbox. Both clocks may be asynchronous.
// Input payload held until destination acknowledges receipt. Events while busy are dropped.
module temporal_result_cdc #(
    parameter int XW=9, YW=8, CW=17
)(
    input wire src_clk, input wire src_rst_n,
    input wire src_valid,
    input wire src_target_valid,
    input wire [XW-1:0] src_x,
    input wire [YW-1:0] src_y,
    input wire [CW-1:0] src_count,
    output wire src_ready,
    output logic src_overrun,
    input wire dst_clk, input wire dst_rst_n,
    output logic dst_valid,
    output logic dst_target_valid,
    output logic [XW-1:0] dst_x,
    output logic [YW-1:0] dst_y,
    output logic [CW-1:0] dst_count
);
    localparam int PW=1+XW+YW+CW;
    logic [PW-1:0] hold_payload;
    logic req_toggle, ack_toggle;
    (* async_reg = "true" *) logic ack_s1, ack_s2;
    (* async_reg = "true" *) logic req_d1, req_d2;
    logic req_seen;
    logic capture_pending;
    assign src_ready = (req_toggle == ack_s2);
    always_ff @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            req_toggle <= 1'b0;
            hold_payload <= '0;
            ack_s1 <= 1'b0;
            ack_s2 <= 1'b0;
            src_overrun <= 1'b0;
        end else begin
            ack_s1 <= ack_toggle;
            ack_s2 <= ack_s1;
            src_overrun <= 1'b0;
            if (src_valid) begin
                if (src_ready) begin
                    hold_payload <= {src_target_valid,src_x,src_y,src_count};
                    req_toggle <= ~req_toggle;
                end else src_overrun <= 1'b1;
            end
        end
    end
    always_ff @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            req_d1 <= 1'b0;
            req_d2 <= 1'b0;
            req_seen <= 1'b0;
            capture_pending <= 1'b0;
            ack_toggle <= 1'b0;
            dst_valid <= 1'b0;
            dst_target_valid <= 1'b0;
            dst_x <= '0;
            dst_y <= '0;
            dst_count <= '0;
        end else begin
            req_d1 <= req_toggle;
            req_d2 <= req_d1;
            dst_valid <= 1'b0;
            if (capture_pending) begin
                {dst_target_valid,dst_x,dst_y,dst_count} <= hold_payload;
                dst_valid <= 1'b1;
                ack_toggle <= req_seen;
                capture_pending <= 1'b0;
            end else if (req_d2 != req_seen) begin
                req_seen <= req_d2;
                capture_pending <= 1'b1;
            end
        end
    end
endmodule
`default_nettype wire
