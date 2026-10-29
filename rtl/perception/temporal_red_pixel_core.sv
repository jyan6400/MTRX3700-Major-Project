`default_nettype none
// V3-D3: streaming OFF-reference / ON-pixel temporal comparison prototype.
// One RGB565 reference sample per pixel in synchronous inferred block RAM.
// Caller MUST provide exactly WIDTH*HEIGHT pixels in raster order for each
// frame, with frame_start on pixel 0 and frame_end on last pixel.
// A frame is accepted only if full length and no invalidation occurred.
// Pixel change output is from one clock before (synchronous RAM read latency).
// WARNING: pixel hit stream is NOT connected-component filtering. A downstream
// blob filter and fresh bay-specific OFF/ON handshake are mandatory for motors.
module temporal_red_pixel_core #(
    parameter int unsigned WIDTH = 320,
    parameter int unsigned HEIGHT = 240,
    parameter int unsigned RED_DELTA = 35,
    parameter int unsigned CHROMA_MARGIN = 0
) (
    input wire clk,
    input wire rst_n,
    input wire invalidate,
    input wire capture_off, // static across full OFF frame
    input wire capture_on,  // static across full ON frame
    input wire pixel_valid,
    input wire frame_start,
    input wire frame_end,
    input wire [15:0] pixel_rgb565,
    output logic off_ready,
    output logic pair_complete,
    output logic pair_error,
    output logic hit_valid,
    output logic hit,
    output logic [$clog2(WIDTH)-1:0] hit_x,
    output logic [$clog2(HEIGHT)-1:0] hit_y
);
    localparam int unsigned PIXELS = WIDTH * HEIGHT;
    localparam int unsigned AW = (PIXELS <= 1) ? 1 : $clog2(PIXELS);
    localparam int unsigned CW = $clog2(PIXELS + 1);
    localparam int unsigned XW = (WIDTH <= 1) ? 1 : $clog2(WIDTH);
    localparam int unsigned YW = (HEIGHT <= 1) ? 1 : $clog2(HEIGHT);
    (* ramstyle = "M10K" *) logic [15:0] reference_ram [0:PIXELS-1];
    logic [15:0] ref_data;
    logic [15:0] on_data_q;
    logic [XW-1:0] x_q;
    logic [YW-1:0] y_q;
    logic read_pending;
    logic final_pending;
    logic [CW-1:0] count;
    logic [XW-1:0] x_pos;
    logic [YW-1:0] y_pos;
    logic frame_bad;
    logic off_committed;
    logic on_in_progress;
    logic off_in_progress;

    // Comparisons are in 8-bit reconstructed channels, matching the
    // convention used by the original RGB888 photo-reference algorithm.
    function automatic logic changed(input logic [15:0] on_px, input logic [15:0] off_px);
        logic signed [10:0] dr, dg, db;
        logic signed [11:0] score2;
        logic [7:0] ron, gon, bon, roff, goff, boff;
        begin
            ron  = {on_px[15:11],on_px[15:13]};
            gon  = {on_px[10:5],on_px[10:9]};
            bon  = {on_px[4:0],on_px[4:2]};
            roff = {off_px[15:11],off_px[15:13]};
            goff = {off_px[10:5],off_px[10:9]};
            boff = {off_px[4:0],off_px[4:2]};
            dr = $signed({1'b0,ron}) - $signed({1'b0,roff});
            dg = $signed({1'b0,gon}) - $signed({1'b0,goff});
            db = $signed({1'b0,bon}) - $signed({1'b0,boff});
            score2 = (dr <<< 1) - dg - db;
            changed = (dr > 0) &&
                      (score2 >= $signed(2 * RED_DELTA)) &&
                      (dr >= $signed(CHROMA_MARGIN));
        end
    endfunction

    // Dedicated synchronous RAM interface for Cyclone V block RAM inference.
    // The RAM itself is deliberately not reset.
    wire [AW-1:0] ram_addr = frame_start ? AW'(0) : AW'(count);

    wire ram_write_en = rst_n && !invalidate && pixel_valid &&
        (frame_start || off_in_progress) &&
        capture_off && !capture_on;

    wire ram_read_en = rst_n && !invalidate && pixel_valid &&
        (frame_start || on_in_progress) &&
        capture_on && !capture_off && off_committed;

    always_ff @(posedge clk) begin
        if (ram_write_en)
            reference_ram[ram_addr] <= pixel_rgb565;

        if (ram_read_en)
            ref_data <= reference_ram[ram_addr];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            on_data_q <= '0;
            x_q <= '0;
            y_q <= '0;
            read_pending <= 1'b0;
            final_pending <= 1'b0;
            count <= '0;
            x_pos <= '0;
            y_pos <= '0;
            frame_bad <= 1'b0;
            off_committed <= 1'b0;
            off_in_progress <= 1'b0;
            on_in_progress <= 1'b0;
            off_ready <= 1'b0;
            pair_complete <= 1'b0;
            pair_error <= 1'b0;
            hit_valid <= 1'b0;
            hit <= 1'b0;
            hit_x <= '0;
            hit_y <= '0;
        end else begin
            pair_complete <= 1'b0;
            pair_error <= 1'b0;
            hit_valid <= read_pending;
            hit <= 1'b0;
            if (read_pending) begin
                hit <= changed(on_data_q, ref_data);
                hit_x <= x_q;
                hit_y <= y_q;
            end
            read_pending <= 1'b0;
            if (final_pending) begin
                pair_complete <= 1'b1;
                off_committed <= 1'b0; // never reuse same OFF reference
                off_ready <= 1'b0;
                final_pending <= 1'b0;
            end
            if (invalidate) begin
                off_committed <= 1'b0;
                off_ready <= 1'b0;
                count <= '0;
                x_pos <= '0;
                y_pos <= '0;
                frame_bad <= 1'b0;
                off_in_progress <= 1'b0;
                on_in_progress <= 1'b0;
                read_pending <= 1'b0;
                final_pending <= 1'b0;
                pair_error <= 1'b1;
            end else if (pixel_valid) begin
                if (frame_start) begin
                    count <= '0;
                    x_pos <= '0;
                    y_pos <= '0;
                    frame_bad <= 1'b0;
                    off_in_progress <= capture_off && !capture_on;
                    on_in_progress <= capture_on && !capture_off && off_committed;
                    if (capture_off) begin
                        off_committed <= 1'b0;
                        off_ready <= 1'b0;
                    end
                end
                if ((frame_start || off_in_progress) && capture_off && !capture_on) begin

                end
                if ((frame_start || on_in_progress) && capture_on && !capture_off && off_committed) begin

                    on_data_q <= pixel_rgb565;
                    x_q <= frame_start ? '0 : x_pos;
                    y_q <= frame_start ? '0 : y_pos;
                    read_pending <= 1'b1;
                end
                if (frame_end) begin
                    if ((frame_start ? CW'(1) : count + CW'(1)) != CW'(PIXELS)) begin
                        pair_error <= 1'b1;
                        off_committed <= 1'b0;
                        off_ready <= 1'b0;
                    end else if ((frame_start || off_in_progress) && capture_off && !capture_on) begin
                        off_committed <= 1'b1;
                        off_ready <= 1'b1;
                    end else if ((frame_start || on_in_progress) && capture_on && !capture_off && off_committed) begin
                        final_pending <= 1'b1;
                    end else begin
                        pair_error <= 1'b1;
                    end
                    off_in_progress <= 1'b0;
                    on_in_progress <= 1'b0;
                end else begin
                    count <= (frame_start ? CW'(1) : count + CW'(1));
                    if (frame_start) x_pos <= XW'(1);
                    else if (x_pos == XW'(WIDTH-1)) begin
                        x_pos <= '0;
                        y_pos <= y_pos + 1'b1;
                    end else x_pos <= x_pos + 1'b1;
                end
            end
        end
    end
endmodule
`default_nettype wire
