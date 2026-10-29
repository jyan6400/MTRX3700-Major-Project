`default_nettype none
// V3-D1 experimental tile-count temporal red detector.
// Two complete frames: capture_off=1 and then capture_off=0, each with frame_start,
// exactly WIDTH*HEIGHT in-bounds pixels (pixel_valid,x,y), and frame_end.
// Results are published on the ON frame_end. Not connected to V3 motor control.
module temporal_red_tile_detector #(
    parameter int WIDTH = 640,
    parameter int HEIGHT = 480,
    parameter int TILE = 16,
    parameter int MIN_DELTA = 5,
    parameter int MIN_RED_5 = 9,
    parameter int RED_MARGIN_5 = 4
)(
    input wire clk,
    input wire rst_n,
    input wire frame_start,
    input wire frame_end,
    input wire capture_off,
    input wire pixel_valid,
    input wire [9:0] x,
    input wire [9:0] y,
    input wire [15:0] rgb565,
    output logic result_valid,
    output logic target_valid,
    output logic [9:0] target_x,
    output logic [9:0] target_y,
    output logic [15:0] best_delta,
    output logic baseline_ready
);
    localparam int NX = (WIDTH + TILE - 1) / TILE;
    localparam int NY = (HEIGHT + TILE - 1) / TILE;
    localparam int N = NX * NY;
    localparam int COUNT_W = $clog2(TILE*TILE + 1);
    localparam int IDX_W = (N > 1) ? $clog2(N) : 1;
    logic [COUNT_W-1:0] off_count [0:N-1];
    logic [COUNT_W-1:0] on_count [0:N-1];
    logic frame_open;
    logic mode_off;
    logic [31:0] pixel_count;
    wire [4:0] r5 = rgb565[15:11];
    wire [4:0] g5 = rgb565[10:6]; // drop LSB of green RGB565
    wire [4:0] b5 = rgb565[4:0];
    wire red_pixel = (r5 >= MIN_RED_5) &&
                     (r5 >= g5 + 6'(RED_MARGIN_5)) &&
                     (r5 >= b5 + 6'(RED_MARGIN_5));
    wire [IDX_W-1:0] tile_index = IDX_W'((int'(y)/TILE)*NX + int'(x)/TILE);
    integer i;
    integer delta;
    integer max_delta;
    integer max_index;

    initial begin
        assert (WIDTH > 0 && HEIGHT > 0 && TILE > 0 && MIN_DELTA > 0)
            else $fatal(1,"Invalid temporal detector parameters");
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_open <= 1'b0;
            mode_off <= 1'b0;
            pixel_count <= '0;
            baseline_ready <= 1'b0;
            result_valid <= 1'b0;
            target_valid <= 1'b0;
            target_x <= '0;
            target_y <= '0;
            best_delta <= '0;
            for (integer j=0; j<N; j=j+1) begin
                off_count[j] <= '0;
                on_count[j] <= '0;
            end
        end else begin
            result_valid <= 1'b0;
            if (frame_start) begin
                frame_open <= 1'b1;
                mode_off <= capture_off;
                pixel_count <= '0;
                // A new OFF frame invalidates the old baseline immediately.
                if (capture_off) begin
                    baseline_ready <= 1'b0;
                    target_valid <= 1'b0;
                    for (integer j=0; j<N; j=j+1) off_count[j] <= '0;
                end else begin
                    for (integer j=0; j<N; j=j+1) on_count[j] <= '0;
                end
            end else if (frame_open && pixel_valid && x < WIDTH && y < HEIGHT) begin
                pixel_count <= pixel_count + 1'b1;
                if (red_pixel) begin
                    if (mode_off && off_count[tile_index] < TILE*TILE)
                        off_count[tile_index] <= off_count[tile_index] + 1'b1;
                    if (!mode_off && on_count[tile_index] < TILE*TILE)
                        on_count[tile_index] <= on_count[tile_index] + 1'b1;
                end
            end else if (frame_end && frame_open) begin
                frame_open <= 1'b0;
                if (pixel_count != WIDTH*HEIGHT) begin
                    baseline_ready <= 1'b0; // reject truncated or duplicate frames
                    target_valid <= 1'b0;
                end else if (mode_off) begin
                    baseline_ready <= 1'b1;
                end else begin
                    result_valid <= 1'b1;
                    target_valid <= 1'b0;
                    best_delta <= '0;
                    max_delta = 0;
                    max_index = 0;
                    if (baseline_ready) begin
                        for (i=0; i<N; i=i+1) begin
                            delta = int'(on_count[i]) - int'(off_count[i]);
                            if (delta > max_delta) begin
                                max_delta = delta;
                                max_index = i;
                            end
                        end
                        if (max_delta >= MIN_DELTA) begin
                            target_valid <= 1'b1;
                            best_delta <= 16'(max_delta);
                            target_x <= 10'(((max_index % NX)*TILE) + TILE/2);
                            target_y <= 10'(((max_index / NX)*TILE) + TILE/2);
                        end
                    end
                    baseline_ready <= 1'b0; // require fresh OFF before next ON
                end
            end
        end
    end
endmodule
`default_nettype wire
