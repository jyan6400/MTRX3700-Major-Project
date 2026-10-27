`timescale 1ns/1ps
`default_nettype none

// Group 26 V2: LED centroid with target validation and distractor rejection.
//
// Per frame, accumulates the red-pixel count, coordinate sums and bounding box.
// After frame_end:
//   cycle 1      latch totals, box width/height
//   cycle 2      box area and shape gates
//   cycles 3-34  serial division: centroid x, y and fill confidence
// frame_done / target_valid pulse together 34 PCLK cycles after frame_end.
//
// A frame is accepted when all gates pass:
//   - MIN_PIXELS <= count <= MAX_PIXELS     (too small = noise, too big = red background)
//   - box aspect ratio within MAX_ASPECT:1  (rejects stripes, e.g. floor reflections)
//   - confidence >= MIN_CONFIDENCE          (red pixels must fill their box: a second red
//                                            object far from the LED stretches the box,
//                                            drops the fill and rejects the frame instead
//                                            of steering to the midpoint of the two)
// confidence = 256 * count / box_area, saturated at 255 (a filled disc gives ~200).
//
// gate_bypass = 1 restores V1 acceptance (count >= MIN_PIXELS only) for the R9 bypass
// test; confidence and reject_flags are still reported so both modes can be compared.
// Left unconnected it reads 0, i.e. gating on.
module led_centroid #(
    parameter logic [31:0] MIN_PIXELS     = 32'd20,
    parameter logic [31:0] MAX_PIXELS     = 32'd40000,
    parameter int unsigned MAX_ASPECT     = 4,
    parameter logic [7:0]  MIN_CONFIDENCE = 8'd64
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        gate_bypass,
    input  wire        frame_start,
    input  wire        frame_end,
    input  wire        pixel_valid,
    input  wire        is_red,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_y,
    output logic        frame_done,
    output logic        target_valid,
    output logic [9:0]  target_x,
    output logic [9:0]  target_y,
    output logic [31:0] red_count,
    output logic [7:0]  confidence,
    output logic [3:0]  reject_flags,   // [0] too few, [1] too many, [2] aspect, [3] low fill
    output logic [9:0]  bbox_x_min,
    output logic [9:0]  bbox_x_max,
    output logic [9:0]  bbox_y_min,
    output logic [9:0]  bbox_y_max
);
    // ---------------- Accumulation (one pixel per cycle) ----------------
    logic [31:0] sum_x, sum_y, count;
    logic [9:0]  min_x, max_x, min_y, max_y;

    wire [31:0] base_x     = frame_start ? 32'd0 : sum_x;
    wire [31:0] base_y     = frame_start ? 32'd0 : sum_y;
    wire [31:0] base_count = frame_start ? 32'd0 : count;
    wire [9:0]  base_min_x = frame_start ? 10'h3FF : min_x;
    wire [9:0]  base_max_x = frame_start ? 10'h000 : max_x;
    wire [9:0]  base_min_y = frame_start ? 10'h3FF : min_y;
    wire [9:0]  base_max_y = frame_start ? 10'h000 : max_y;

    wire use_pixel = pixel_valid && is_red;

    wire [31:0] next_x     = base_x + (use_pixel ? {22'd0, pixel_x} : 32'd0);
    wire [31:0] next_y     = base_y + (use_pixel ? {22'd0, pixel_y} : 32'd0);
    wire [31:0] next_count = base_count + (use_pixel ? 32'd1 : 32'd0);
    wire [9:0]  next_min_x = (use_pixel && pixel_x < base_min_x) ? pixel_x : base_min_x;
    wire [9:0]  next_max_x = (use_pixel && pixel_x > base_max_x) ? pixel_x : base_max_x;
    wire [9:0]  next_min_y = (use_pixel && pixel_y < base_min_y) ? pixel_y : base_min_y;
    wire [9:0]  next_max_y = (use_pixel && pixel_y > base_max_y) ? pixel_y : base_max_y;

    // ---------------- Post-frame pipeline ----------------
    typedef enum logic [1:0] {IDLE, BOX, GATE, DIVIDE} state_t;
    state_t state;

    logic [31:0] frame_count, frame_sum_x, frame_sum_y;
    logic [9:0]  frame_min_x, frame_max_x, frame_min_y, frame_max_y;
    logic [10:0] box_w, box_h;          // 0..640, 0..480
    logic [2:0]  flags;               // [0] too few, [1] too many, [2] aspect

    // Three-lane restoring divider: x = sum_x / n, y = sum_y / n, c = 256n / area.
    logic [5:0]  bits_left;
    logic [31:0] den_xy, den_c;
    logic [31:0] dvd_x, dvd_y, dvd_c;
    logic [31:0] quo_x, quo_y, quo_c;
    logic [32:0] rem_x, rem_y, rem_c;

    wire [32:0] trial_x = {rem_x[31:0], dvd_x[31]};
    wire [32:0] trial_y = {rem_y[31:0], dvd_y[31]};
    wire [32:0] trial_c = {rem_c[31:0], dvd_c[31]};
    wire bit_x = trial_x >= {1'b0, den_xy};
    wire bit_y = trial_y >= {1'b0, den_xy};
    wire bit_c = trial_c >= {1'b0, den_c};
    wire [31:0] new_quo_x = {quo_x[30:0], bit_x};
    wire [31:0] new_quo_y = {quo_y[30:0], bit_y};
    wire [31:0] new_quo_c = {quo_c[30:0], bit_c};

    wire [21:0] box_area = box_w * box_h;
    wire [7:0]  conf_sat = (new_quo_c > 32'd255) ? 8'd255 : new_quo_c[7:0];

    // Shape gate inputs (MAX_ASPECT is a small constant, so these are cheap)
    wire [15:0] w_limit = box_h * MAX_ASPECT[4:0];
    wire [15:0] h_limit = box_w * MAX_ASPECT[4:0];

    // Acceptance on the final divide step, using the just-computed confidence.
    wire low_fill = conf_sat < MIN_CONFIDENCE;
    wire accept   = gate_bypass ? !flags[0] : (flags == 3'b000) && !low_fill;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_x <= 0; sum_y <= 0; count <= 0;
            min_x <= 10'h3FF; max_x <= 0; min_y <= 10'h3FF; max_y <= 0;
            state <= IDLE;
            frame_count <= 0; frame_sum_x <= 0; frame_sum_y <= 0;
            frame_min_x <= 0; frame_max_x <= 0; frame_min_y <= 0; frame_max_y <= 0;
            box_w <= 0; box_h <= 0; flags <= 0;
            bits_left <= 0; den_xy <= 0; den_c <= 0;
            dvd_x <= 0; dvd_y <= 0; dvd_c <= 0;
            quo_x <= 0; quo_y <= 0; quo_c <= 0;
            rem_x <= 0; rem_y <= 0; rem_c <= 0;
            frame_done <= 0; target_valid <= 0;
            target_x <= 0; target_y <= 0; red_count <= 0;
            confidence <= 0; reject_flags <= 0;
            bbox_x_min <= 0; bbox_x_max <= 0; bbox_y_min <= 0; bbox_y_max <= 0;
        end else begin
            frame_done   <= 1'b0;
            target_valid <= 1'b0;

            // Accumulators restart after each frame_end; the frame_end pixel is counted.
            if (frame_end) begin
                sum_x <= 0; sum_y <= 0; count <= 0;
                min_x <= 10'h3FF; max_x <= 0; min_y <= 10'h3FF; max_y <= 0;
            end else begin
                sum_x <= next_x; sum_y <= next_y; count <= next_count;
                min_x <= next_min_x; max_x <= next_max_x;
                min_y <= next_min_y; max_y <= next_max_y;
            end

            case (state)
                IDLE: if (frame_end) begin
                    // A frame_end arriving mid-calculation is ignored (frames are far
                    // longer than the 34-cycle pipeline).
                    frame_count <= next_count;
                    frame_sum_x <= next_x;
                    frame_sum_y <= next_y;
                    frame_min_x <= next_min_x; frame_max_x <= next_max_x;
                    frame_min_y <= next_min_y; frame_max_y <= next_max_y;
                    state <= BOX;
                end

                BOX: begin
                    if (frame_count == 0) begin
                        box_w <= 0;
                        box_h <= 0;
                    end else begin
                        box_w <= {1'b0, frame_max_x} - {1'b0, frame_min_x} + 11'd1;
                        box_h <= {1'b0, frame_max_y} - {1'b0, frame_min_y} + 11'd1;
                    end
                    state <= GATE;
                end

                GATE: begin
                    flags[0] <= (frame_count < MIN_PIXELS) || (frame_count == 0);
                    flags[1] <= (frame_count > MAX_PIXELS);
                    flags[2] <= ({5'd0, box_w} > w_limit) || ({5'd0, box_h} > h_limit);

                    den_xy <= (frame_count == 0) ? 32'd1 : frame_count;
                    den_c  <= (box_area == 0) ? 32'd1 : {10'd0, box_area};
                    dvd_x  <= frame_sum_x;
                    dvd_y  <= frame_sum_y;
                    dvd_c  <= (frame_count > 32'h00FF_FFFF) ? 32'hFFFF_FF00
                                                           : {frame_count[23:0], 8'd0};
                    quo_x <= 0; quo_y <= 0; quo_c <= 0;
                    rem_x <= 0; rem_y <= 0; rem_c <= 0;
                    bits_left <= 6'd32;
                    state <= DIVIDE;
                end

                DIVIDE: begin
                    dvd_x <= {dvd_x[30:0], 1'b0};
                    dvd_y <= {dvd_y[30:0], 1'b0};
                    dvd_c <= {dvd_c[30:0], 1'b0};
                    rem_x <= bit_x ? (trial_x - {1'b0, den_xy}) : trial_x;
                    rem_y <= bit_y ? (trial_y - {1'b0, den_xy}) : trial_y;
                    rem_c <= bit_c ? (trial_c - {1'b0, den_c})  : trial_c;
                    quo_x <= new_quo_x;
                    quo_y <= new_quo_y;
                    quo_c <= new_quo_c;
                    bits_left <= bits_left - 6'd1;

                    if (bits_left == 6'd1) begin
                        // Final results, all registered on the same edge.
                        frame_done   <= 1'b1;
                        target_valid <= accept;
                        target_x     <= accept ? new_quo_x[9:0] : 10'd0;
                        target_y     <= accept ? new_quo_y[9:0] : 10'd0;
                        red_count    <= frame_count;
                        confidence   <= conf_sat;
                        reject_flags <= {low_fill, flags};
                        bbox_x_min   <= (frame_count == 0) ? 10'd0 : frame_min_x;
                        bbox_x_max   <= (frame_count == 0) ? 10'd0 : frame_max_x;
                        bbox_y_min   <= (frame_count == 0) ? 10'd0 : frame_min_y;
                        bbox_y_max   <= (frame_count == 0) ? 10'd0 : frame_max_y;
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
