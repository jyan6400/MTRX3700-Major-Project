`default_nettype none
// V3-D9C stage 1 prototype. Standalone, NOT connected to robot top.
// Stores one complete 1-bit 320x240 hit frame for later sequential CCL.
// No backpressure: a new frame while the stored frame is owned causes overrun.
module temporal_hit_frame_buffer #(
    parameter integer WIDTH=320,
    parameter integer HEIGHT=240
)(
    input wire clk,
    input wire rst_n,
    input wire invalidate,
    input wire hit_valid,
    input wire hit,
    input wire frame_start,
    input wire frame_end,
    input wire [((WIDTH<=1)?1:$clog2(WIDTH))-1:0] hit_x,
    input wire [((HEIGHT<=1)?1:$clog2(HEIGHT))-1:0] hit_y,
    input wire release_frame,
    output logic frame_ready,
    output logic frame_error,
    output logic overrun,
    input wire read_en,
    input wire [((WIDTH*HEIGHT<=1)?1:$clog2(WIDTH*HEIGHT))-1:0] read_addr,
    output logic read_valid,
    output logic read_hit
);
    localparam integer XW=(WIDTH<=1)?1:$clog2(WIDTH);
    localparam integer YW=(HEIGHT<=1)?1:$clog2(HEIGHT);
    localparam integer N=WIDTH*HEIGHT;
    localparam integer AW=(N<=1)?1:$clog2(N);
    localparam integer CW=(N<=1)?1:$clog2(N+1);
    typedef enum logic [1:0] {EMPTY, WRITING, FULL} state_t;
    state_t state;
    // No reset of RAM data: allows M10K inference. Validity is controlled
    // exclusively by frame_ready; do not read until a complete frame exists.
    (* ramstyle = "M10K" *) logic bit_ram [0:N-1];
    logic [CW-1:0] write_index;
    logic bad_frame;
    wire [CW-1:0] linear_position = CW'(hit_y)*CW'(WIDTH)+CW'(hit_x);
    wire position_ok = (hit_x < WIDTH) && (hit_y < HEIGHT);

    always_ff @(posedge clk) begin
        read_valid <= read_en && frame_ready && !invalidate && !release_frame;
        if (read_en && frame_ready && !invalidate && !release_frame)
            read_hit <= bit_ram[read_addr];
        if (!invalidate && !release_frame && hit_valid && position_ok &&
            ((state==EMPTY && frame_start && linear_position==0) ||
             (state==WRITING && !frame_start && linear_position==write_index)))
            bit_ram[linear_position[AW-1:0]] <= hit;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=EMPTY; frame_ready<=0; frame_error<=0; overrun<=0;
            write_index<=0; bad_frame<=0;
        end else if (invalidate) begin
            state<=EMPTY; frame_ready<=0; frame_error<=0; overrun<=0;
            write_index<=0; bad_frame<=0;
        end else begin
            if (release_frame) begin
                state<=EMPTY; frame_ready<=0; frame_error<=0;
                bad_frame<=0; write_index<=0; overrun<=0;
            end else if (hit_valid) begin
                case (state)
                EMPTY: begin
                    if (frame_start && position_ok && linear_position==0) begin
                        frame_error<=0; bad_frame<=0; write_index<=CW'(1);
                        if (frame_end) begin
                            if (N==1) begin state<=FULL; frame_ready<=1; end
                            else begin state<=EMPTY; frame_error<=1; end
                        end else state<=WRITING;
                    end
                end
                WRITING: begin
                    if (frame_start || !position_ok || linear_position!=write_index) begin
                        bad_frame<=1;
                    end else begin
                        write_index<=write_index+CW'(1);
                    end
                    if (frame_end) begin
                        if (!bad_frame && !frame_start && position_ok &&
                            linear_position==CW'(N-1) && write_index==CW'(N-1)) begin
                            state<=FULL; frame_ready<=1; frame_error<=0;
                        end else begin
                            state<=EMPTY; frame_ready<=0; frame_error<=1;
                        end
                    end else if (frame_start || !position_ok ||
                                  linear_position!=write_index) begin
                        state<=EMPTY; frame_error<=1;
                    end
                end
                FULL: begin
                    if (frame_start) overrun<=1;
                end
                default: state<=EMPTY;
                endcase
            end
        end
    end
endmodule
`default_nettype wire
