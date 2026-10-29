`default_nettype none
// V3-D9C stage 2B EXPERIMENT: frame-buffer consumer using sequential 8-neighbour BFS.
// Single-clock interface; run only with a FULL, immutable frame buffer.
// Uses M10K-style visited bitmap and BFS address queue. Not integrated into robot.
module temporal_ccl_sequential #(
    parameter integer WIDTH=320, HEIGHT=240, MIN_PIXELS=5
)(
    input wire clk, rst_n, invalidate,
    input wire frame_ready, frame_error,
    output logic read_en,
    output logic [((WIDTH*HEIGHT<=1)?1:$clog2(WIDTH*HEIGHT))-1:0] read_addr,
    input wire read_valid, read_hit,
    output logic release_frame, busy, result_valid, target_valid,
    output logic [((WIDTH<=1)?1:$clog2(WIDTH))-1:0] target_x,
    output logic [((HEIGHT<=1)?1:$clog2(HEIGHT))-1:0] target_y,
    output logic [((WIDTH*HEIGHT<=1)?1:$clog2(WIDTH*HEIGHT+1))-1:0] blob_pixels
);
    localparam integer N=WIDTH*HEIGHT;
    localparam integer AW=(N<=1)?1:$clog2(N);
    localparam integer CW=(N<=1)?1:$clog2(N+1);
    localparam integer XW=(WIDTH<=1)?1:$clog2(WIDTH);
    localparam integer YW=(HEIGHT<=1)?1:$clog2(HEIGHT);
    // Coordinates fit in 17 bits at 320x240, matching the previous queue width.
    localparam integer QW=(AW>XW+YW)?AW:(XW+YW);
    localparam integer SXW=(N*WIDTH<=1)?1:$clog2(longint'(N)*WIDTH+1);
    localparam integer SYW=(N*HEIGHT<=1)?1:$clog2(longint'(N)*HEIGHT+1);
    typedef enum logic [3:0] {
        IDLE, CLEAR, SCAN_REQ, SCAN_EVAL, POP_REQ, POP_EVAL,
        NEIGH_REQ, NEIGH_EVAL, COMPLETE, DONE, DIVIDE
    } state_t;
    state_t state;
    (* ramstyle="M10K" *) logic visited [0:N-1];
    (* ramstyle="M10K" *) logic [QW-1:0] queue_mem [0:N-1];
    logic visited_q;
    logic [QW-1:0] queue_q;
    logic [AW-1:0] clear_addr, scan_addr, neighbour_addr;
    logic [CW-1:0] head, tail, component_count, best_count;
    logic [SXW-1:0] component_sx, best_sx;
    logic [SYW-1:0] component_sy, best_sy;
    logic [3:0] neighbour_index;
    logic [XW-1:0] scan_x;
    logic [YW-1:0] scan_y;
    logic [XW-1:0] centre_x;
    logic [YW-1:0] centre_y;
    logic [XW-1:0] query_x;
    logic [YW-1:0] query_y;
    logic neighbour_in_bounds;
    logic [AW-1:0] neighbour_candidate;
    logic [AW-1:0] lookup_addr;
    logic lookup_en, visit_we, queue_we;
    logic [AW-1:0] visit_waddr, queue_waddr, queue_raddr;
    logic [QW-1:0] queue_wdata;
    logic queue_re;
    // Restoring unsigned centroid divider: two dimensions in parallel, one bit/clock.
    // Arithmetic is moved out of COMPLETE to avoid a long combinational / path.
    localparam integer DW=(SXW>SYW)?SXW:SYW;
    localparam integer DCW=(DW<=1)?1:$clog2(DW+1);
    logic [DW-1:0] div_x_shift, div_y_shift;
    logic [DW-1:0] div_x_quot, div_y_quot;
    logic [DW:0] div_x_rem, div_y_rem;
    logic [DW:0] div_den;
    logic [DCW-1:0] div_steps;
    wire [DW:0] div_x_trial={div_x_rem[DW-1:0],div_x_shift[DW-1]};
    wire [DW:0] div_y_trial={div_y_rem[DW-1:0],div_y_shift[DW-1]};
    wire div_x_ge=(div_x_trial>=div_den);
    wire div_y_ge=(div_y_trial>=div_den);
    wire [DW:0] div_x_next_rem=div_x_ge?(div_x_trial-div_den):div_x_trial;
    wire [DW:0] div_y_next_rem=div_y_ge?(div_y_trial-div_den):div_y_trial;
    wire [DW-1:0] div_x_next_quot={div_x_quot[DW-2:0],div_x_ge};
    wire [DW-1:0] div_y_next_quot={div_y_quot[DW-2:0],div_y_ge};
    // Queue and visited memories have synchronous read ports and no mass reset.
    always_ff @(posedge clk) begin
        if (lookup_en) visited_q <= visited[lookup_addr];
        if (visit_we) visited[visit_waddr] <= (state != CLEAR);
        if (queue_we) queue_mem[queue_waddr] <= queue_wdata;
        if (queue_re) queue_q <= queue_mem[queue_raddr];
    end
    // Calculate one of eight neighbours at a time, with signed bound checks.
    always_comb begin : neighbour_calc
        integer nx, ny;
        nx = int'(centre_x);
        ny = int'(centre_y);
        case (neighbour_index)
            0: begin nx=nx-1; ny=ny-1; end
            1: begin ny=ny-1; end
            2: begin nx=nx+1; ny=ny-1; end
            3: begin nx=nx-1; end
            4: begin nx=nx+1; end
            5: begin nx=nx-1; ny=ny+1; end
            6: begin ny=ny+1; end
            default: begin nx=nx+1; ny=ny+1; end
        endcase
        neighbour_in_bounds=(nx>=0 && nx<WIDTH && ny>=0 && ny<HEIGHT);
        neighbour_candidate=AW'(ny*WIDTH+nx);
        query_x=XW'(nx);
        query_y=YW'(ny);
    end
    always_comb begin
        read_en=0; read_addr='0; lookup_en=0; lookup_addr='0;
        visit_we=0; visit_waddr='0; queue_we=0; queue_waddr='0;
        queue_re=0; queue_raddr='0; queue_wdata='0;
        release_frame=0;
        case (state)
            CLEAR: begin visit_we=1; visit_waddr=clear_addr; end
            SCAN_REQ: begin
                read_en=1; read_addr=scan_addr;
                lookup_en=1; lookup_addr=scan_addr;
            end
            POP_REQ: begin queue_re=1; queue_raddr=AW'(head); end
            NEIGH_REQ: if (neighbour_in_bounds) begin
                read_en=1; read_addr=neighbour_candidate;
                lookup_en=1; lookup_addr=neighbour_candidate;
            end
            SCAN_EVAL: if (read_valid && read_hit && !visited_q) begin
                visit_we=1; visit_waddr=scan_addr;
                queue_we=1; queue_waddr='0; queue_wdata=QW'({scan_y,scan_x});
            end
            NEIGH_EVAL: if (read_valid && read_hit && !visited_q) begin
                visit_we=1; visit_waddr=neighbour_addr;
                queue_we=1; queue_waddr=AW'(tail); queue_wdata=QW'({query_y,query_x});
            end
            DONE: release_frame=1;
            default: ;
        endcase
    end
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; clear_addr<=0; scan_addr<=0; scan_x<=0; scan_y<=0;
            neighbour_addr<=0; head<=0; tail<=0; neighbour_index<=0;
            component_count<=0; component_sx<=0; component_sy<=0;
            best_count<=0; best_sx<=0; best_sy<=0;
            div_x_shift<='0; div_y_shift<='0; div_x_quot<='0; div_y_quot<='0;
            div_x_rem<='0; div_y_rem<='0; div_den<='0; div_steps<='0;
            centre_x<=0; centre_y<=0;
            busy<=0; result_valid<=0; target_valid<=0;
            target_x<=0; target_y<=0; blob_pixels<=0;
        end else if (invalidate) begin
            state<=IDLE; busy<=0; result_valid<=0; target_valid<=0;
            target_x<=0; target_y<=0; blob_pixels<=0;
            clear_addr<=0; scan_addr<=0; scan_x<=0; scan_y<=0; head<=0; tail<=0;
        end else begin
            result_valid<=0;
            case (state)
                IDLE: if (frame_ready) begin
                    busy<=1; target_valid<=0; blob_pixels<=0;
                    clear_addr<=0; scan_addr<=0; scan_x<=0; scan_y<=0;
                    // Reset queue ownership before the next frame's first seed write.
                    // Otherwise SCAN_EVAL writes at the previous frame's tail,
                    // while the subsequent POP_REQ reads queue_mem[0].
                    head<=0; tail<=0;
                    best_count<=0; best_sx<=0; best_sy<=0;
                    state<=CLEAR;
                end
                CLEAR: if (clear_addr == AW'(N-1)) state<=SCAN_REQ;
                       else clear_addr<=clear_addr+AW'(1);
                SCAN_REQ: state<=SCAN_EVAL;
                SCAN_EVAL: if (read_valid) begin
                    if (read_hit && !visited_q) begin
                        component_count<=CW'(1);
                        component_sx<=SXW'(scan_x);
                        component_sy<=SYW'(scan_y);
                        head<=0; tail<=CW'(1);
                        state<=POP_REQ;
                    end else if (scan_addr == AW'(N-1)) state<=COMPLETE;
                    else begin
                        scan_addr<=scan_addr+AW'(1);
                        if (scan_x == XW'(WIDTH-1)) begin
                            scan_x<='0;
                            scan_y<=scan_y+YW'(1);
                        end else scan_x<=scan_x+XW'(1);
                        state<=SCAN_REQ;
                    end
                end
                POP_REQ: if (head==tail) begin
                    if (component_count>best_count) begin
                        best_count<=component_count;
                        best_sx<=component_sx; best_sy<=component_sy;
                    end
                    if (scan_addr==AW'(N-1)) state<=COMPLETE;
                    else begin
                        scan_addr<=scan_addr+AW'(1);
                        if (scan_x == XW'(WIDTH-1)) begin
                            scan_x<='0;
                            scan_y<=scan_y+YW'(1);
                        end else scan_x<=scan_x+XW'(1);
                        state<=SCAN_REQ;
                    end
                end else state<=POP_EVAL;
                POP_EVAL: begin
                    // Queue holds pixel coordinates directly: no division/modulo.
                    centre_x<=queue_q[XW-1:0];
                    centre_y<=queue_q[XW +: YW];
                    head<=head+CW'(1);
                    neighbour_index<=0;
                    state<=NEIGH_REQ;
                end
                NEIGH_REQ: begin
                    neighbour_addr<=neighbour_candidate;
                    if (neighbour_in_bounds) state<=NEIGH_EVAL;
                    else if (neighbour_index==7) state<=POP_REQ;
                    else neighbour_index<=neighbour_index+4'd1;
                end
                NEIGH_EVAL: if (read_valid) begin
                    if (read_hit && !visited_q) begin
                        tail<=tail+CW'(1);
                        component_count<=component_count+CW'(1);
                        component_sx<=component_sx+SXW'(query_x);
                        component_sy<=component_sy+SYW'(query_y);
                    end
                    if (neighbour_index==7) state<=POP_REQ;
                    else begin neighbour_index<=neighbour_index+4'd1; state<=NEIGH_REQ; end
                end
                COMPLETE: begin
                    if (frame_error || best_count<CW'(MIN_PIXELS)) begin
                        result_valid<=1; busy<=0; target_valid<=0;
                        blob_pixels<='0; target_x<='0; target_y<='0;
                        state<=DONE;
                    end else begin
                        // Initialise two restoring dividers for best_sx/count
                        // and best_sy/count. The denominator is nonzero here.
                        div_x_shift<=DW'(best_sx);
                        div_y_shift<=DW'(best_sy);
                        div_x_quot<='0; div_y_quot<='0;
                        div_x_rem<='0; div_y_rem<='0;
                        div_den<=(DW+1)'(best_count);
                        div_steps<=DCW'(DW);
                        state<=DIVIDE;
                    end
                end
                DIVIDE: begin
                    div_x_shift<={div_x_shift[DW-2:0],1'b0};
                    div_y_shift<={div_y_shift[DW-2:0],1'b0};
                    div_x_rem<=div_x_next_rem;
                    div_y_rem<=div_y_next_rem;
                    div_x_quot<=div_x_next_quot;
                    div_y_quot<=div_y_next_quot;
                    div_steps<=div_steps-DCW'(1);
                    if (div_steps==DCW'(1)) begin
                        target_x<=XW'(div_x_next_quot);
                        target_y<=YW'(div_y_next_quot);
                        target_valid<=1;
                        blob_pixels<=best_count;
                        result_valid<=1; busy<=0;
                        state<=DONE;
                    end
                end
                DONE: state<=IDLE;
                default: state<=IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
