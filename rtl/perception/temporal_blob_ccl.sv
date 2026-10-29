`default_nettype none
// V3-D4 experimental streaming 8-connected-component labeller.
// Pixel positions must arrive in raster order, with a hit_valid pulse for
// EVERY pixel (including hit==0); frame_end accompanies the final pixel.
// Connected components are labelled online; statistics are reduced after frame_end.
// If the fixed label budget is exhausted the entire frame is rejected safely.
// No motor outputs. Not integrated with the OV7670 top level.
module temporal_blob_ccl #(
    parameter int unsigned WIDTH = 320,
    parameter int unsigned HEIGHT = 240,
    parameter int unsigned MAX_LABELS = 256,
    parameter int unsigned MIN_PIXELS = 5
) (
    input wire clk, rst_n, invalidate,
    input wire hit_valid, hit, frame_start, frame_end,
    input wire [((WIDTH<=1)?1:$clog2(WIDTH))-1:0] hit_x,
    input wire [((HEIGHT<=1)?1:$clog2(HEIGHT))-1:0] hit_y,
    output logic result_valid, target_valid, frame_error,
    output logic [((WIDTH<=1)?1:$clog2(WIDTH))-1:0] target_x,
    output logic [((HEIGHT<=1)?1:$clog2(HEIGHT))-1:0] target_y,
    output logic [31:0] blob_pixels,
    output logic busy
);
    localparam int XW=(WIDTH<=1)?1:$clog2(WIDTH);
    localparam int YW=(HEIGHT<=1)?1:$clog2(HEIGHT);
    // Lossless widths for a full WIDTH x HEIGHT image. Use longint for
    // elaboration calculations to avoid integer overflow for large parameters.
    localparam int CW = $clog2(longint'(WIDTH)*HEIGHT + 1);
    localparam int SXW = (WIDTH<=1) ? 1 : $clog2(longint'(WIDTH)*HEIGHT*(WIDTH-1) + 1);
    localparam int SYW = (HEIGHT<=1) ? 1 : $clog2(longint'(WIDTH)*HEIGHT*(HEIGHT-1) + 1);
    localparam int LW=$clog2(MAX_LABELS+1);
    localparam int IDXW=(MAX_LABELS<=2)?1:$clog2(MAX_LABELS+1);
    typedef enum logic [1:0] {IDLE, CAPTURE, REDUCE, FINISH} state_t;
    state_t state;
    // Two row buffers, selected by the parity of the current row.
    logic [LW-1:0] row0 [0:WIDTH-1];
    logic [LW-1:0] row1 [0:WIDTH-1];
    logic [LW-1:0] parent [0:MAX_LABELS];
    logic [CW-1:0] count [0:MAX_LABELS];
    logic [SXW-1:0] sx [0:MAX_LABELS];
    logic [SYW-1:0] sy [0:MAX_LABELS];
    logic [CW-1:0] merged_count [0:MAX_LABELS];
    logic [SXW-1:0] merged_sx [0:MAX_LABELS];
    logic [SYW-1:0] merged_sy [0:MAX_LABELS];
    logic [LW-1:0] next_label;
    logic [LW-1:0] scan;
    logic [XW-1:0] expected_x;
    logic [YW-1:0] expected_y;
    logic bad;
    logic [LW-1:0] a,b,c,d, root_a,root_b,root_c,root_d, chosen;
    logic [LW-1:0] root_scan;
    logic [CW-1:0] best_count;
    logic [SXW-1:0] best_sx;
    logic [SYW-1:0] best_sy;
    logic [LW-1:0] best_label;
    logic [LW-1:0] selected_label;
    logic [CW-1:0] total_count;
    logic [SXW-1:0] total_sx;
    logic [SYW-1:0] total_sy;

    // All non-zero parent entries are maintained as DIRECT canonical roots.
    // This avoids the original MAX_LABELS-stage combinational root traversal.
    // Union operations below flatten existing aliases on the same clock edge.
    function automatic logic [LW-1:0] root(input logic [LW-1:0] label);
        begin
            root = (label == '0) ? '0 : parent[label];
        end
    endfunction

    always_comb begin
        a='0; b='0; c='0; d='0;
        // left neighbour is the current row (written on prior cycle).
        if (hit_x>0) a=hit_y[0] ? row1[hit_x-1'b1] : row0[hit_x-1'b1];
        // three neighbours from previous row.
        if (hit_y>0) begin
            if (hit_x>0) b=hit_y[0] ? row0[hit_x-1'b1] : row1[hit_x-1'b1];
            c=hit_y[0] ? row0[hit_x] : row1[hit_x];
            if (hit_x<WIDTH-1) d=hit_y[0] ? row0[hit_x+1'b1] : row1[hit_x+1'b1];
        end
        root_a=root(a); root_b=root(b); root_c=root(c); root_d=root(d);
        chosen='0;
        if (root_a!=0) chosen=root_a;
        if (root_b!=0 && (chosen==0 || root_b<chosen)) chosen=root_b;
        if (root_c!=0 && (chosen==0 || root_c<chosen)) chosen=root_c;
        if (root_d!=0 && (chosen==0 || root_d<chosen)) chosen=root_d;
        root_scan=root(scan);
        selected_label=(chosen!=0)?chosen:next_label;
        total_count=merged_count[scan];
        total_sx=merged_sx[scan];
        total_sy=merged_sy[scan];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; busy<=0; result_valid<=0; target_valid<=0;
            frame_error<=0; target_x<='0; target_y<='0; blob_pixels<=0;
            next_label<=1; expected_x<=0; expected_y<=0;
            scan<=0; best_label<=0; best_count<=0; best_sx<=0; best_sy<=0; bad<=0;
            for (int i=0;i<WIDTH;i++) begin row0[i]<=0; row1[i]<=0; end
            for (int i=0;i<=MAX_LABELS;i++) begin
                parent[i]<=LW'(i); count[i]<=0; sx[i]<=0; sy[i]<=0;
                merged_count[i]<=0; merged_sx[i]<=0; merged_sy[i]<=0;
            end
        end else begin
            result_valid<=0;
            if (invalidate) begin
                state<=IDLE; busy<=0; target_valid<=0; frame_error<=1;
                next_label<=1; bad<=0;
            end else begin
                case (state)
                IDLE: begin
                    busy<=0;
                    if (hit_valid && frame_start && hit_x==0 && hit_y==0) begin
                        state<=CAPTURE; busy<=1; bad<=0; frame_error<=0; target_valid<=0;
                        expected_x<= XW'(1); expected_y<=0; next_label<=1;
                        for (int i=0;i<WIDTH;i++) begin row0[i]<=0; row1[i]<=0; end
                        for (int i=0;i<=MAX_LABELS;i++) begin
                            parent[i]<=LW'(i); count[i]<=0; sx[i]<=0; sy[i]<=0;
                            merged_count[i]<=0; merged_sx[i]<=0; merged_sy[i]<=0;
                        end
                        if (hit) begin
                            row0[0]<=LW'(1); count[1]<=1; next_label<=2;
                        end
                    end
                end
                CAPTURE: if (hit_valid) begin
                    if (frame_start || hit_x!=expected_x || hit_y!=expected_y) bad<=1;
                    if (hit_y[0]) row1[hit_x] <= hit ? selected_label : '0;
                    else row0[hit_x] <= hit ? selected_label : '0;
                    if (hit) begin
                        if (chosen==0) begin
                            if (next_label>MAX_LABELS) bad<=1;
                            else begin
                                parent[next_label]<=next_label;
                                count[next_label]<=1;
                                sx[next_label]<=32'(hit_x); sy[next_label]<=32'(hit_y);
                                next_label<=next_label+1'b1;
                            end
                        end else begin
                            // Count each pixel under the original label chosen.
                            count[chosen]<=count[chosen]+1;
                            sx[chosen]<=sx[chosen]+32'(hit_x);
                            sy[chosen]<=sy[chosen]+32'(hit_y);
                            // Union neighbouring component roots. The final reduction
                            // resolves all transitive parent links.
                            // Repoint EVERY alias of a merged root to chosen.
                            // Invariant: parent[label] is always its direct root,
                            // so root() needs only a single indexed lookup.
                            // Use the old values on this edge for all comparisons.
                            for (int i=1; i<=MAX_LABELS; i++) begin
                                if ((root_a!=0 && parent[i]==root_a) ||
                                    (root_b!=0 && parent[i]==root_b) ||
                                    (root_c!=0 && parent[i]==root_c) ||
                                    (root_d!=0 && parent[i]==root_d)) begin
                                    parent[i] <= chosen;
                                end
                            end
                        end
                    end
                    if (expected_x==WIDTH-1) begin
                        expected_x<=0;
                        expected_y<=expected_y+1'b1;
                    end else expected_x<=expected_x+1'b1;
                    if (frame_end) begin
                        if (hit_x!=WIDTH-1 || hit_y!=HEIGHT-1 || bad || next_label>MAX_LABELS) begin
                            state<=FINISH; bad<=1;
                        end else begin
                            state<=REDUCE; scan<=1; best_count<=0; best_sx<=0;
                            best_sy<=0; best_label<=0;
                        end
                    end else if (hit_x==WIDTH-1 && hit_y==HEIGHT-1) begin
                        bad<=1; state<=FINISH;
                    end
                end
                REDUCE: begin
                    // Phase 1: Aggregate original labels into canonical roots.
                    if (scan<next_label) begin
                        merged_count[root_scan]<=merged_count[root_scan]+count[scan];
                        merged_sx[root_scan]<=merged_sx[root_scan]+sx[scan];
                        merged_sy[root_scan]<=merged_sy[root_scan]+sy[scan];
                        scan<=scan+1'b1;
                    end else begin
                        state<=FINISH; scan<=1;
                    end
                end
                FINISH: begin
                    // Phase 2: select the largest aggregated root, one per cycle.
                    if (!bad && scan<next_label) begin
                        if (merged_count[scan]>best_count) begin
                            best_count<=merged_count[scan]; best_sx<=merged_sx[scan];
                            best_sy<=merged_sy[scan]; best_label<=scan;
                        end
                        scan<=scan+1'b1;
                    end else begin
                        result_valid<=1; frame_error<=bad;
                        target_valid<=!bad && best_count>=MIN_PIXELS;
                        blob_pixels<=bad ? 0 : best_count;
                        if (!bad && best_count!=0) begin
                            target_x<=XW'(best_sx/best_count);
                            target_y<=YW'(best_sy/best_count);
                        end
                        state<=IDLE; busy<=0;
                    end
                end
                default: state<=IDLE;
                endcase
            end
        end
    end
endmodule
`default_nettype wire
