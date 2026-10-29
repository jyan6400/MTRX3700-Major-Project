/*
 *  frame_snap.sv -- press KEY1 (or run tcl/grab_jtag.tcl on the PC): the next whole camera frame, and the numbers
 *  your design computed from that same frame, are kept in a memory that the PC reads over the USB-Blaster (JTAG).
 *  No extra cable and no extra hardware: the cable that programs the board brings the frame back.
 *
 *  Why: the Major Project asks for a golden model of a processing module and a "replay": data from your own
 *  robot fed to the golden model and to the simulation, and the answers compared. This is the data.
 *  tools/grab.py turns it into frame.png, frame.hex, a scene file the camera model can show (so the simulation
 *  sees the same picture) and results.json (the board's numbers).
 *
 *  THE MEMORY holds the snapshot as a run of bytes, one 16-bit word per two bytes (high byte first), which
 *  tools/grab.py decodes:
 *
 *      word         field
 *      0, 1         magic "GR" "AB" (0x4752 0x4142)
 *      2            version: 1
 *      3, 4         width W and height H of the stored picture
 *      5            pixel format: 1 = RGB565 {R[4:0], G[5:0], B[4:0]}; 2 = 8-bit grey, two pixels a word
 *                   (the left one in the high byte)
 *      6            N_RESULTS: the number of 32-bit result words
 *      7 ..         the result words, word 0 first, each high half first
 *      7 + 2N ..    the pixels, row by row from the top left
 *      last         CRC-16/CCITT-FALSE of every byte before it (Python: binascii.crc_hqx(data, 0xFFFF))
 *
 *  MODE sets the picture, and so the memory it takes on the FPGA (397 M10K blocks in all; the camera's frame
 *  buffer takes 150):
 *      0  320 x 240 RGB565, the frame buffer's own picture (every second pixel of every second row)   about 150 M10K
 *      1  160 x 120 RGB565 (every fourth pixel of every fourth row)                                   about 38 M10K
 *      2  320 x 240 grey, camera_pkg's (R + 2G + B) / 4, 8 bits a pixel                               about 75 M10K
 *
 *  THE STEPS (all in the camera's clock, pclk)
 *    1. TRIGGER. A rising edge of `trigger` (KEY1), or the PC flipping its request bit, while no snapshot is
 *       being taken. The memory's old snapshot is no longer "ready" from here on.
 *    2. CAPTURE. From the next frame boundary (frame_start: VSYNC has just fallen) every kept pixel of that frame
 *       is written to the memory, at its place in the picture.
 *    3. RESULTS. At the following frame boundary the frame is whole. At the next frame's first pixel `results`
 *       is copied (so a design that publishes its numbers at frame_start, as frame_stats.sv does, has
 *       them there), and the header and the result words are written.
 *    4. CRC. The memory is read back in order, two bytes a clock, and the CRC is written last. `ready` goes high
 *       and stays high (LEDR3) until the next trigger: the snapshot stays put however long the PC takes.
 *  A snapshot is ready 42 to 82 ms after the request at 25 frames a second, 22 to 42 ms at 50 (the rest of the frame
 *  in progress, the whole next one, then about 2 ms). Triggers during it are not lost:
 *  the PC's request waits until the snapshot is done; a key press then is ignored (key bounce does not matter).
 *
 *  THE PC'S SIDE, in snapshot_probes (In-System Sources and Probes, instance PROBE_NAME):
 *      source bit 0   the request: the PC flips it to ask for a snapshot
 *      probe          {count[15:0], ack, busy, ready, MODE[1:0]}: count of finished snapshots; ack = the request
 *                     bit's value at the last snapshot started (equal to the source bit: the request was taken)
 *  tcl/grab_jtag.tcl flips the request, waits for ready with the count moved on, reads the memory (instance
 *  GRAB) and runs tools/grab.py on it. Quartus's In-System Memory Content Editor reads the same memory by
 *  hand (after a KEY1 press), and tools/jtag_export.py decodes what it exports.
 *
 *  `results` must be in the pclk domain (where ov7670_capture's outputs are).
 */
module frame_snap #(
    parameter int    MODE       = 0,         // the picture kept: 0, 1 or 2 (above)
    parameter int    N_RESULTS  = 5,         // 32-bit result words (at least 1)
    parameter string PROBE_NAME = "GCTL"     // the probes' name in the In-System Sources and Probes Editor (up to 4 characters)
) (
    input  logic                    pclk,          // camera PCLK: ov7670_capture's clock
    input  logic                    reset,         // synchronous to pclk, active high
    input  logic                    frame_start,   // ov7670_capture's outputs
    input  logic                    px_valid,
    input  logic [15:0]             pixel,
    input  logic [9:0]              x,
    input  logic [8:0]              y,
    input  logic [32*N_RESULTS-1:0] results,       // your numbers, word 0 in the top bits: {word0, word1, ...}
    input  logic                    trigger,       // a rising edge asks for a snapshot (asynchronous: ~KEY[1] will do)
    output logic                    busy,          // a snapshot is being taken
    output logic                    ready          // a whole snapshot is in the memory (for an LED)
);
    import camera_pkg::*;

    localparam int W         = (MODE == 1) ? 160 : 320;
    localparam int H         = (MODE == 1) ? 120 : 240;
    localparam int FORMAT    = (MODE == 2) ? 2 : 1;
    localparam int ROW_WORDS = (MODE == 2) ? W / 2 : W;          // memory words a row of the picture takes
    localparam int PIX0      = 7 + 2 * N_RESULTS;                // the first pixel word
    localparam int CRC_ADDR  = PIX0 + ROW_WORDS * H;             // the CRC: the last word
    localparam int DEPTH     = CRC_ADDR + 1;
    localparam int AW        = $clog2(DEPTH);
    localparam logic [16*7-1:0] HEADER = {16'h4752, 16'h4142, 16'd1, 16'(W), 16'(H), 16'(FORMAT), 16'(N_RESULTS)};

    // CRC-16/CCITT-FALSE, one byte, most significant bit first
    function automatic logic [15:0] crc16_ccitt(input logic [15:0] c, input logic [7:0] b);
        logic [15:0] r;
        r = c ^ {b, 8'h00};
        for (int k = 0; k < 8; k++) r = r[15] ? ({r[14:0], 1'b0} ^ 16'h1021) : {r[14:0], 1'b0};
        return r;
    endfunction

    // ---- the memory and the PC's probes ----------------------------------------------------------------
    logic          ram_we;
    logic [AW-1:0] ram_addr;
    logic [15:0]   ram_data, ram_q;
    snapshot_ram #(.WIDTH(16), .DEPTH(DEPTH), .AW(AW), .HINT("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=GRAB")) ram_u (
        .clk(pclk), .we(ram_we), .addr(ram_addr), .data(ram_data), .q(ram_q));

    logic [15:0] count = '0;                     // snapshots finished since the FPGA was configured
    logic        pc_req, pc_ack = 1'b0;          // the PC's request bit (already re-timed into pclk), and the one taken
    snapshot_probes #(.ID(PROBE_NAME), .SW(1), .PW(21)) probes_u (
        .clk(pclk), .probe({count, pc_ack, busy, ready, 2'(MODE)}), .source(pc_req));

    // ---- 1. the trigger ----------------------------------------------------------------------------------
    logic [2:0] key_s = 3'b000;                  // trigger, synchronised (two flops) and one clock late, for the edge
    always_ff @(posedge pclk) key_s <= {key_s[1:0], trigger};
    wire key_edge = key_s[1] & ~key_s[2];

    // ---- the picture's kept pixels -------------------------------------------------------------------------
    logic        kept;                           // this pixel goes into the picture
    logic [8:0]  u;                              // its column and row in the picture
    logic [7:0]  v;
    logic [7:0]  grey, grey_left = '0;           // MODE 2: the pixel's grey, and the left one of the pair waiting
    always_comb begin
        if (MODE == 1) begin
            kept = px_valid && x[1:0] == 2'b00 && y[1:0] == 2'b00;
            u = 9'(x[9:2]);
            v = 8'(y[8:2]);
        end else begin
            kept = px_valid && !x[0] && !y[0];
            u = x[9:1];
            v = y[8:1];
        end
        kept = kept && (int'(u) < W) && (int'(v) < H);
        grey = rgb565_to_grey(pixel);
    end

    // ---- 2. to 4.: the snapshot ------------------------------------------------------------------------------
    enum logic [2:0] {S_IDLE, S_ARMED, S_CAPTURE, S_RESULTS, S_HEAD, S_CRC, S_CRC_WRITE} state = S_IDLE;
    logic [AW-1:0]           i = '0;             // S_HEAD: the word being written; S_CRC: the word being read
    logic                    folding = 1'b0;     // S_CRC: ram_q holds the word read on the clock before
    logic [15:0]             crc = 16'hFFFF;
    logic [32*N_RESULTS-1:0] results_q = '0;
    logic [16*PIX0-1:0]      head;
    assign head = {HEADER, results_q};
    logic                    ready_q = 1'b0;

    always_ff @(posedge pclk) begin
        if (reset) begin
            state <= S_IDLE;
            ready_q <= 1'b0;
            pc_ack <= pc_req;                    // a request from before the reset is not carried over
        end else begin
            case (state)
                S_IDLE:      if (key_edge || pc_req != pc_ack) begin                                  // 1.
                                 pc_ack <= pc_req;
                                 ready_q <= 1'b0;
                                 state  <= S_ARMED;
                             end
                S_ARMED:     if (frame_start) state <= S_CAPTURE;                                     // 2. a frame begins
                S_CAPTURE:   begin
                                 if (MODE == 2 && kept && !u[0]) grey_left <= grey;
                                 if (frame_start) state <= S_RESULTS;                                 // 3. it has ended
                             end
                S_RESULTS:   if (px_valid) begin                                                      //    the next one's first pixel
                                 results_q <= results;
                                 i <= '0;
                                 state <= S_HEAD;
                             end
                S_HEAD:      if (i == AW'(PIX0 - 1)) begin i <= '0; folding <= 1'b0; crc <= 16'hFFFF; state <= S_CRC; end
                             else i <= i + 1'b1;
                S_CRC:       begin                                                                    // 4.
                                 if (folding) crc <= crc16_ccitt(crc16_ccitt(crc, ram_q[15:8]), ram_q[7:0]);
                                 if (i == AW'(CRC_ADDR)) state <= S_CRC_WRITE;                        // the last word has been folded in
                                 else begin i <= i + 1'b1; folding <= 1'b1; end
                             end
                S_CRC_WRITE: begin ready_q <= 1'b1; count <= count + 1'b1; state <= S_IDLE; end
                default:     state <= S_IDLE;
            endcase
        end
    end
    assign busy  = (state != S_IDLE);
    assign ready = ready_q;

    // the memory's one port: the pixels (2.), the header and words (3.), the read-back and the CRC (4.)
    always_comb begin
        ram_we   = 1'b0;
        ram_addr = '0;
        ram_data = pixel;
        case (state)
            S_CAPTURE:   if (MODE == 2) begin
                             ram_we   = kept && u[0];                                     // the right pixel of a pair: write both
                             ram_addr = AW'(PIX0 + int'(v) * ROW_WORDS + int'(u[8:1]));
                             ram_data = {grey_left, grey};
                         end else begin
                             ram_we   = kept;
                             ram_addr = AW'(PIX0 + int'(v) * ROW_WORDS + int'(u));
                         end
            S_HEAD:      begin ram_we = 1'b1; ram_addr = i; ram_data = head[16 * (PIX0 - 1 - int'(i)) +: 16]; end
            S_CRC:       ram_addr = i;
            S_CRC_WRITE: begin ram_we = 1'b1; ram_addr = AW'(CRC_ADDR); ram_data = crc; end
            default:     ;
        endcase
    end
endmodule
