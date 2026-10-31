/*
 *  snapshot_ram.sv -- a memory that the design writes and reads, and that the PC reads over JTAG through the
 *  USB-Blaster: the frame grab's snapshot (frame_snap.sv).
 *
 *  On the board this is Intel's altsyncram with "ENABLE_RUNTIME_MOD=YES" in its lpm_hint: the IP Parameter
 *  Editor's tick box "Allow In-System Memory Content Editor to capture and update content independently of the
 *  system clock". Quartus then gives the memory's second port to the JTAG hub, so the PC reads it (Tools >
 *  In-System Memory Content Editor, or quartus_stp's read_content_from_memory) while the design keeps the first
 *  port. INSTANCE_NAME in HINT (up to four characters) is the name the editor and tcl/grab_jtag.tcl show.
 *
 *  It cannot be inferred from an array: the runtime-modification option exists only on the IP. Reads take one
 *  clock: q is the word at the address of the clock before.
 *
 *  In simulation (Verilator) it is a plain array, `mem`, which a testbench reads as the PC would.
 */
module snapshot_ram #(
    parameter int    WIDTH = 16,
    parameter int    DEPTH = 1024,
    parameter int    AW    = 10,
    parameter string HINT  = "ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=GRAB"
) (
    input  logic             clk,
    input  logic             we,
    input  logic [AW-1:0]    addr,
    input  logic [WIDTH-1:0] data,
    output logic [WIDTH-1:0] q
);
`ifdef VERILATOR
    logic [WIDTH-1:0] mem [0:DEPTH-1];
    initial for (int i = 0; i < DEPTH; i++) mem[i] = '0;           // the board's power-up value
    always_ff @(posedge clk) begin
        if (we && int'(addr) < DEPTH) mem[addr] <= data;
        q <= (int'(addr) < DEPTH) ? mem[addr] : '0;
    end
`else
    altsyncram #(
        .operation_mode("SINGLE_PORT"),
        .width_a(WIDTH), .widthad_a(AW), .numwords_a(DEPTH),
        .lpm_type("altsyncram"), .lpm_hint(HINT),
        .intended_device_family("Cyclone V"), .ram_block_type("M10K"),
        .outdata_reg_a("UNREGISTERED"), .outdata_aclr_a("NONE"),
        .clock_enable_input_a("BYPASS"), .clock_enable_output_a("BYPASS"),
        .power_up_uninitialized("FALSE"), .read_during_write_mode_port_a("NEW_DATA_NO_NBE_READ"),
        .width_byteena_a(1)
    ) ram_u (
        .clock0(clk), .address_a(addr), .data_a(data), .wren_a(we), .q_a(q),
        .aclr0(1'b0), .aclr1(1'b0), .address_b(1'b1), .addressstall_a(1'b0), .addressstall_b(1'b0),
        .byteena_a(1'b1), .byteena_b(1'b1), .clock1(1'b1), .clocken0(1'b1), .clocken1(1'b1),
        .clocken2(1'b1), .clocken3(1'b1), .data_b(1'b1), .eccstatus(), .q_b(), .rden_a(1'b1),
        .rden_b(1'b1), .wren_b(1'b0));
`endif
endmodule
