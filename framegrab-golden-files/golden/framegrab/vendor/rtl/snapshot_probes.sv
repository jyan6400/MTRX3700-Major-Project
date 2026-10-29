/*
 *  snapshot_probes.sv -- In-System Sources and Probes: a few control bits from the PC, status bits back, over
 *  the USB-Blaster's JTAG. No extra hardware and no Platform Designer system.
 *
 *  On the board this is Intel's altsource_probe (the IP behind Tools > In-System Sources and Probes Editor).
 *  The PC writes `source` and reads `probe` with quartus_stp (tcl/grab_jtag.tcl: write_source_data,
 *  read_probe_data). The IP re-times the sources into `clk` with two flip-flops (enable_metastability), so
 *  every source bit is in the `clk` domain. The probes are sampled by the JTAG clock whenever the PC reads them,
 *  so a probe word that is changing can be read half old and half new: the PC reads twice and trusts a value
 *  only when both reads agree.
 *
 *  In simulation (Verilator) it is a stand-in with the same timing: the testbench writes `src_jtag` (as
 *  write_source_data would) and reads `probe` (as read_probe_data would).
 */
module snapshot_probes #(
    parameter string ID = "NONE",      // the instance name quartus_stp lists (up to four characters)
    parameter int    SW = 1,           // source bits (PC -> design)
    parameter int    PW = 8            // probe bits (design -> PC)
) (
    input  logic          clk,         // the clock the sources are re-timed into
    input  logic [PW-1:0] probe,
    output logic [SW-1:0] source
);
`ifdef VERILATOR
    logic [SW-1:0] src_jtag = '0;      // written by the testbench
    logic [SW-1:0] s1 = '0, s2 = '0;
    always_ff @(posedge clk) begin s1 <= src_jtag; s2 <= s1; end
    assign source = s2;
`else
    altsource_probe #(
        .lpm_type("altsource_probe"), .lpm_hint("UNUSED"),
        .sld_auto_instance_index("YES"), .sld_instance_index(0),
        .instance_id(ID), .probe_width(PW), .source_width(SW),
        .source_initial_value("0"), .enable_metastability("YES")
    ) issp_u (
        .probe(probe), .source(source), .source_clk(clk), .source_ena(1'b1));
`endif
endmodule
