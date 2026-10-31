# vo_grab.sdc -- Group 26: vision_grab.sdc (FrameGrabJtag V2026.3's constraints) for vo_grab_top.sv, plus the HEX
# displays (static: false paths) at the end. beacon_vo runs wholly in PCLK here; its mailbox CDC sees one clock.
# The instance names it uses (video_pll_u, sccb_clk_u) are the same in vision_grab_top. What vision_grab_top adds:
#   - the robot's vision chain and vision_grab_results: all in PCLK, already constrained (PCLK at 40 MHz, 50 fps);
#   - SW1's two-flop synchroniser into PCLK: SW[*] is a false path below;
#   - the target cross for the VGA (PCLK -> clk_25 through tgt_s1/tgt_s2): covered by the asynchronous clock groups;
#   - the resets from KEY0 and the PLL's lock, asserted at once and released through two flops in each domain.
#
# FrameGrabJtag.sdc -- the camera project's Camera.sdc, plus the JTAG pins at the end.
#
# Camera.sdc -- timing constraints for the OV7670 camera project (DE1-SoC, Quartus 18.1).
#
# Five clocks: the 50 MHz oscillator, the 25 MHz pixel clock and the 10 MHz camera XCLK the PLL
# makes from it, the camera's own PCLK (20 MHz at the default 25 fps) coming back in on GPIO_1[21], and the 100 kHz SCCB
# clock from a counter (below). PCLK is NOT a PLL
# clock: XCLK went out through 33 R and a ribbon, and the sensor's own PLL multiplied it, so PCLK
# is a separate, unrelated clock. The Lesson 4 rule applies: every path between two domains is either a
# proper crossing (the frame buffer's dual-clock RAM) or a false path.

create_clock -name CLOCK_50 -period 20.000 [get_ports CLOCK_50]
create_clock -name PCLK     -period 25.000 [get_ports {GPIO_1[21]}]
# PCLK is constrained at 40 MHz, the faster of the camera code's two rates (CAMERA_FPS = 50), so the analysis covers
# both; at the default, 25 frames a second (20 MHz), the margins are larger than Quartus reports.
derive_pll_clocks
derive_clock_uncertainty

# The 100 kHz SCCB clock comes out of a counter (clock_divider.sv), 250 x slower than the PLL
# clock; declare it so that the I2C master's paths are checked (as in Lesson 3's mic-input).
create_generated_clock -name sccb_clk -source [get_pins {video_pll_u|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}] \
    -divide_by 250 [get_registers {clock_divider:sccb_clk_u|clk_out}]

# The camera drives D[7:0], HREF and VSYNC up to 5 ns after the FALLING edge of PCLK (datasheet Table 4), plus
# about 2 ns of margin for skew between the ribbon's wires and the DE1-SoC's 47 R + clamp on each pin. PCLK comes
# down the same ribbon, so a longer ribbon delays the clock and the data alike and does not change these numbers. The capture latches them on the RISING edge, 12.5 ns after the fall at 40 MHz: 5.5 ns
# of margin outside the chip, and the capture flops sit in the I/O cells (FAST_INPUT_REGISTER in the .qsf) so the
# path inside is as short as it can be. At a 100 MHz PCLK the same numbers would leave none, even on paper.
set_input_delay -clock PCLK -clock_fall -max 7.0 [get_ports {GPIO_1[12] GPIO_1[13] GPIO_1[14] GPIO_1[15] GPIO_1[16] GPIO_1[17] GPIO_1[18] GPIO_1[19] GPIO_1[22] GPIO_1[23]}]
set_input_delay -clock PCLK -clock_fall -min 0.0 [get_ports {GPIO_1[12] GPIO_1[13] GPIO_1[14] GPIO_1[15] GPIO_1[16] GPIO_1[17] GPIO_1[18] GPIO_1[19] GPIO_1[22] GPIO_1[23]}]

# Domains that only meet through the frame buffer RAM (PCLK -> pixel clock), through synchronisers
# (KEY0's reset) or not at all (the frame toggle only drives an LED): asynchronous.
set_clock_groups -asynchronous -group [get_clocks CLOCK_50] -group [get_clocks PCLK] \
    -group [get_clocks {*video_pll* sccb_clk}]

# Slow or static I/O: the buttons and switches, the LEDs, the camera's control pins and XCLK, the VGA pins
# (the VGA Controller registers them at the pixel clock and the DAC tolerates the skew).
set_false_path -from [get_ports {KEY[*] SW[*]}]
set_false_path -to   [get_ports {LEDR[*] GPIO_1[10] GPIO_1[11] GPIO_1[20] GPIO_1[24] GPIO_1[25]}]
set_false_path -from [get_ports {GPIO_1[24]}]
set_false_path -to   [get_ports {VGA_*}]

# The JTAG frame grab (In-System Sources and Probes, In-System Memory Content Editor): Quartus's JTAG hub runs on the
# USB-Blaster's TCK, which is unrelated to every other clock. The hub constrains that clock itself (its own SDC
# creates altera_reserved_tck at 30 MHz, in a clock group of its own: the snapshot memory and the probes cross into
# it inside Intel's IP), so it is not created again here. The JTAG pins are timed by the USB-Blaster, not by this
# design: TDI and TMS are cut, and TDO too in the timing analysis (the fitter keeps the hub's own rule for TDO).
if {[get_collection_size [get_ports -nowarn {altera_reserved_tck}]] > 0} {
    set_false_path -from [get_ports {altera_reserved_tdi altera_reserved_tms}]
    if {![string equal quartus_fit $::TimeQuestInfo(nameofexecutable)]} {
        set_false_path -to [get_ports {altera_reserved_tdo}]
    }
}

# the 7-segment displays change once a frame
set_false_path -to [get_ports {HEX0[*] HEX1[*] HEX2[*] HEX3[*] HEX4[*] HEX5[*]}]
