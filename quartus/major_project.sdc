# MTRX3700 Group 26 - V1 preliminary timing constraints
# These constraints are for initial synthesis/fitter analysis.
# Final timing sign-off requires generated-clock and CDC review.

# DE1-SoC system oscillator: 50 MHz
create_clock -name CLOCK_50 -period 20.000 [get_ports CLOCK_50]

# OV7670 PCLK: nominal 40 MHz
# Verify its actual operating frequency on hardware.
create_clock -name PCLK -period 25.000 [get_ports {GPIO_1[21]}]

# Recognise camera and sonar PLL output clocks
derive_pll_clocks
derive_clock_uncertainty

# Camera data and timing signals.
# Initial values follow Ed's OV7670 timing reference.
set_input_delay -clock PCLK -clock_fall -max 7.0 \
    [get_ports {GPIO_1[12] GPIO_1[13] GPIO_1[14] GPIO_1[15] GPIO_1[16] GPIO_1[17] GPIO_1[18] GPIO_1[19] GPIO_1[22] GPIO_1[23]}]

set_input_delay -clock PCLK -clock_fall -min 0.0 \
    [get_ports {GPIO_1[12] GPIO_1[13] GPIO_1[14] GPIO_1[15] GPIO_1[16] GPIO_1[17] GPIO_1[18] GPIO_1[19] GPIO_1[22] GPIO_1[23]}]

# Board buttons, switches, IR and sonar echo are asynchronous
# external inputs. Their receiving logic includes synchronisers.
set_false_path -from [get_ports {KEY[0] SW[0] SW[1] SW[2] IRDA_RXD GPIO_1[29]}]

# Slow/status outputs do not have a synchronous external receiver.
set_false_path -to [get_ports {LEDR[*]}]

# External control outputs; final I/O timing needs hardware review.
set_false_path -to [get_ports {GPIO_1[10] GPIO_1[11] GPIO_1[20] GPIO_1[24] GPIO_1[25] GPIO_1[27] GPIO_1[31]}]

# ------------------------------------------------------------
# CDC handshake synchronisers
# ------------------------------------------------------------
# Only exclude asynchronous paths entering the FIRST
# synchroniser stage.
#
# The paths from *_meta to *_sync remain timing-constrained.
# Bundled payload buses require separate delay verification.

set_false_path -to [get_registers {*u_vision_cdc|ack_meta}]
set_false_path -to [get_registers {*u_vision_cdc|req_meta}]

set_false_path -to [get_registers {*u_sonar_cdc|ack_meta}]
set_false_path -to [get_registers {*u_sonar_cdc|req_meta}]



# SCCB clock: 25 MHz video PLL / 250 = 100 kHz
create_generated_clock -name SCCB_CLK -source [get_pins {clock_divider:u_sccb_divider|clk_out|clk}] -divide_by 250 [get_registers {*u_sccb_divider|clk_out*}]

# OV7670 SCCB SIOD input timing (preliminary)
# GPIO_1[24] is the bidirectional SCCB data pin.
# OV7670 response delay: 100 ns minimum, 900 ns maximum.
# Board delay and output-clock skew remain to be verified.

set_input_delay -clock [get_clocks {SCCB_CLK}] -max 900.000 [get_ports {GPIO_1[24]}]
set_input_delay -clock [get_clocks {SCCB_CLK}] -min 100.000 [get_ports {GPIO_1[24]}]

# Sonar bundled-data CDC: 43.904 MHz -> CLOCK_50
set_max_delay -from [get_registers {*u_sonar_cdc|distance_hold*}] -to [get_registers {*u_sonar_cdc|measurement_mm*}] 20.000
set_false_path -hold -from [get_registers {*u_sonar_cdc|distance_hold*}] -to [get_registers {*u_sonar_cdc|measurement_mm*}]

# Vision bundled-data CDC: PCLK -> CLOCK_50
set_max_delay -from [get_registers {*u_vision_cdc|payload_hold*}] -to [get_registers {*u_vision_cdc|dst_target_x*}] 20.000

# Vision target-valid bundled-data CDC
set_max_delay -from [get_registers {*u_vision_cdc|payload_hold*}] -to [get_registers {*u_vision_cdc|dst_target_valid*}] 20.000

# SCCB camera-configuration status synchroniser: first stage only
set_false_path -to [get_registers {*cfg_sync[0]}]
# Camera stream-integrity status: PCLK -> CLOCK_50
# Exclude only the asynchronous input to the first synchroniser flop.
# The first-to-second synchroniser stage remains timing-constrained.
set_false_path -from [get_registers {*u_camera_adapter|stream_ok}] -to [get_registers {*cam_ok_sync[0]}]

