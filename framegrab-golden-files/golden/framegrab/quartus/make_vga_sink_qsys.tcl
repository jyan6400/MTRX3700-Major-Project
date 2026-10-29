# Builds vga_sink.qsys: a 25 MHz clock input and the University Program VGA Controller with its
# Avalon-ST video sink EXPORTED, so the pixel source is plain SystemVerilog in top_level.sv.
#   qsys-script --script=make_vga_sink_qsys.tcl
package require -exact qsys 18.1
create_system vga_sink
set_project_property DEVICE_FAMILY "Cyclone V"
set_project_property DEVICE 5CSEMA5F31C6
set_project_property HIDE_FROM_IP_CATALOG false

add_instance clk_0 clock_source
set_instance_parameter_value clk_0 clockFrequency 25000000
set_instance_parameter_value clk_0 clockFrequencyKnown true
set_instance_parameter_value clk_0 resetSynchronousEdges DEASSERT

add_instance video_vga_controller_0 altera_up_avalon_video_vga_controller
set_instance_parameter_value video_vga_controller_0 board "DE2-115"
set_instance_parameter_value video_vga_controller_0 device "VGA Connector"
set_instance_parameter_value video_vga_controller_0 resolution "VGA 640x480"
set_instance_parameter_value video_vga_controller_0 underflow_flag false

add_connection clk_0.clk       video_vga_controller_0.clk
add_connection clk_0.clk_reset video_vga_controller_0.reset

add_interface clk clock sink
set_interface_property clk EXPORT_OF clk_0.clk_in
add_interface reset reset sink
set_interface_property reset EXPORT_OF clk_0.clk_in_reset
add_interface video_in avalon_streaming sink
set_interface_property video_in EXPORT_OF video_vga_controller_0.avalon_vga_sink
add_interface vga conduit end
set_interface_property vga EXPORT_OF video_vga_controller_0.external_interface

save_system vga_sink.qsys
