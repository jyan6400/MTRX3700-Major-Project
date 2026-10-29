Frame grab over JTAG on the DE1-SoC (V2026.3)
=============================================

The camera code with one module more, frame_snap. Press KEY1, or ask from the PC, and
frame_snap keeps the next whole camera frame, with the numbers your design computed from that
same frame, in a memory inside the FPGA. The PC reads that memory over the USB-Blaster, the
cable that programs the board: no extra cable and no extra hardware. The picture on the
monitor carries on while it does.

The picture's path through rtl/ is the camera code's: ov7670_capture turns the camera's signals
into pixels, frame_buffer keeps every second pixel of every second row, and camera_display sends
them through vga_sink to the VGA monitor. Beside the frame buffer, on ov7670_capture's outputs,
frame_stats computes your design's numbers (it is an example), and frame_snap keeps a whole
frame and those numbers in the memory GRAB.

What a grab gives you, through tools/grab.py:

    frame.png      the picture as the FPGA kept it: 320 x 240, every second pixel of every
                   second row, as in the frame buffer
    frame.hex      the same pixels for $readmemh, one RGB565 word (4 hex digits) a line
    scene.hex      the picture as a scene for the camera model: in a simulation the model
                   then sends this frame again
    results.json   your design's numbers for that frame (the result words), and the CRC
    dump.bin       the raw bytes, to decode again:
                   python tools/grab.py --file grab1/dump.bin --out grab1


Simulate
--------

    ./simulate.sh                 the top level, then a replay of the frame it grabbed
    ./simulate.sh top             top_level_jtag with the camera model, pins to pins: KEY1,
                                  then the PC's request, each keep a whole frame (the picture
                                  SNAP_MODE sets), read back as the PC reads it; out/top/grab_1/
                                  has the files a grab gives
    ./simulate.sh replay grab1    the frame in the folder grab1 (your own grab: copy the folder
                                  into this workspace) sent again by the camera model, through
                                  ov7670_capture into the frame buffer: every pixel must come
                                  back exactly as frame.hex has it

Each bench ends with a PASS or FAIL line. Every .sv file in rtl/ is compiled, packages
(*_pkg.sv) first, so your own modules go there.

In a replay your design, simulated, sees the picture the board saw. frame_stats runs on the
replayed frame, at YOUR MODULE in tb/replay_tb.sv, and its words from word 1 on must be the
ones in results.json (word 0 is the frame number). Put your own module there in its place,
and the replay checks it against what the board computed. That holds exactly for the pixels
the frame buffer keeps: the camera's odd rows and columns, which it drops, come back as copies
of their neighbours. A replay takes a SNAP_MODE 0 grab, 320 x 240 RGB565: SNAP_MODE 1 keeps
only every fourth pixel, and SNAP_MODE 2 no colour.


Build (Quartus 18.1)
--------------------

Open quartus/FrameGrabJtag.qpf and compile it (Processing > Start Compilation). Program
quartus/output_files/FrameGrabJtag.sof with the Programmer (Tools > Programmer). The design
configures the camera by itself (KEY0 runs the configuration again): LEDR0 lights, LEDR2
flickers, and the picture is on the monitor.

The same from a Command Prompt in the quartus folder, with Quartus installed in its usual
place (change C:\intelFPGA_lite\18.1 if yours is elsewhere; @2: the FPGA is the second device
on the DE1-SoC's JTAG chain):

    C:\intelFPGA_lite\18.1\quartus\bin64\quartus_sh.exe --flow compile FrameGrabJtag
    C:\intelFPGA_lite\18.1\quartus\bin64\quartus_pgm.exe -m jtag -o "p;output_files/FrameGrabJtag.sof@2"

On Linux, with Quartus's bin folder on the PATH, cd quartus && sh build.sh compiles it and then
prints the resources, the timing slack and any critical warning.


Grab a frame
------------

Once, on the PC: Python 3 with numpy and Pillow.

    python -m pip install numpy pillow

On Debian or Ubuntu Linux: sudo apt install python3-numpy python3-pil

With FrameGrabJtag.sof running, in a Command Prompt in this workspace's top folder:

    C:\intelFPGA_lite\18.1\quartus\bin64\quartus_stp.exe -t tcl\grab_jtag.tcl grab1

quartus_stp is Quartus's Tcl shell for working with a running design. The script asks the
board for a snapshot, reads it over the USB-Blaster into grab1\dump.bin, and runs
tools\grab.py on it, which checks the CRC and writes the files above into grab1. Without a
folder name it makes grab_<date>_<time>. If it finds no Python, it prints the grab.py command
to run yourself. On Linux: quartus_stp -t tcl/grab_jtag.tcl grab1

KEY1 takes a snapshot too: LEDR4 lights while it is taken, LEDR3 while a whole one is held.
Add --held after the folder name to read that snapshot without taking a new one.

SW9 up holds the camera's automatic exposure, gain and white balance (rtl/ov7670_hold.sv), and
LEDR9 lights once the camera has taken the hold. The camera adjusts all three by itself, so
the same object can grab with different colours a minute later because something else came
into view. Let the camera settle in the room you will use, put SW9 up, then grab the pictures
you set a colour threshold from, and keep the hold on the robot. SW9 down lets the camera adjust
again. LEDR8 lights if the hold's write is not acknowledged (it is sent again). The simulation
has no such loops, so SW9 changes nothing there.

By hand, in Quartus: Tools > In-System Memory Content Editor, the instance GRAB. Read it, then
File > Export Data to File, as a .mif. Then

    python tools/jtag_export.py grab.mif --out grab2

makes the same files.


Your own numbers
----------------

rtl/frame_stats.sv is an example: five 32-bit words a frame (the frame number, how many pixels
are "red", the sums of their columns and of their rows, and the sum of the grey values).
Replace it with your own module. frame_snap's results input takes N_RESULTS words, word 0 in
the top 32 bits, in the camera's PCLK clock domain, and keeps the values they have at the next
frame's first pixel (frame_snap.sv: THE STEPS). A golden model of your module, run on
frame.hex, must then give the words in results.json, and so must your module in a replay
(both with SNAP_MODE 0: frame_stats counts over its 320 x 240 RGB565 picture).


The memory it takes
-------------------

The FPGA has 397 M10K memory blocks and the frame buffer takes 150 of them. The snapshot
takes more, set by SNAP_MODE at the top of rtl/top_level_jtag.sv:

    SNAP_MODE 0    320 x 240 RGB565, the frame buffer's own picture   about 150 M10K
    SNAP_MODE 1    160 x 120 RGB565                                    about 38
    SNAP_MODE 2    320 x 240 grey, 8 bits a pixel                      about 75

grab.py reads the size and format from the snapshot itself. When your own design needs the
memory, use SNAP_MODE 1 or 2 (the replay and a golden model of frame_stats then no longer
apply: they need SNAP_MODE 0's picture), or take the grab out of your final design: remove the
frame_snap instance and its three files (frame_snap.sv, snapshot_ram.sv, snapshot_probes.sv)
from the .qsf. Nothing else depends on it.

CAMERA_FPS, also at the top of rtl/top_level_jtag.sv, is the camera code's frame rate: 25 (the
default) or 50, an overclock (faster frames, but faint bands across plain surfaces).


Files
-----

    rtl/         the camera code (ov7670_config, ov7670_hold, ov7670_capture, frame_buffer,
                 camera_display, camera_pkg, i2c_master, clock_divider, video_pll) and the frame grab:
                 frame_snap, snapshot_ram, snapshot_probes, frame_stats, top_level_jtag
    tb/          top_jtag_tb, replay_tb, ov7670_model (the camera), vga_monitor_model,
                 vga_sink_sim
    tools/       grab.py, jtag_export.py, check_snap.py (the benches' checks), make_scene.py
    tcl/         grab_jtag.tcl
    quartus/     FrameGrabJtag.qpf, .qsf, .sdc, vga_sink.qsys and make_vga_sink_qsys.tcl
                 (the VGA system), build.sh
