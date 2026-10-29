# grab_jtag.tcl -- the JTAG frame grab, the PC's side: take a snapshot, read it over the USB-Blaster, decode it.
#
# Run it on the computer the DE1-SoC's USB-Blaster is plugged into, after programming a design with the JTAG frame
# grab (rtl/frame_snap.sv, as in rtl/top_level_jtag.sv), from the frame-grab folder:
#
#     quartus_stp -t tcl/grab_jtag.tcl                    a snapshot now, into a new folder grab_<date>_<time>
#     quartus_stp -t tcl/grab_jtag.tcl mygrab             into the folder mygrab
#     quartus_stp -t tcl/grab_jtag.tcl mygrab --held      no new snapshot: read the one already held (after KEY1)
#     quartus_stp -t tcl/grab_jtag.tcl mygrab --hstart 152    anything after the folder that is not --held goes to
#                                                             tools/grab.py (--mvfp, --hstart, --vstart)
#
# On Windows quartus_stp is C:\intelFPGA_lite\18.1\quartus\bin64\quartus_stp.exe (or wherever Quartus 18.1 is).
#
# What it does:
#   1. finds the USB-Blaster and the DE1-SoC's FPGA, the probes GCTL and the memory GRAB (rtl/frame_snap.sv);
#   2. flips the request bit in GCTL and waits until the snapshot is done: the count has moved on, ready is high
#      (under 85 ms at 25 frames a second, 45 ms at 50);
#   3. reads the memory GRAB: the header first, for how many words the snapshot takes, then all of them;
#   4. writes them, in address order, two bytes a word, to <folder>/dump.bin, the bytes as frame_snap.sv laid them out;
#   5. runs tools/grab.py --file <folder>/dump.bin --out <folder>, which checks the CRC and writes frame.png,
#      frame.hex, scene.hex and results.json. If no Python is found, it prints the command to run.
# The exit status is 1 if anything went wrong, grab.py's check included (a bad CRC, or no numpy), and 0 otherwise.
# Everything it prints goes into <folder>/grab_jtag.log too, with how long the JTAG read took.

load_package insystem_source_probe
load_package insystem_memory_edit
load_package jtag

set HERE  [file dirname [file normalize [info script]]]
set TOOLS [file normalize [file join $HERE .. tools]]

# ---- the arguments ---------------------------------------------------------------------------------------------------
set OUT ""; set HELD 0; set GRAB_ARGS {}
foreach a $argv {
    if {$a eq "--held"} { set HELD 1 } \
    elseif {$OUT eq "" && ![string match "-*" $a] && [llength $GRAB_ARGS] == 0} { set OUT $a } \
    else { lappend GRAB_ARGS $a }
}
if {$OUT eq ""} { set OUT [clock format [clock seconds] -format "grab_%Y%m%d_%H%M%S"] }
file mkdir $OUT
set LOG [open [file join $OUT grab_jtag.log] a]
set T0  [clock milliseconds]
proc say {msg} {
    set line [format "%6.2f s  %s" [expr {([clock milliseconds] - $::T0) / 1000.0}] $msg]
    puts $line; puts $::LOG $line; flush stdout; flush $::LOG
}
proc give_up {msg} { say "ERROR: $msg"; close $::LOG; exit 1 }

# ---- 1. the cable, the FPGA, the probes and the memory ----------------------------------------------------------------
set HW ""; set DEV ""
if {[catch {get_hardware_names} hws]} { give_up "no JTAG cable ($hws): is the USB-Blaster's cable plugged into this computer?" }
foreach hw $hws {
    if {[catch {get_device_names -hardware_name $hw} devs]} { continue }
    foreach dev $devs {
        if {[string match "*5CSE*" $dev] || [string match "*02D120DD*" $dev]} { set HW $hw; set DEV $dev }
    }
}
if {$HW eq ""} { give_up "no DE1-SoC FPGA (5CSEMA5) on any JTAG cable: is the board on, and its USB-Blaster cable plugged in?" }
say "cable: $HW, device: $DEV"

set PROBES -1
if {[catch {get_insystem_source_probe_instance_info -hardware_name $HW -device_name $DEV} insts]} { set insts {} }
foreach inst $insts { if {[lindex $inst 3] eq "GCTL"} { set PROBES [lindex $inst 0] } }
set MEM -1; set MEM_DEPTH 0
if {[catch {get_editable_mem_instances -hardware_name $HW -device_name $DEV} mems]} { set mems {} }
foreach inst $mems { if {[lindex $inst 5] eq "GRAB"} { set MEM [lindex $inst 0]; set MEM_DEPTH [lindex $inst 1] } }
if {$PROBES < 0 || $MEM < 0} {
    give_up "the FPGA has no JTAG frame grab (probes GCTL: [expr {$PROBES < 0 ? "missing" : "found"}], memory GRAB:\
             [expr {$MEM < 0 ? "missing" : "found"}]): program the design with rtl/frame_snap.sv in it first"
}

# probe bits, most significant first: count[15:0], ack, busy, ready, MODE[1:0]
proc probe {} {
    while 1 {                          ;# a probe that is changing can be read half old, half new: read until two agree
        set a [read_probe_data -instance_index $::PROBES]
        set b [read_probe_data -instance_index $::PROBES]
        if {$a eq $b} { break }
    }
    set s [string range "[string repeat 0 21]$a" end-20 end]
    set count 0; foreach c [split [string range $s 0 15] ""] { set count [expr {($count << 1) | $c}] }
    return [list count $count ack [string index $s 16] busy [string index $s 17] ready [string index $s 18]]
}

# ---- 2. the snapshot ------------------------------------------------------------------------------------------------------
start_insystem_source_probe -hardware_name $HW -device_name $DEV
set p [probe]
if {$HELD} {
    if {![dict get $p ready]} {
        end_insystem_source_probe
        give_up "no whole snapshot is held (LEDR3 is off): press KEY1 first, or leave out --held"
    }
    say "reading the snapshot already held (number [dict get $p count] since the board was programmed)"
} else {
    set req [string index [read_source_data -instance_index $PROBES] end]
    set new [expr {$req eq "1" ? 0 : 1}]
    set t [clock milliseconds]
    write_source_data -instance_index $PROBES -value $new
    while 1 {
        set q [probe]
        if {[dict get $q count] != [dict get $p count] && ![dict get $q busy] && [dict get $q ready] && [dict get $q ack] == $new} { break }
        if {[clock milliseconds] - $t > 3000} {
            end_insystem_source_probe
            if {[dict get $q busy]} {
                give_up "no snapshot after 3 s: frames stopped arriving during it (LEDR0 should be lit and LEDR2 flickering; KEY0 configures the camera again)"
            }
            give_up "the snapshot did not start within 3 s. frame_snap takes the request on the camera's PCLK, so the camera\
                     is not sending: LEDR0 should be lit and LEDR2 flickering (check the ribbon; KEY0 configures the camera again)"
        }
        after 5
    }
    set p $q
    say "snapshot [dict get $p count] taken in [expr {[clock milliseconds] - $t}] ms"
}
end_insystem_source_probe

# ---- 3. and 4. read the memory, write dump.bin --------------------------------------------------------------------------
proc words {start n} {                  ;# n words from start, as hex in address order (quartus_stp gives the highest first)
    set hex [read_content_from_memory -instance_index $::MEM -start_address $start -word_count $n -content_in_hex -timeout 60000]
    set out ""
    for {set k [expr {$n - 1}]} {$k >= 0} {incr k -1} { append out [string range $hex [expr {4 * $k}] [expr {4 * $k + 3}]] }
    return $out
}
set t [clock milliseconds]
if {[catch {begin_memory_edit -hardware_name $HW -device_name $DEV} err]} { give_up "cannot open the memory for reading: $err" }
set head [words 0 7]
scan $head "%4x%4x%4x%4x%4x%4x%4x" m1 m2 version w h fmt n
if {$m1 != 0x4752 || $m2 != 0x4142} {
    end_memory_edit
    give_up "the memory does not start with 'GRAB': no snapshot in it (press KEY1, or run without --held)"
}
set pix [expr {$fmt == 2 ? $w * $h / 2 : $w * $h}]
set total [expr {7 + 2 * $n + $pix + 1}]
if {$total > $MEM_DEPTH} { end_memory_edit; give_up "the header says $total words, but the memory has only $MEM_DEPTH" }
set data ""
for {set s 0} {$s < $total} {incr s 16384} {
    append data [words $s [expr {min(16384, $total - $s)}]]
}
end_memory_edit
set dt [expr {[clock milliseconds] - $t}]
set f [open [file join $OUT dump.bin] wb]
puts -nonewline $f [binary format H* $data]
close $f
say "read $total words ([expr {2 * $total}] bytes: $w x $h, format $fmt, $n result words) in $dt ms; wrote [file join $OUT dump.bin]"

# ---- 5. decode it with tools/grab.py ------------------------------------------------------------------------------------------
set cmd [concat [list [file join $TOOLS grab.py] --file [file join $OUT dump.bin] --out $OUT] $GRAB_ARGS]
set done 0; set failed 0
foreach py {python3 python {py -3}} {
    if {[catch {exec {*}$py {*}$cmd 2>@1} result opts]} {
        if {[string match "*grab.py*" $result] || [string match "*REJECTED*" $result]} { say $result; set done 1; set failed 1; break }
        continue                       ;# not this Python (not installed, or a placeholder): try the next one
    }
    say $result; set done 1; break
}
if {!$done} {
    say "no Python found to run tools/grab.py. Run it yourself:\n    python [join $cmd]"
}
close $LOG
if {$failed} { exit 1 }                 ;# grab.py ran and rejected the dump, or could not run (no numpy)
