// Group 26 - beacon VO calibration (written by golden/vo/vo_calibrate.py; edit there, not here).
// Source: default guess, not yet measured on the robot (focal length 560 px, pitch 12 deg up,
// LEDs 150 mm above the lens). Run vo_calibrate.py on grabs at tape-measured positions.
`ifndef VO_CALIBRATION_SVH
`define VO_CALIBRATION_SVH
`define VO_F_Q4    8960
`define VO_CX_Q4   5112
`define VO_CY_Q4   3832
`define VO_SIN_Q14 3406
`define VO_COS_Q14 16026
`define VO_DH_MM   150
`endif
