# Bay handoff: who tells who what

The robot docks at one bay, then the next LED lights up and the robot goes to that one. Four things take part:

- **FPGA**: sees the LED, drives, docks.
- **Nano 33 IoT**: on the robot. Passes messages between the FPGA and the ESP32.
- **ESP32**: at the bays. Switches the LEDs.
- **LEDs**: red, green, blue, one lit at a time.

```mermaid
sequenceDiagram
    participant F as FPGA
    participant N as Nano 33 IoT
    participant E as ESP32
    participant L as LEDs

    E->>L: turn on first LED
    E->>N: LED is on
    N->>F: go

    loop for each bay
        F->>F: find the LED, drive, dock a set distance away
        F->>N: docked
        N->>E: docked
        E->>L: turn this LED off, next one on
        E->>N: next LED is on
        N->>F: go
    end
```

The FPGA handles all of the robot's functionality. The Arduino just tells the ESP32 that the robot is docked and to change the lights, and the ESP32 tells the Arduino that the lights have been changed and the robot should now find its new dock.
