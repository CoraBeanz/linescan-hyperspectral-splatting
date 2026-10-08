# Scan-mirror firmware

ESP32 firmware for the scanner head's mirror: it drives the NEMA 8 stepper through a TMC2209, homes on the A3144 hall sensor, and runs scans whose lines land on the camera's frame clock. The Jetson talks to it over USB serial in plain text lines ([PROTOCOL.md](PROTOCOL.md)).

- **Moves** ramp from a start speed to `vmax` and back, and stop exactly on the target microstep.
- **Homing** measures both edges of the hall sensor's window at a slow speed in one direction and takes the middle, so it repeats to a microstep.
- **Stare scans** step the mirror at each line tick and report when it has settled. **Sweep scans** turn it at a constant speed and report its exact position at each tick.
- **Frame sync.** Line ticks follow a clock the host sets in ESP32 microseconds (`t0`, `period`) and nudges to stay on the camera's frames. Every line, move and hall edge is stamped on that clock, and two pins (`MOVING`, `TRIG`) show the timing to a scope or to the Jetson's GPIO.
- **TMC2209 over UART:** microstepping, current and chopper mode are set in software, and a watchdog stops the motor on overheating, shorts or a driver reset.

All the logic is plain C++ in [`lib/scanmirror`](lib/scanmirror/src), tested on a simulated rig (a fake clock, driver and rotor passing a hall sensor); [`src/main.cpp`](src/main.cpp) is the ESP32 glue.

## Parts

| Part | Notes |
|---|---|
| ESP32 DevKit (ESP32-WROOM-32, `esp32dev`) | any 30 or 38 pin board with GPIO 16 to 27 |
| BIGTREETECH TMC2209 module | 0.11 Ω sense resistors; for another module set `kTmcRSense` in [`include/board_pins.h`](include/board_pins.h) |
| StepperOnline 8HS11-0204S | NEMA 8, 1.8°, 0.2 A, 24 Ω per coil |
| 12 V 3 A supply | for the motor only; the ESP32 runs from USB |
| A3144 hall sensor + 5 × 2 mm magnet | magnet in the mirror clamp, sensor in the head's +X wall |
| 100 µF 25 V electrolytic capacitor | across VM and GND at the driver |
| 10 kΩ resistors (2), 1 kΩ resistor (1) | EN pull-up, hall pull-up, UART |

## Wiring

| ESP32 | Goes to | Notes |
|---|---|---|
| GPIO 26 | TMC2209 STEP | |
| GPIO 25 | TMC2209 DIR | |
| GPIO 27 | TMC2209 EN | **and 10 kΩ from EN to 3.3 V**, so the motor stays off while the ESP32 boots |
| GPIO 16 (RX2) | TMC2209 PDN_UART | straight to the pin |
| GPIO 17 (TX2) | TMC2209 PDN_UART | **through 1 kΩ** (both ends share the one wire) |
| 3.3 V | TMC2209 VDD (VIO) | logic supply |
| GND | TMC2209 GND, 12 V supply −, A3144 GND | one common ground |
| GPIO 33 | A3144 OUT (pin 3) | **and 10 kΩ from OUT to 3.3 V** |
| 5 V (VIN) | A3144 VCC (pin 1) | the A3144 needs 4.5 V or more; its output only pulls low, so 3.3 V is all the ESP32 sees |
| GPIO 18 | MOVING out | high while the mirror moves or settles |
| GPIO 19 | TRIG out | 100 µs pulse when a scan line is ready |
| GPIO 2 | status LED | off: disabled; on: enabled; blinking: driver fault. A genuine DevKitC has no LED on this pin (many clones have a blue one); the [controller board](../pcb/README.md#scan-mirror-controller) has its own, D4 |

TMC2209 MS1 and MS2 go to GND (UART address 0). The 12 V supply goes to VM and GND, with the capacitor right at the module. The motor's two coils go to A1/A2 and B1/B2 (the BTT board labels them A2, A1, B1, B2, below VM and GND). Find the pairs with a multimeter instead of trusting wire colours: about 24 Ω within a coil, open between coils. A coil wired backwards only reverses the direction, which `CFG dir_inv=1` undoes.

The motor and hall wires run up the arm to the head. Twist each coil's pair, and keep the hall wires away from them.

## Before the first power-up

1. **Turn the current down.** The BTT module leaves the factory with VREF at about 1.2 V, which is 0.85 A, over four times what this motor takes. Its current follows I<sub>RMS</sub> = VREF / √2, so 0.2 A needs **VREF ≈ 0.28 V**. With 12 V on and the motor *unplugged*, measure between the potentiometer and GND and turn it down. The firmware sets the current over the UART and ignores VREF, but VREF is what the driver uses before the firmware takes over and in `ENABLE force=1`.
2. **Never plug or unplug the motor while the 12 V is on.** The voltage spike can kill the driver.
3. Check the EN pull-up: with the ESP32 in reset (hold its EN button), the TMC2209's EN pin should read 3.3 V.

The NEMA 8 at 200 mA turns 1 to 2 W into heat and runs warm to the touch. The PETG housing copes; a PLA one would soften over time. During scans the motor holds at full current (`scan_hold=1`) so the mirror doesn't shift between lines; between scans it drops to `ihold` (50%).

## Build and flash

With [PlatformIO](https://platformio.org/) (the VS Code extension, or `pip install platformio`), from this folder:

```
pio run -e esp32dev -t upload          # build and flash; add --upload-port COM5 to pick the port
pio device monitor -e esp32dev         # 921600 baud, doesn't reset the board
pio test -e native                     # the unit tests, on the PC (needs gcc/g++)
```

If the upload can't connect, hold the board's BOOT button while it says `Connecting...`. On Windows, the native tests need a gcc on the PATH (MSYS2's `mingw-w64-ucrt-x86_64-gcc`); CI runs them on every push either way.

## First run

In the monitor (type a command and press Enter), or with `python tools/mirrorctl.py -p COM5 <commands>`:

```
STATUS                          state=disabled, drv=ok: the ESP32 and the TMC2209's UART work
DRV                             ver=0x21 enn=1: the driver is a TMC2209 and EN is high (motor off)
ENABLE                          the motor holds; DRV now shows enn=0
MOVE rel=32                     one full step: the mirror turns 1.8°
MOVE rel=-32
HOME                            EV HOMED ... width=<the hall window, in microsteps>
SCAN lines=20 step=8 period=100000
```

If `HOME` reports `not_found`: check that `STATUS` shows `hall=1` with the magnet held at the sensor, and if it never does, flip the magnet over (the A3144 only reacts to one pole). If the mirror turns the wrong way for your setup, `CFG dir_inv=1` then `SAVE`. Settings stay in RAM until `SAVE`.

## Bench test without the motor

The timing can be checked before the stepper arrives, with the Analog Discovery 3's logic analyser on STEP (GPIO 26), DIR (25), MOVING (18) and TRIG (19), and its ground on the ESP32's GND. Without a TMC2209 wired up, `ENABLE` fails with `code=driver`; `ENABLE force=1` runs without it. Then:

```
python tools/mirrorctl.py -p COM5 "ENABLE force=1" "SCAN start=0 step=2 lines=10 period=33333.333"
```

From line 1 on, each line shows two STEP pulses about 0.65 ms apart starting at its tick, MOVING high from the first step until 3 ms after the last, and a TRIG pulse as MOVING falls. A sweep (`mode=sweep step=4 period=10000`) shows STEP pulses every 2.5 ms with MOVING high throughout. The `t` and `ready` in the `EV LINE` lines match the edges to within a few microseconds.

## Calibration

- **Microstep angles are not even.** A hybrid stepper's microsteps can be a few percent of a full step off their ideal angles, repeating every 4 full steps (128 microsteps at 1/32). For the datacube, measure once: shine a laser pointer off the mirror onto a wall a couple of metres away and mark the spot every 8 microsteps over 128 (one microstep moves it 4 mm at 2 m). The marks give a correction table from position to angle.
- **Working angle.** `home_pos` (−711 by default, the CAD's −40°) sets which position is the mirror's 45° working angle. Once the head is built, find the position that centres the view, then `CFG home_pos=<−711 minus that position>`, `SAVE` and `HOME` again.
- **Settle time.** 3 ms covers a small mirror on this motor with margin. If stare lines look smeared, raise `settle`.

## How it works

A hardware timer interrupts every 50 µs on core 0 (`loop()` and the serial port run on core 1, so neither can delay a step). Each tick runs [`MirrorCore::tick()`](lib/scanmirror/src/sm_core.cpp):

- **Step generator.** A fixed-rate DDA: speed is a fraction of a step per tick in 32-bit fixed point, added to an accumulator each tick, and a carry is a step. It accelerates by `accel` per tick and starts slowing down when the distance left is what it takes to slow to `vstart`. Only integer maths runs in the interrupt, because the ESP32's interrupts must not use the FPU. The first step of a move is taken on the tick it starts, so a one-microstep line move happens on the line tick itself.
- **Line clock.** Line ticks are kept in 1/65536 µs, so a 33333.333 µs period stays on the camera's frames over thousands of lines. `NUDGE` and `PERIOD` move it while it runs.
- **Hall sensor.** Read every tick and debounced; an edge is reported with the position and time of its first reading.
- **Events** go into a lock-free queue that `loop()` drains and prints, with a count of any lost.

`loop()` runs the [controller](lib/scanmirror/src/sm_controller.cpp): command parsing, homing as a sequence of moves driven by events, and the TMC2209 watchdog.

**Why plain serial rather than micro-ROS.** The parts list planned micro-ROS. A text protocol needs no agent on the Jetson, can be typed into any serial monitor and tested without ROS, and carries the ESP32's microsecond timestamps exactly; a ROS 2 node on the Jetson turns it into topics and services.

## Files

```
platformio.ini              esp32dev and native (test) environments
include/board_pins.h        pin map
src/main.cpp                ESP32 glue: timer interrupt, pins, TMC2209, flash, serial
lib/scanmirror/src/
├── sm_stepgen.*            step generator
├── sm_line_clock.h         scan line clock
├── sm_core.*               everything the timer interrupt runs
├── sm_controller.*         commands, homing, driver watchdog
├── sm_cmdline.*            command parsing and reply formatting
├── sm_config.*             settings, their ranges and defaults
├── sm_event_queue.h        interrupt-to-loop queue
└── sm_interfaces.h         what the controller needs from the board
test/                       Unity tests and the simulated rig they run on
tools/mirrorctl.py          command-line client (pyserial)
PROTOCOL.md                 the serial protocol
```
