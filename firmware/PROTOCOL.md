# Scan-mirror serial protocol

Protocol version 1 (`proto=1` in `INFO` and `EV BOOT`). The ESP32 speaks it over its USB serial port; [`tools/mirrorctl.py`](tools/mirrorctl.py) is a working client, and the ROS 2 bridge on the Jetson is the other one.

## The link

- **921600 baud, 8N1**, plain ASCII, one message per line ending in `\n` (`\r\n` and `\r` work too).
- **Keep DTR and RTS released.** On an ESP32 DevKit they drive the reset and boot pins, so a port that opens with them asserted restarts the board. pyserial: set `ser.dtr = False` and `ser.rts = False` *before* `ser.open()`. PlatformIO's monitor does it through `monitor_dtr = 0` and `monitor_rts = 0`.
- **Ignore lines that don't start with `OK `, `ERR ` or `EV `.** After a reset the ESP32's boot ROM prints a few lines at 115200 baud, which arrive as garbage.
- Lines are at most 255 characters.

## Messages

The host sends commands:

```
VERB key=value key=value ... [id=<anything>]
```

Verbs are case-insensitive, keys are lowercased, values have no spaces. `id` is optional and comes back in the reply, so a client can match replies to commands. A blank line or one starting with `#` is ignored.

Every command gets **exactly one reply**, in the order the commands were sent:

```
OK VERB key=value ... [id=...]
ERR VERB code=<code> [id=...] [msg=<text to the end of the line>]
```

The ESP32 also sends **events** whenever something happens:

```
EV NAME key=value ...
```

A reply always comes before any event its command causes (for example `OK STOP` before `EV STOPPED`).

### Units

- **Positions** are microsteps, signed, at the configured microstepping (`usteps`, default 32). The NEMA 8 has 200 full steps per turn, so one turn is 6400 microsteps and one microstep turns the mirror 0.05625° and the reflected view 0.1125°. Position 0 is wherever the mirror was at power-up until `HOME` runs; after `HOME` it is the mirror's working angle (see `home_pos`).
- **Times** (`t`, `t0`, `ready`, `next`) are the ESP32's own clock: microseconds since boot, 64-bit. Step pulses and line ticks happen on its 50 µs timer grid, and every time reported is the moment the interrupt acted.
- **Periods** are microseconds with up to six decimals (`33333.333`); the ESP32 keeps them to 1/65536 µs, so a 30 fps period doesn't drift.

## Commands

| Command | Reply | Then |
|---|---|---|
| `INFO` | `OK INFO fw=0.1.0 proto=1 usteps=32 full_steps=200 tick_us=50 max_rate=10000` | |
| `PING` | `OK PING t=<now>` | |
| `STATUS` | `OK STATUS state= pos= target= homed= en= hall= line= lines= drv= fault= dropped= t=` | |
| `ENABLE [force=1]` | `OK ENABLE drv=ok` | |
| `DISABLE` | `OK DISABLE` | `EV STOPPED` if it was moving |
| `HOME` | `OK HOME` | `EV HOMED` or `EV HOME_FAILED` |
| `MOVE pos=<p>` or `MOVE rel=<d>` `[v=<µsteps/s>] [a=<µsteps/s²>]` | `OK MOVE pos=<target>` | `EV MOVED` |
| `STOP` | `OK STOP` | `EV STOPPED`, always |
| `SCAN lines=<n> period=<µs> [mode=stare\|sweep] [start=<p>] [step=<d>] [t0=<t> \| delay=<µs>] [settle=<µs>]` | `OK SCAN mode= t0= period= start= step= lines=` | `EV LINE` per line, then `EV SCAN_DONE` |
| `NUDGE dt=<µs>` | `OK NUDGE next=<t>` | |
| `PERIOD us=<µs>` | `OK PERIOD us=<µs>` | |
| `ZERO [pos=<p>]` | `OK ZERO pos=<p>` | |
| `CFG` | `OK CFG <every setting>` | |
| `CFG key=value ...` | `OK CFG <the settings changed>` | |
| `SAVE` | `OK SAVE` | |
| `DEFAULTS` | `OK DEFAULTS` | |
| `DRV` | `OK DRV ver= enn= cs= stst= stealth= ot= otpw= s2g= s2vs= ol= reset= drv_err= uv_cp= mscnt= raw=` | |
| `REBOOT` | `OK REBOOT` | the board restarts: `EV BOOT` |
| `HELP` | `OK HELP verbs=...` | |

**STATUS.** `state` is one of `disabled`, `idle`, `moving`, `stopping`, `homing`, `scanning`, `fault`. `pos` is where the mirror is and `target` where it is going. `homed` is 1 once `HOME` has succeeded (cleared by `DISABLE`, `ZERO`, `DEFAULTS`, faults, and changes to `usteps`, `dir_inv` or `home_pos`). `en` is 1 while the motor is powered. `hall` is 1 while the magnet is at the sensor. `line` and `lines` count the lines done and asked for in the running or last scan. `drv` is `ok`, `noresp` or `bypass`, `fault` the last driver fault code (`none`), and `dropped` the number of events lost because the host read too slowly.

**ENABLE** writes every setting to the TMC2209 over its UART, checks that it took them, clears a fault and powers the motor. It fails with `code=driver` if the TMC2209 doesn't answer; `force=1` then runs without the UART, at the current the driver's VREF potentiometer sets (`drv=bypass`; set VREF first, see the README). Sent while the mirror is busy, it replies `OK ENABLE` and changes nothing.

**DISABLE** cuts the motor current at once (EN high). A move stops dead (`EV STOPPED`), homing fails (`EV HOME_FAILED reason=disabled`) and a scan ends (`EV SCAN_DONE aborted=1`). The position is kept but no longer trusted, so `homed` drops to 0.

**HOME** finds the middle of the hall sensor's window and calls it `home_pos`, then moves to `home_park` (both settings). It needs `state=idle`:

1. If the magnet is already at the sensor, it first moves off it in the + direction.
2. It searches in the − direction at `home_fast` (up to `home_range` microsteps) until the sensor turns on, and stops.
3. It backs off `home_clear` microsteps past that edge.
4. It crosses the window in the − direction at `home_slow`, noting where the sensor turns on and off, and stops.
5. The middle of those two edges becomes `home_pos`; it moves to `home_park`.

Both edges are measured moving the same way at the same slow speed, so the sensor's hysteresis shifts them alike and the middle repeats to a microstep. `EV HOMED pos=<home_park> t= width=<window in µsteps> shift=<how far the numbering moved>`. Failures: `EV HOME_FAILED reason=<r> pos= t=` with `not_found` (no magnet within `home_range`: check the wiring, then flip the magnet over, as the A3144 reacts to one pole only), `stuck` (the sensor stayed on while moving off it), `too_wide` (the window is wider than `home_width`), `lost` (the slow pass missed the window), `timeout` (over 60 s), `stopped`, `disabled` or `fault`. While homing, `EV HALL` events show each edge.

**MOVE** goes to `pos`, or `rel` microsteps past the current target, within `min`..`max`. `v` and `a` override `vmax` and `accel` for this move. It starts at `vstart` (no ramp below that), accelerates, and decelerates to stop exactly on the target. `EV MOVED pos= t=` comes when the mirror has been still for `settle` µs, and `t` is that moment. A MOVE while moving replaces the old target, and only the last one reports `MOVED`. If the new target is behind the mirror, or closer than it can brake, the mirror slows down past it at the move's acceleration and comes back, so a MOVE never stops it harder than a normal move does.

**STOP** decelerates to a stop and reports `EV STOPPED pos= t=` once settled, also when nothing was moving. It aborts a scan (`EV SCAN_DONE aborted=1` first) or homing (`EV HOME_FAILED reason=stopped` first).

**ZERO** renumbers the current position as `pos` (default 0) without moving. Not while moving.

**SAVE** writes the settings to flash; they load at every boot. Not while moving: writing flash pauses the step interrupt for a few ms. **DEFAULTS** restores the factory settings (in RAM; `SAVE` to keep them).

**DRV** reads the TMC2209: `ver` (0x21 for a TMC2209), `enn` (what the driver sees on its EN pin: 1 = motor off), `cs` (the current scale it is using, 0..31), `stst` (standstill), `stealth` (in StealthChop), the fault flags `ot`, `otpw`, `s2g`, `s2vs`, `ol`, the reset and error flags from GSTAT (reading clears them), `mscnt` (position in the 1024-entry sine table) and `raw` (DRV_STATUS).

### Scans

A scan takes `lines` lines on a clock: line `n` is due at `t0 + n × period`. That clock is meant to follow the camera's frame clock, and the host keeps it there with `NUDGE` and `PERIOD`.

- **`mode=stare`** (default). At each line's tick the mirror steps to `start + n × step` and stops. When it has been still for `settle` µs (default the `settle` setting, 3 ms), the line is **ready**: the TRIG pin pulses and the `MOVING` pin is low. `EV LINE n= t=<tick> pos= ready=<t or -1>` is sent at that moment. `ready=-1` means the mirror was still moving when the next tick came: lengthen `period`, make `step` smaller, or raise `vmax` and `accel`. Line 0 starts with the mirror already at `start`, so it is ready at its tick.
- **`mode=sweep`**. The mirror turns at a constant `|step| / period` microsteps per second from line 0's tick, and `EV LINE n= t=<tick> pos=` gives its exact position at each tick (half a microstep of rounding). That speed must not exceed `vstart`, because the sweep starts and stops without a ramp. TRIG pulses at every tick.

Before line 0 the mirror moves to `start` (default: where it is). Ticks that come before it has arrived and settled are skipped a whole period at a time, so line 0 always lands on the `t0 + k × period` grid. `t0` is a time on the ESP32's clock and `delay` gives it relative to now; with neither, `t0` is now. A `t0` less than 2 ms ahead (or in the past) moves forward by whole periods, keeping its phase, and `OK SCAN` reports the `t0` actually used. `t0` may be up to 10 minutes ahead.

`EV SCAN_DONE lines=<lines sent> t= pos= aborted=<0|1>` ends every scan; for a complete scan, `t` is the tick after the last line. `STOP`, `DISABLE` and driver faults abort one. Scans need `state=idle`; the range `start` .. `start + (lines − 1) × step` (stare) or `start + lines × step` (sweep) must lie within `min`..`max`. With `scan_hold=1` the motor keeps its full run current while the scan stands still (no current dip between lines).

**NUDGE** `dt=<µs>` moves every remaining tick of the running scan by `dt` (negative is earlier). `next` is the next tick's new time. **PERIOD** `us=<µs>` sets a new period from the tick after the next one (the next one keeps its time); in a sweep it changes the speed too. Both need a running scan (`code=no_scan` otherwise).

### Settings

`CFG key=value ...` changes settings while the mirror is idle (or disabled); `SAVE` keeps them. Positions and speeds are in microsteps at the current `usteps`.

| Key | Default | Range | Meaning |
|---|---|---|---|
| `usteps` | 32 | 1..256, power of 2 | microsteps per full step (TMC2209 MRES) |
| `irun` | 200 | 50..1200 | motor current, mA RMS (the 8HS11-0204S is rated 200 mA) |
| `ihold` | 50 | 0..100 | standstill current when idle, % of `irun` |
| `stealth` | 1 | 0, 1 | 1 = StealthChop (quiet, smooth), 0 = SpreadCycle |
| `dir_inv` | 0 | 0, 1 | flip the DIR pin, so + turns the other way |
| `vmax` | 3200 | 1..10000 | move speed, µsteps/s (3200 = half a turn per second) |
| `vstart` | 1600 | 1..10000 | start/stop speed: moves begin and end at this speed without a ramp |
| `accel` | 20000 | 100..1000000 | µsteps/s² |
| `settle` | 3000 | 0..1000000 | µs of stillness after the last step before a move or line counts as done |
| `min`, `max` | −3200, 3200 | | soft limits for MOVE and SCAN |
| `home_pos` | −711 | | position given to the middle of the hall window (−40° of mirror, from the CAD) |
| `home_park` | 0 | within `min`..`max` | where HOME leaves the mirror |
| `home_fast` | 1600 | 1..10000 | search speed |
| `home_slow` | 100 | ≤ `home_fast` | speed for the measuring pass |
| `home_range` | 7200 | | furthest the search goes (a little over a turn) |
| `home_clear` | 160 | | back-off past the window before the measuring pass |
| `home_width` | 1600 | | widest window accepted |
| `hall_high` | 0 | 0, 1 | 1 if the sensor output is high at the magnet (the A3144 pulls low) |
| `debounce` | 4 | 1..1000 | hall readings (50 µs each) a change must hold for |
| `trig_us` | 100 | 0..100000 | TRIG pulse length; 0 turns it off |
| `scan_hold` | 1 | 0, 1 | full current at standstill during scans |

Changing `usteps`, `irun`, `ihold` or `stealth` rewrites the TMC2209 at once.

## Events

| Event | When |
|---|---|
| `EV BOOT fw= proto= reset= drv=ok\|noresp t=` | after every start. `reset` is `poweron`, `external`, `software`, `panic`, `brownout`, a watchdog (`int_wdt`, `task_wdt`, `wdt`) or `unknown`. The motor is off. |
| `EV LINE n= t= pos= [ready=]` | a scan line (see Scans) |
| `EV SCAN_DONE lines= t= pos= aborted=` | a scan ended |
| `EV MOVED pos= t=` | a MOVE arrived and settled |
| `EV STOPPED pos= t=` | after STOP, or when DISABLE or a fault stopped a moving mirror |
| `EV HOMED pos= t= width= shift=` | HOME succeeded |
| `EV HOME_FAILED reason= pos= t=` | HOME failed |
| `EV HALL on=0\|1 pos= t=` | the hall sensor changed; `pos` and `t` are where and when the new state was first seen |
| `EV FAULT code= t= msg=` | the TMC2209 reported a problem; the motor is off and `state=fault` until `ENABLE`. Codes: `drv_ot` (overheated), `drv_short` (short on a coil), `drv_uv` (supply too low), `drv_err`, `drv_reset` (the driver lost power and forgot its settings), `drv_noresp` (stopped answering) |
| `EV WARN code= t= msg=` | `drv_otpw` (the driver is getting hot, sent once), `events_dropped n=<total>` (the host is not reading fast enough) |

The driver is checked every half second while the motor is enabled (not in `bypass`).

## Errors

| Code | Meaning |
|---|---|
| `syntax` | not `VERB key=value ...`, a repeated key, or a line over 255 characters (`ERR ? code=syntax`) |
| `unknown_cmd` | no such verb |
| `bad_arg` | an unknown key, a value that isn't a number or is out of its range, a missing or conflicting argument |
| `range` | a target or scan outside `min`..`max`, a sweep faster than `vstart`, a `t0` too far ahead |
| `disabled` | the motor is off (`state=disabled` or `fault`): `ENABLE` first |
| `busy` | not while moving, stopping, homing or scanning |
| `no_scan` | `NUDGE` or `PERIOD` with no scan running |
| `driver` | the TMC2209 didn't answer |
| `flash` | the settings could not be saved |

## Example

```
> ENABLE id=1
OK ENABLE drv=ok id=1
> HOME id=2
OK HOME id=2
EV HALL on=1 pos=-1440 t=1900500
EV HALL on=0 pos=-1429 t=1909800
EV HALL on=1 pos=-1440 t=3596050
EV HALL on=0 pos=-1571 t=4906050
EV HALL on=1 pos=-765 t=4915150
EV HALL on=0 pos=-634 t=4972300
EV HOMED pos=0 t=5193950 width=131 shift=795
> SCAN mode=stare start=-20 step=2 lines=5 period=33333.333 t0=5243950 id=3
OK SCAN mode=stare t0=5243950 period=33333.333 start=-20 step=2 lines=5 id=3
EV LINE n=0 t=5243950 pos=-20 ready=5243950
EV LINE n=1 t=5277300 pos=-18 ready=5280950
EV LINE n=2 t=5310650 pos=-16 ready=5314300
EV LINE n=3 t=5343950 pos=-14 ready=5347600
EV LINE n=4 t=5377300 pos=-12 ready=5380950
EV SCAN_DONE lines=5 t=5410650 pos=-12 aborted=0
> SCAN mode=sweep start=-20 step=4 lines=4 period=33333.333 delay=20000 id=4
OK SCAN mode=sweep t0=5430650 period=33333.333 start=-20 step=4 lines=4 id=4
EV LINE n=0 t=5430650 pos=-20
EV LINE n=1 t=5464000 pos=-16
EV LINE n=2 t=5497350 pos=-12
EV LINE n=3 t=5530650 pos=-8
EV SCAN_DONE lines=4 t=5564000 pos=-4 aborted=0
```

(From the simulated rig in `test/`, with the magnet 1500 microsteps from where the mirror powered up.)

## Syncing scan lines to the camera

The IMX219 free-runs and has no trigger input, so the camera is the master clock and the ESP32 follows it. The host does this:

1. **Map the clocks.** Send `PING` a few dozen times, timing each round trip on the host. The reply with the shortest round trip was delayed least; its `t` matches the middle of that round trip. Refresh it every few seconds and fit a line through the best samples to follow the slow rate difference between the two crystals (tens of ppm). Over USB this is good to about half a millisecond. For better, wire the `MOVING` or `TRIG` pin to a Jetson GPIO and timestamp its edges with the GPIO driver, which gets within tens of microseconds.
2. **Measure the frame period on the ESP32's clock.** Fit a line through about 100 frame timestamps (start of frame, from the camera driver), mapped to ESP32 time. Its slope is the period to send, e.g. `period=33333.297`.
3. **Put the ticks in the gap between exposures.** With a rolling shutter, the frame's first row starts exposing at `SOF − exposure` and the last row stops at `SOF + readout`. The mirror must hold still across that whole window, so pick `t0` just after the last row of some frame stops, and check that a move plus `settle` fits before the next frame's first row starts: for 2 microsteps that is about 0.7 ms + 3 ms. If it doesn't fit, use `period` = two frame periods and keep every second frame.
4. **Keep the lock.** Compare each `EV LINE` tick with the frame times. Correct a phase error with `NUDGE dt=...` and a rate error with `PERIOD us=...`.
5. **Match frames to lines.** In a stare scan, the frame for line `n` is the one whose whole exposure window lies between that line's `ready` and the next line's `t` (when the mirror moves again). Frames that straddle a move are discarded. In a sweep, the mirror angle when row `r` of a frame was exposed is `pos_n + (t_r − t_n) × step / period`, from the `EV LINE` just before `t_r`.

`mirrorctl.py --sync` prints the clock offset from step 1 against the PC's monotonic clock.
