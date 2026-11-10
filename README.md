# Motion Rig — Slider + Pan Controller

Firmware for a two-axis camera motion rig: a **linear slider (X)** with end-stop switches, and a **pan head (Y)**. You control everything with one rotary encoder and a 16×2 I²C LCD.

The rig does **one shot**: it moves smoothly from a **START** pose to an **END** pose, with the slider and pan moving together, over a time you choose. The whole workflow is:

> **Power on → aim the pan → click to home → set START → set END → set TIME → Run.**

Highlights:

- **One knob, three gestures:** turn = move / scroll, click = select / next, hold = cancel.
- **No step sizes to set:** turn slowly for fine moves and spin fast for big ones.
- **Ease in/out, back-and-forth loop, start delay** (off, 3, 5 or 10 s).
- **Checks before every run:** a shot that is too fast for the motors is refused, with the shortest time that would work.
- **Safe by default:** the rig can't run until homed, a limit-switch hit forces a re-home, and homing stops if the motor runs the wrong way. Stops are always smooth, and the controller keeps running if the LCD drops off the bus.

- **MCU:** ESP32-C3 SuperMini
- **Framework:** Arduino on PlatformIO (`espressif32`)
- **Source:** all firmware is in [src/main.cpp](src/main.cpp)

---

## Contents

1. [Hardware](#hardware)
2. [Wiring / pin map](#wiring--pin-map)
3. [Building and flashing](#building-and-flashing)
4. [First power-up checklist](#first-power-up-checklist)
5. [Controls](#controls)
6. [Using the rig](#using-the-rig)
7. [Screen reference](#screen-reference)
8. [How it works](#how-it-works)
9. [Configuration constants](#configuration-constants)
10. [Persistent storage](#persistent-storage)
11. [Troubleshooting](#troubleshooting)
12. [Known limitations](#known-limitations)

---

## Hardware

| Part | Notes |
|---|---|
| ESP32-C3 SuperMini | Programmed and powered over its native USB-C (USB-Serial-JTAG). |
| CNC shield + 2 stepper drivers | A4988/DRV8825-style STEP/DIR drivers. Microstepping set to **16×**. |
| Stepper X (slider) | 200 steps/rev, 20-tooth GT2 pulley → 40 mm per rev → **80 steps/mm**. |
| Stepper Y (pan) | 200 steps/rev, 16× microstep, **4:1 reduction** → 12 800 steps per 360°. |
| 2 × limit switches | MIN and MAX ends of the slider. 3-wire modules or bare switches to GND. |
| KY-040 rotary encoder | With push button. |
| 16×2 HD44780 LCD + I²C backpack | Auto-detected at `0x27` or `0x3F`. |

If your mechanics are different (pulley, gearing, microstepping), change the constants in [Configuration constants](#configuration-constants).

---

## Wiring / pin map

| Function | GPIO | Notes |
|---|---|---|
| X STEP | 0 | Slider driver |
| X DIR | 1 | |
| Y STEP | 3 | Pan driver |
| Y DIR | 4 | |
| Encoder CLK (A) | 7 | Internal pull-up, interrupt |
| Encoder DT (B) | 10 | Internal pull-up, interrupt |
| Encoder SW | 2 | Internal pull-up, active LOW |
| I²C SDA | 8 | LCD |
| I²C SCL | 9 | LCD |
| Limit MIN | 20 | Internal pull-up, active LOW by default |
| Limit MAX | 21 | Internal pull-up, active LOW by default |

### Wiring notes

- **Common ground.** The ESP32-C3, CNC shield logic, encoder, LCD and switches must all share GND.
- **Driver enable.** The firmware has no ENABLE pin. The drivers' EN line must be held at its *enabled* level (usually LOW) by the shield or by a jumper to GND, or the motors won't hold or move.
- **Strapping pins.** GPIO2, GPIO8 and GPIO9 are ESP32-C3 boot-strapping pins. GPIO9 is also the SuperMini's BOOT button.
  - Don't hold the encoder button down while the board powers up or resets.
  - Anything on GPIO8/GPIO9 must not pull them LOW at boot. Standard I²C pull-ups are fine.
- **GPIO20/21** are the C3's UART0 pins. They're free for the limit switches because serial output goes over USB instead (see `ARDUINO_USB_CDC_ON_BOOT` below).
- **Cable routing.** Keep the SDA/SCL and limit-switch wires short and away from the motor cables. The firmware filters out noise, but less noise is always better.

---

## Building and flashing

### `platformio.ini`

```ini
[env:esp32-c3-devkitm-1]
platform = espressif32
board = esp32-c3-devkitm-1
framework = arduino
monitor_speed = 115200
upload_port = COM5
monitor_port = COM5
lib_deps =
    waspinator/AccelStepper@^1.64
    marcoschwartz/LiquidCrystal_I2C@^1.1.4

build_flags =
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
```

**Why the two USB flags matter.** The SuperMini has no USB-UART chip. Its USB-C port goes straight to the C3's built-in USB-Serial-JTAG peripheral.

- `ARDUINO_USB_CDC_ON_BOOT=1` sends `Serial` to that USB port.
- `ARDUINO_USB_MODE=1` picks the hardware CDC (`HWCDC`) driver. The C3 has no USB-OTG peripheral, so without this flag the core never declares `Serial`. The build then fails with `'Serial' was not declared in this scope`.

**Port.** `upload_port`/`monitor_port` are fixed to `COM5`, the port Windows gave the board. If your board comes up as a different COM port, change both lines. Delete them to let PlatformIO auto-detect.

### Commands

```bash
pio run                 # build
pio run -t upload       # build + flash
pio device monitor      # serial monitor @115200
pio run -t clean        # wipe build artefacts
```

> The `.pio/build/supermini` and `.pio/libdeps/supermini` folders are left over from an older env name. They do no harm and you can delete them.

### Entering the bootloader manually

If the upload can't find or open the port, the running firmware may be blocking USB. Force the bootloader:

1. Hold **BOOT**.
2. Tap **RESET**.
3. Release **BOOT** after about a second.

The board should now enumerate as a USB serial device (Espressif VID `303A`). Retry the upload.

---

## First power-up checklist

Do these in order before you film anything:

1. **LCD.** The display shows `MOTION RIG / Ready to set up` for about a second, then `Aim pan ahead / then click: home`. If it stays blank, see [Troubleshooting](#lcd-shows-nothing).
2. **Diagnostics.** On the `Aim pan ahead` screen, **hold** the button to open Diagnostics:
   - `MIN:0 MAX:0` with nothing pressed. Press each switch by hand: its value should go to `1`. If the values are inverted, set `LIMIT_ACTIVE_LEVEL` to `HIGH`.
   - `CLK DT SW` show the raw encoder pins. `SW` should go to `0` while the button is pressed. Turn slowly: `CLK` and `DT` should both toggle.

   Click to go back.
3. **First home.** Click. Because no travel length has been saved yet, the rig goes to MIN, then runs to MAX to **measure the travel**, then returns to MIN. It should finish with `READY`.
   - If it stops with `WRONG DIRECTION`, flip `X_DIR_INVERT` (or the switches are swapped).
4. **Directions.** Open **Set start**. Turn the knob clockwise: the slider should move **away from MIN**. Click, then turn: the pan should rotate the way you think of as positive. Fix either with `X_DIR_INVERT` / `Y_DIR_INVERT`.
5. **Encoder count.** If one click of the knob moves twice as far as expected, or skips, set `ENC_TRANSITIONS_PER_DETENT` to `2`. If it moves the wrong way, set `ENC_INVERT`.

---

## Controls

One encoder with a push button. Everywhere:

| Gesture | Meaning |
|---|---|
| **Turn** | Move the axis being set, scroll the menu, or change the time. |
| **Click** | Select / next / save. |
| **Hold** (≥ 0.7 s) | Cancel and go back. On the `Aim pan ahead` screen it opens Diagnostics. |

- There is no double-click. Every click acts straight away.
- Turning while the button is held down never counts as a click.
- During homing or a run, **any** click stops it.

### Screen indicators

- **Pop-up messages** (e.g. `START SAVED`, `READY`) replace the second line for about 1.2 s. Turning or pressing clears them early.
- **Error pop-ups** that you must not miss stay on screen until you click. These are `LIMIT HIT-REHOME`, `WRONG DIRECTION`, `HOME TIMEOUT`, `LIMIT WIRING?`, `RESET START/END` and `MIN TIME Ns`. That click only dismisses the message and does nothing else.
- **Limit indicator.** The top-right character of every screen shows `<` while MIN is pressed, `>` while MAX is pressed, and `#` when both are pressed (a wiring fault).

---

## Using the rig

### 1. Home

```
Aim pan ahead
then click: home
```

This screen appears at power-up and **whenever the slider isn't homed**. The firmware returns to it on its own after a limit-switch hit, for example.

1. **Point the pan head straight ahead**, at the same reference every time. The pan has no switch, so **wherever it points when you click becomes 0°**.
2. **Click.** The slider homes to MIN. If no travel length is saved, or you chose **Re-measure**, it also runs to MAX to measure the travel first.
3. When it's done you see `READY` and the main menu.

A click during homing stops it (`HOMING STOPPED`) and returns you to this screen.

### 2. Set START and END

Choose **Set start** (or **Set end**) from the menu.

```
>X  123.4mm START
 P   12.0°  next
```

1. If that pose was already saved, the rig first drives to it, so you can fine-tune instead of starting over.
2. **Turn** to move the slider (`>` on the X line).
3. **Click** to switch to the pan (`>` on the P line). The right-hand hint changes from `next` to `save`.
4. **Turn** to move the pan.
5. **Click** again to save (`START SAVED` / `END SAVED`). The menu cursor jumps to the next setup step: after START it moves to **Set end**, after END to **Time**.

**Hold** at any time to cancel without saving. The rig stays where it is.

**Speed-sensitive knob.** There's no step size to set, because the knob works it out from how fast you turn:

| How you turn | Slider per click | Pan per click |
|---|---|---|
| Slowly | 0.5 mm | 0.5° |
| Medium (clicks < 110 ms apart) | 2 mm | 2° |
| Fast spin (clicks < 40 ms apart) | 10 mm | 10° |

The target can only run a short way ahead of the carriage, so a hard spin can't queue up a long move. The slider is kept inside its measured travel, and the pan is limited to **±350°** so the cables can't wind up.

### 3. Set the time

Choose **Time: Ns**.

```
SHOT TIME
10 s  click=ok
```

- Turn slowly to change the time by **1 s** per click; spin fast to change it by **10 s** per click. Range **1–600 s**, default 10 s.
- **Click** saves; the cursor moves on to **Ease**.
- **Hold** cancels and restores the previous time.

### 4. Options

These are on the menu; click to toggle or cycle them. Each change is saved at once.

| Option | Effect |
|---|---|
| **Ease: ON/OFF** | ON (default): the move starts and ends gently (smoothstep). OFF: constant speed from start to end. |
| **Loop: ON/OFF** | ON: after reaching END, the rig goes back to START over the same time, then forward again, until you click. OFF: one pass, then `DONE`. |
| **Delay: OFF/3s/5s/10s** | Countdown after the rig reaches START, before the move begins. Gives you time to start recording or step out of frame. It only runs before the **first** pass of a loop. |

### 5. Run

Choose **Run shot**. The rig first checks the shot (see [Run checks](#run-checks)), then goes through three screens:

```
To start 62%          Start in 4s           RUN 3/10s
click to cancel       click to cancel       click to stop
```

1. **To start:** drives to START at half speed. The percentage is how much of that move is done.
2. **Start in:** the countdown, if a delay is set.
3. **RUN** (or **LOOP**): the shot itself, with elapsed and total seconds.

A click during the first two screens cancels straight away. A click during the shot makes the rig **decelerate smoothly** to rest (`stopping...`), then shows `STOPPED`. When a one-pass shot finishes, the rig glides to rest exactly on END and shows `DONE`.

### Re-home and Re-measure

- **Re-home:** goes back to the `Aim pan ahead` screen. Use it if you think steps were lost, or to set a new pan zero.
- **Re-measure:** the same, but the next home also measures the slider travel again. Use it after moving the rails or the switches.

Both take you to the home screen, which has no "back": once you pick one, you have to home (click) to return to the menu.

---

## Screen reference

### Main menu

The 2-line display shows two items at a time; `>` marks the selected one. Turn to scroll, click to select. There's no "back" from the main menu, because it's the top level.

| Item | Action |
|---|---|
| `Run shot` | Check and run the shot. |
| `Set start` / `Set start ok` | Set the START pose (`ok` once saved). |
| `Set end` / `Set end ok` | Set the END pose. |
| `Time: 10s` | Set the shot duration. |
| `Ease: ON` | Toggle ease in/out. |
| `Loop: OFF` | Toggle back-and-forth looping. |
| `Delay: OFF` | Cycle the start delay: OFF → 3 s → 5 s → 10 s. |
| `Re-home` | Home again (and set a new pan zero). |
| `Re-measure` | Home again and re-measure the slider travel. |

### Homing

```
HOMING
Seeking MIN
```

Stages: `Seeking MIN` → `Backing off` → (when measuring) `Measuring` → `Backing off` → `Returning`.

### Diagnostics

From the `Aim pan ahead` screen, **hold** the button. Any press returns to the home screen.

```
MIN:0 MAX:0
CLK1 DT1 SW1
```

Line 1 shows the debounced limit switches. Line 2 shows the raw encoder pins.

### Messages

| Message | Meaning / what to do |
|---|---|
| `READY` | Homing finished. |
| `START SAVED` / `END SAVED` | Pose saved. |
| `SET START+END` | Both poses must be set before a run. |
| `START = END` | The two poses are the same; there's nothing to move. |
| `MIN TIME Ns` | The shot is too fast for the motors. Set the time to at least *N* seconds, or turn Ease off (ease needs more peak speed). *(Stays until clicked.)* |
| `RESET START/END` | START or END lies outside the measured travel, probably after a Re-measure. Set them again. *(Stays until clicked.)* |
| `STOPPED` / `DONE` | The run was stopped / finished. |
| `HOMING STOPPED` | You clicked during homing. |
| `LIMIT HIT-REHOME` | A limit switch was hit while moving. The rig stopped and must be re-homed. *(Stays until clicked.)* |
| `WRONG DIRECTION` | Homing reached the wrong switch. Flip `X_DIR_INVERT` or swap the MIN/MAX switch wires. *(Stays until clicked.)* |
| `HOME TIMEOUT` | No switch was found within 5 minutes. Check the switch and the X direction. *(Stays until clicked.)* |
| `LIMIT WIRING?` | Both switches read as pressed. Check wiring or `LIMIT_ACTIVE_LEVEL`. *(Stays until clicked.)* |

---

## How it works

### Main loop

`loop()` runs these steps and never blocks:

1. `motionRun()` samples and debounces the limit switches (`pollLimits()`), acts on any trip (`guardLimits()`), then advances homing, the shot, or normal point-to-point moves.
2. `pollInput()` debounces the button, works out click / hold, and reads the encoder.
3. `handleInput()` deals with pop-up dismissal, then passes the input to the current screen.
4. If the slider isn't homed (and isn't homing), the screen is forced back to `Aim pan ahead`. Diagnostics is the one exception.
5. `draw()` redraws the screen buffer every 50 ms (idle) or every 200 ms (while moving).
6. `lcdHealth()` re-probes a failed LCD once a second, **only while idle**.
7. `lcdFlush()` sends only the characters that changed. It is limited to **2 I²C writes per loop while moving** and 16 while idle, so the LCD never delays step pulses.

### Encoder

Both encoder pins trigger an interrupt on every edge. A quadrature lookup table decodes the direction, which rejects contact bounce. Counts accumulate in an ISR-safe accumulator and become whole detents (`ENC_TRANSITIONS_PER_DETENT`). The time between detents decides the fine / medium / coarse step when setting poses, and 1 s / 10 s when setting the time.

### Homing (X only)

This is a non-blocking state machine running at `X_HOME_SPEED` (1200 steps/s ≈ 15 mm/s):

```
(pan position := 0°)
SEEK_MIN → BACKOFF_MIN (3 mm, until switch releases) → X = 0
   └─ measure: SEEK_MAX → BACKOFF_MAX → save travel = trip − 3 mm → RETURN to 0
```

- **Measuring happens automatically** when no travel is saved: on the first ever home, after a MAX hit, or after **Re-measure**.
- If both switches read as pressed at the start, it refuses to run (`LIMIT WIRING?`).
- **Wrong-direction guard.** When seeking MIN, the slider must never reach MAX. If MAX is pressed after it was clear, or after the slider has moved 5 mm, homing stops at once (`WRONG DIRECTION`). The same happens if MIN is hit while seeking MAX.
- It times out after 5 minutes (`HOME TIMEOUT`).

### Limits

- **Soft limits.** Once homed, the slider is clamped to 0 … measured travel. The pan is always clamped to ±`PAN_LIMIT_DEG` (350°).
- **Debounced switches.** A switch has to read pressed for 3 ms (`LIMIT_PRESS_US`) before it counts, and released for 10 ms (`LIMIT_RELEASE_US`). A noise spike from the motor wires can't abort a take.
- **Hard limits** (`guardLimits()`). The soft limits keep a homed slider off the switches, so a switch hit while moving towards it means steps were lost: the carriage was bumped, stalled, or moved by hand. Then:
  - X stops instantly. A run stops too.
  - The slider is marked **not homed** (`LIMIT HIT-REHOME`), and the rig returns to the `Aim pan ahead` screen.
  - If it was **MAX** and a travel was saved, the travel is erased, so the next home re-measures it.
- You can still move *away* from a pressed switch.

### Run checks

Before moving, **Run shot** refuses to start (with a message) unless:

1. START and END are both set (`SET START+END`).
2. With a measured travel, both poses are inside it (`RESET START/END`).
3. START and END differ (`START = END`).
4. The time is long enough for the motors (`MIN TIME Ns`). The shortest allowed time is the larger of:
   - **Speed:** peak speed ≤ 90 % of `X_VMAX`/`Y_VMAX`. With Ease on, the peak is 1.5× the average.
   - **Acceleration (Ease on):** the smoothstep's peak acceleration, 6 × distance / T², must stay within `X_ACC`/`Y_ACC`. That gives T ≥ √(6 × distance / acceleration).

### The shot

1. **GOTO.** An accelerated move to START at 50 % of max speed.
2. **DELAY.** On the first pass only, if set, a non-blocking countdown.
3. **RUN.** Time-based tracking. For the current time *t*, the firmware computes where each axis *should* be: linear, or smoothstep `3u² − 2u³` with Ease. It then sets each motor's speed to

   `feed-forward velocity + PLAY_KP × (ideal position − actual position)`

   capped at the axis maximum. This keeps both axes in step with the clock and with each other. For the first second (`PLAY_RAMP_MS`), speed builds up gradually.

   **Slew limiting.** The speed actually sent to each motor can change by no more than `X_ACC`/`Y_ACC` per second. So starts, stops and the turn-around in Loop mode are always ramps, never jolts.
4. **End.** When the time is up, both axes glide to rest on END (with a 3 s safety timeout). One pass → `DONE`. With Loop on, START and END swap and the next pass begins.
5. **User stop.** A click while running sets both target speeds to zero. The slew limiter decelerates the rig smoothly, and it shows `STOPPED` once it's still.

### LCD

At boot the firmware scans I²C for `0x27`, then `0x3F`. If it finds neither, it runs **headless**: everything still works (if you can't see the screen) and `No LCD found - running headless` is printed on serial.

**If the LCD fails mid-shoot** (loose wire, noise on the bus), the motion keeps running:

- Every I²C call is limited to 3 ms (`Wire.setTimeOut(3)`), so a dead bus can't freeze the loop.
- If a screen update takes far longer than normal, the LCD is marked failed and nothing more is sent to it.
- Once the rig is idle, the firmware probes the LCD once a second. When it answers again, it is re-initialised and redrawn.

### Safety summary

| Risk | Protection |
|---|---|
| Running before the rig knows where it is | Every screen except Diagnostics is blocked until homed. |
| Lost steps (bump, stall, hand-moved) | Any limit hit while moving un-homes the slider and forces a re-home. A MAX hit also forces a re-measure. |
| Motor wired backwards | Homing stops with `WRONG DIRECTION` instead of driving into the far end. |
| Shot too fast for the motors | Refused before moving, with the shortest time that would work. |
| Poses outside the travel | Refused before moving (`RESET START/END`). |
| Pan cable wind-up | Pan clamped to ±350°. |
| Runaway knob | The target is capped a short way ahead of the carriage. |
| Jerky starts, stops, turn-arounds | All shot speed changes are slew-limited to the axis acceleration. |
| Electrical noise on limit switches | 3 ms press / 10 ms release debounce. |
| Corrupt saved data | Checksum and sanity checks; bad data falls back to safe defaults. |
| LCD or I²C failure | Bus timeouts, auto-disable and recovery when idle. Motion never waits on the display. |

---

## Configuration constants

All of these are near the top of [src/main.cpp](src/main.cpp):

| Constant | Default | Meaning |
|---|---|---|
| `LCD_COLS` / `LCD_ROWS` | 16 / 2 | Display size. |
| `LCD_ROW_OFFSET` | `{0x00,0x40}` | DDRAM row addresses. |
| `LIMIT_ACTIVE_LEVEL` | `LOW` | Pin level that means "switch pressed". |
| `X_DIR_INVERT` / `Y_DIR_INVERT` | `false` | Reverse motor direction. |
| `ENC_INVERT` | `false` | Reverse encoder direction. |
| `ENC_TRANSITIONS_PER_DETENT` | 4 | Encoder edges per click (try 2 if clicks skip). |
| `X_STEPS_PER_MM` | 80 | Slider steps per mm. |
| `Y_STEPS_PER_DEG` | 12800/360 ≈ 35.56 | Pan steps per degree. |
| `X_VMAX` / `Y_VMAX` | 3000 steps/s | Max speed (≈ 37.5 mm/s, ≈ 84 °/s). |
| `X_ACC` / `Y_ACC` | 6000 steps/s² | Acceleration; also the slew limit during a shot. |
| `JOG_PCT` | 0.6 | Speed (fraction of max) while setting poses. |
| `PAN_LIMIT_DEG` | 350 | Pan travel limit either side of 0°. |
| `X_HOME_SPEED` | 1200 steps/s | Homing speed. |
| `X_BACKOFF_STEPS` | 240 (3 mm) | Distance to back off a switch. |
| `X_HOME_TIMEOUT_MS` | 300 000 | Homing timeout (5 min). |
| `PLAY_KP` | 6.0 | Shot position-correction gain. |
| `PLAY_RAMP_MS` | 1000 | Soft-start time for a shot. |
| `DELAY_S` | 0 / 3 / 5 / 10 s | Start-delay choices (index 0 = OFF). |
| `DEBOUNCE_MS` / `LONG_MS` | 25 / 700 | Button debounce and hold time. |
| `LIMIT_PRESS_US` / `LIMIT_RELEASE_US` | 3000 / 10000 µs | Limit-switch debounce. These are defined in the *Limit switches* section. |

The move to START always runs at 50 % of max speed. This is fixed in `gotoA()`.

---

## Persistent storage

Everything you set is kept in one record, `shot`, in ESP32 NVS (flash) under the namespace `rig`:

| Field | Contents |
|---|---|
| `sx`, `sy` | START pose: slider and pan, in steps. |
| `ex`, `ey` | END pose. |
| `xmax` | Measured slider travel, in steps. |
| `dur` | Shot time, 1–600 s. |
| `ease`, `loop`, `delayIdx` | Options. |
| `flags` | Which of START / END / travel are set. |
| `ck` | FNV-1a checksum of all of the above. |

- The record is saved straight away whenever something changes.
- On boot it's loaded only if the size matches, the checksum matches, and every field is in range. Otherwise the rig starts from **safe defaults**: no poses, 10 s, Ease ON, Loop OFF, Delay OFF, travel not measured.
- **Not saved:** the homed state and the pan zero. You home at every power-up, and the pan zero is wherever the pan points when you click.

Poses are stored in **steps**, not mm/degrees. If you change `X_STEPS_PER_MM`, `Y_STEPS_PER_DEG` or the microstepping, set START and END again.

### Upgrading from the keyframe firmware

This version uses a new namespace (`rig`) and doesn't read the old `slider` data. Keyframes, sequence slots and old settings are **not carried over**. Set up the shot from scratch after flashing. The old data is left in flash and does no harm.

---

## Troubleshooting

### Build: `'Serial' was not declared in this scope`
`-D ARDUINO_USB_MODE=1` is missing from `build_flags`. See [Building and flashing](#building-and-flashing).

### Build: dozens of `redefinition of ...` errors
The code has been pasted into `main.cpp` twice, so everything is defined twice. Look for a second copy of the header comment (often stuck to the closing `}` of `loop()`) and delete from there to the end of the file.

### Upload: `Please specify upload_port`
PlatformIO couldn't find the board. Set `upload_port` in `platformio.ini`, or pass `--upload-port COMx`.

### Upload: `Could not open COM5, the port is busy or doesn't exist`
Windows can't see the board, or something else has the port open.

1. Close any serial monitor that has the port open.
2. Run `pio device list`, or check Device Manager → *Ports (COM & LPT)*. Look for a device with VID `303A`.
3. Enter the bootloader manually (BOOT + RESET) and retry.
4. Swap the USB-C cable. Charge-only cables are the most common cause. Plug straight into the PC, not a hub.
5. Unplug the shield and peripherals, then retry. A short, or a strapping pin (GPIO2/8/9) held LOW, can stop the board from booting.
6. If the board comes back on a different COM number, update `upload_port`/`monitor_port`.

### LCD shows nothing
- Check the serial monitor for `No LCD found - running headless`. If it's there, check SDA = GPIO8, SCL = GPIO9, power and GND.
- If the backlight is on but no text shows, turn the contrast trimmer on the I²C backpack.

### LCD goes blank or freezes during a move
The firmware has detected a failing I²C bus and stopped writing to the LCD; motion continues. It retries once the rig is idle and redraws when the LCD answers. If this keeps happening, shorten the SDA/SCL wires, route them away from the motor cables, and check the backpack's pull-ups and its power.

### The pan angle of my shot is off after re-homing
The pan zero is set from wherever the camera points when you click on `Aim pan ahead`. If you aim it differently than last time, the saved START/END pan angles shift by the same amount. Always aim at the same reference, or set START and END again.

### Motor moves the wrong way / `WRONG DIRECTION` when homing
Flip `X_DIR_INVERT`. Homing always moves in the negative direction first, so it has to drive *towards* the MIN switch. If the motor direction is right, the MIN and MAX switch wires are swapped.

### `LIMIT HIT-REHOME` during shots, or motors skip steps
Lower `X_VMAX`/`Y_VMAX` or `X_ACC`/`Y_ACC`. Check the driver current setting, the motor supply voltage, the belt tension and the pulley set-screws.

---

## Known limitations

- **One shot only.** There's one START→END move; no multi-point paths or stored shot library.
- **No pan homing.** The pan has no switch. Its zero is set by hand at every home, and the poses depend on aiming it the same way each time.
- **Homing isn't saved.** You must home after every power cycle.
- **Single saved copy.** The shot is stored as one checksummed record. If power is lost *while* it's being saved, the checksum fails on the next boot and everything resets to defaults. Corrupt data is never used, but the setup has to be redone.
- **Fixed speeds outside the shot.** Pose setting runs at 60 % and the move to START at 50 % of max speed; there's no speed setting.
- **No pan protection beyond the ±350° clamp.** Lost steps are only detected on the slider (X).
- **Single-threaded stepping.** Step pulses come from `loop()`, so very high step rates depend on the loop running fast. The LCD write budget exists to protect this.
- **Hardware not yet verified on the bench.** The firmware hasn't yet been flashed and checked on the rig. Work through the [First power-up checklist](#first-power-up-checklist) before you trust any of the constants above.
